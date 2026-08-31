import gc
import time

import aicube
import image
import nncase_runtime as nn
import ujson
import ulab.numpy as np

from machine import FPIOA, UART, TOUCH
from libs.PipeLine import ScopedTiming
from libs.Utils import *
from media.display import *
from media.media import *
from media.sensor import *


display_mode = "lcd"

if display_mode == "lcd":
    DISPLAY_WIDTH = ALIGN_UP(800, 16)
    DISPLAY_HEIGHT = 480
else:
    DISPLAY_WIDTH = ALIGN_UP(1920, 16)
    DISPLAY_HEIGHT = 1080

OUT_RGB888P_WIDTH = ALIGN_UP(640, 16)
OUT_RGB888P_HEIGHT = 360

# AI 实际检测区域，坐标基于原始 640x360 图像。
# 对应 LCD：x=19, y=215, w=765, h=42。
CROP_X = 15
CROP_Y = 161
CROP_W = 612
CROP_H = 32

root_path = "/sdcard/mp_deployment_source/"
config_path = root_path + "deploy_config.json"

debug_mode = 1
UART_SEND_INTERVAL_MS = 20

BALL_LABEL = "小钢球"

# -5cm、O点、+5cm 的 AI 图像坐标
NEG5_X = 186
NEG5_Y = 173

ZERO_X = 318
ZERO_Y = 171

POS5_X = 446
POS5_Y = 179

MAX_LINE_DISTANCE_PX = 30
FILTER_ALPHA = 0.65


def two_side_pad_param(input_size, output_size):
    ratio_w = output_size[0] / input_size[0]
    ratio_h = output_size[1] / input_size[1]
    ratio = min(ratio_w, ratio_h)

    new_w = int(ratio * input_size[0])
    new_h = int(ratio * input_size[1])

    dw = (output_size[0] - new_w) / 2
    dh = (output_size[1] - new_h) / 2

    top = int(round(dh - 0.1))
    bottom = int(round(dh + 0.1))
    left = int(round(dw - 0.1))
    right = int(round(dw - 0.1))

    return top, bottom, left, right, ratio


def read_deploy_config(path):
    with open(path, "r") as json_file:
        try:
            return ujson.load(json_file)
        except ValueError as e:
            print("JSON parse error:", e)
            return None


def init_uart2():
    fpioa = FPIOA()

    fpioa.set_function(11, FPIOA.UART2_TXD)
    fpioa.set_function(12, FPIOA.UART2_RXD)

    uart2 = UART(UART.UART2, 115200)
    print("UART2 init success")

    return uart2


def send_ball_pos(uart, found, pos_0p1cm=0):
    """
    AA 55 found pos_low pos_high 00 00 checksum

    pos_0p1cm:
        0   = O点
        50  = +5.0cm
        -50 = -5.0cm
    """

    pos = int(pos_0p1cm) & 0xFFFF

    pos_l = pos & 0xFF
    pos_h = (pos >> 8) & 0xFF

    checksum = (found + pos_l + pos_h) & 0xFF

    uart.write(bytes([
        0xAA,
        0x55,
        found,
        pos_l,
        pos_h,
        0x00,
        0x00,
        checksum
    ]))


def point_line_distance_sq(x, y):
    dx = POS5_X - NEG5_X
    dy = POS5_Y - NEG5_Y

    line_len_sq = dx * dx + dy * dy

    if line_len_sq == 0:
        return 99999999

    cross = (
        (x - NEG5_X) * dy
        - (y - NEG5_Y) * dx
    )

    return (cross * cross) / line_len_sq


def pixel_to_cm(center_x, center_y):
    dx = POS5_X - NEG5_X
    dy = POS5_Y - NEG5_Y

    line_len_sq = dx * dx + dy * dy

    if line_len_sq == 0:
        return 0.0

    zero_t = (
        (ZERO_X - NEG5_X) * dx
        + (ZERO_Y - NEG5_Y) * dy
    ) / line_len_sq

    ball_t = (
        (center_x - NEG5_X) * dx
        + (center_y - NEG5_Y) * dy
    ) / line_len_sq

    return (ball_t - zero_t) * 10.0


def get_ball_target(det_boxes, labels):
    best_box = None
    best_confidence = -1.0

    max_distance_sq = (
        MAX_LINE_DISTANCE_PX
        * MAX_LINE_DISTANCE_PX
    )

    for box in det_boxes:
        class_id = box[0]
        confidence = box[1]

        x1 = box[2]
        y1 = box[3]
        x2 = box[4]
        y2 = box[5]

        if labels[class_id] != BALL_LABEL:
            continue

        center_x = int((x1 + x2) / 2)
        center_y = int((y1 + y2) / 2)

        if point_line_distance_sq(
            center_x,
            center_y
        ) > max_distance_sq:
            continue

        if confidence > best_confidence:
            best_confidence = confidence
            best_box = box

    return best_box


def restore_crop_boxes(det_boxes_crop):
    """将裁剪区域检测框还原为原始 640x360 坐标。"""

    det_boxes_full = []

    for box in det_boxes_crop:
        x1 = box[2] + CROP_X
        y1 = box[3] + CROP_Y
        x2 = box[4] + CROP_X
        y2 = box[5] + CROP_Y

        # 过滤任何超出真实裁剪区域的框。
        if x1 < CROP_X or y1 < CROP_Y:
            continue

        if x2 > CROP_X + CROP_W:
            continue

        if y2 > CROP_Y + CROP_H:
            continue

        det_boxes_full.append([
            box[0],
            box[1],
            x1,
            y1,
            x2,
            y2
        ])

    return det_boxes_full


def draw_marker(osd_img, x, y, color):
    lcd_x = int(
        x * DISPLAY_WIDTH // OUT_RGB888P_WIDTH
    )

    lcd_y = int(
        y * DISPLAY_HEIGHT // OUT_RGB888P_HEIGHT
    )

    osd_img.draw_circle(
        lcd_x,
        lcd_y,
        7,
        color=color,
        thickness=2
    )


def detection():
    global ZERO_X, ZERO_Y

    print("det_infer start")

    deploy_conf = read_deploy_config(config_path)

    if deploy_conf is None:
        return -1

    kmodel_name = deploy_conf["kmodel_path"]
    labels = deploy_conf["categories"]
    confidence_threshold = deploy_conf["confidence_threshold"]
    nms_threshold = deploy_conf["nms_threshold"]
    img_size = deploy_conf["img_size"]
    num_classes = deploy_conf["num_classes"]
    nms_option = deploy_conf["nms_option"]
    model_type = deploy_conf["model_type"]

    if model_type != "AnchorBaseDet":
        print("Only AnchorBaseDet is supported")
        return -1

    anchors = (
        deploy_conf["anchors"][0]
        + deploy_conf["anchors"][1]
        + deploy_conf["anchors"][2]
    )

    kmodel_frame_size = img_size
    crop_frame_size = [CROP_W, CROP_H]

    strides = [8, 16, 32]

    top, bottom, left, right, ratio = two_side_pad_param(
        crop_frame_size,
        kmodel_frame_size
    )

    uart2 = init_uart2()

    # 触摸屏初始化
    tp = TOUCH(0)
    touch_down = False

    kpu = nn.kpu()
    kpu.load_kmodel(root_path + kmodel_name)

    ai2d = nn.ai2d()

    ai2d.set_dtype(
        nn.ai2d_format.NCHW_FMT,
        nn.ai2d_format.NCHW_FMT,
        np.uint8,
        np.uint8
    )

    # 从原始 640x360 图像中裁剪水管区域，再送入模型。
    ai2d.set_crop_param(
        True,
        CROP_X,
        CROP_Y,
        CROP_W,
        CROP_H
    )

    ai2d.set_pad_param(
        True,
        [0, 0, 0, 0, top, bottom, left, right],
        0,
        [114, 114, 114]
    )

    ai2d.set_resize_param(
        True,
        nn.interp_method.tf_bilinear,
        nn.interp_mode.half_pixel
    )

    ai2d_builder = ai2d.build(
        [
            1,
            3,
            OUT_RGB888P_HEIGHT,
            OUT_RGB888P_WIDTH
        ],
        [
            1,
            3,
            kmodel_frame_size[1],
            kmodel_frame_size[0]
        ]
    )

    sensor = Sensor()
    sensor.reset()

    sensor.set_hmirror(False)
    sensor.set_vflip(False)

    sensor.set_framesize(
        width=DISPLAY_WIDTH,
        height=DISPLAY_HEIGHT,
        chn=CAM_CHN_ID_0
    )

    sensor.set_pixformat(
        PIXEL_FORMAT_YUV_SEMIPLANAR_420,
        chn=CAM_CHN_ID_0
    )

    sensor.set_framesize(
        width=OUT_RGB888P_WIDTH,
        height=OUT_RGB888P_HEIGHT,
        chn=CAM_CHN_ID_2
    )

    sensor.set_pixformat(
        PIXEL_FORMAT_RGB_888_PLANAR,
        chn=CAM_CHN_ID_2
    )

    sensor_bind_info = sensor.bind_info(
        x=0,
        y=0,
        chn=CAM_CHN_ID_0
    )

    Display.bind_layer(
        **sensor_bind_info,
        layer=Display.LAYER_VIDEO1
    )

    if display_mode == "lcd":
        Display.init(Display.ST7701, to_ide=True)
    else:
        Display.init(Display.LT9611, to_ide=True)

    osd_img = image.Image(
        DISPLAY_WIDTH,
        DISPLAY_HEIGHT,
        image.ARGB8888
    )

    MediaManager.init()
    sensor.run()

    data = np.ones(
        (
            1,
            3,
            kmodel_frame_size[1],
            kmodel_frame_size[0]
        ),
        dtype=np.uint8
    )

    ai2d_output_tensor = nn.from_numpy(data)

    last_uart_send_time = time.ticks_ms()
    filtered_pos_cm = 0.0
    clock = time.clock()

    try:
        while True:
            clock.tick()

            with ScopedTiming("total", debug_mode > 0):
                rgb888p_img = sensor.snapshot(
                    chn=CAM_CHN_ID_2
                )

                if rgb888p_img.format() != image.RGBP888:
                    rgb888p_img = None
                    continue

                ai2d_input = rgb888p_img.to_numpy_ref()
                ai2d_input_tensor = nn.from_numpy(ai2d_input)

                ai2d_builder.run(
                    ai2d_input_tensor,
                    ai2d_output_tensor
                )

                kpu.set_input_tensor(0, ai2d_output_tensor)
                kpu.run()

                results = []

                for i in range(kpu.outputs_size()):
                    out_data = kpu.get_output_tensor(i)
                    result = out_data.to_numpy()

                    result = result.reshape((
                        result.shape[0]
                        * result.shape[1]
                        * result.shape[2]
                        * result.shape[3]
                    ))

                    del out_data
                    results.append(result)

                det_boxes_crop = aicube.anchorbasedet_post_process(
                    results[0],
                    results[1],
                    results[2],
                    kmodel_frame_size,
                    crop_frame_size,
                    strides,
                    num_classes,
                    confidence_threshold,
                    nms_threshold,
                    anchors,
                    nms_option
                )

                # 后续标定和显示继续使用原始 640x360 坐标。
                det_boxes = restore_crop_boxes(det_boxes_crop)

                osd_img.clear()

                # 点击屏幕：重新设定 O 点
                touch_points = tp.read(1)

                if touch_points != ():
                    if not touch_down:
                        touch_x = touch_points[0].x
                        touch_y = touch_points[0].y

                        # 触摸屏 800x480 坐标转 AI 640x360 坐标
                        ZERO_X = int(
                            touch_x
                            * OUT_RGB888P_WIDTH
                            / DISPLAY_WIDTH
                        )

                        ZERO_Y = int(
                            touch_y
                            * OUT_RGB888P_HEIGHT
                            / DISPLAY_HEIGHT
                        )

                        filtered_pos_cm = 0.0
                        touch_down = True

                        print(
                            "New O: X={} Y={}".format(
                                ZERO_X,
                                ZERO_Y
                            )
                        )
                else:
                    touch_down = False

                now = time.ticks_ms()

                should_send = (
                    time.ticks_diff(
                        now,
                        last_uart_send_time
                    ) >= UART_SEND_INTERVAL_MS
                )

                neg5_lcd_x = int(
                    NEG5_X
                    * DISPLAY_WIDTH
                    // OUT_RGB888P_WIDTH
                )

                neg5_lcd_y = int(
                    NEG5_Y
                    * DISPLAY_HEIGHT
                    // OUT_RGB888P_HEIGHT
                )

                pos5_lcd_x = int(
                    POS5_X
                    * DISPLAY_WIDTH
                    // OUT_RGB888P_WIDTH
                )

                pos5_lcd_y = int(
                    POS5_Y
                    * DISPLAY_HEIGHT
                    // OUT_RGB888P_HEIGHT
                )

                osd_img.draw_line(
                    neg5_lcd_x,
                    neg5_lcd_y,
                    pos5_lcd_x,
                    pos5_lcd_y,
                    color=(0, 255, 255),
                    thickness=2
                )

                # 蓝：-5cm；黄：当前 O 点；绿：+5cm
                draw_marker(
                    osd_img,
                    NEG5_X,
                    NEG5_Y,
                    (0, 0, 255)
                )

                draw_marker(
                    osd_img,
                    ZERO_X,
                    ZERO_Y,
                    (255, 255, 0)
                )

                draw_marker(
                    osd_img,
                    POS5_X,
                    POS5_Y,
                    (0, 255, 0)
                )

                target = get_ball_target(
                    det_boxes,
                    labels
                )

                if target is not None:
                    x1 = target[2]
                    y1 = target[3]
                    x2 = target[4]
                    y2 = target[5]

                    center_x = int((x1 + x2) / 2)
                    center_y = int((y1 + y2) / 2)

                    raw_pos_cm = pixel_to_cm(
                        center_x,
                        center_y
                    )

                    filtered_pos_cm = (
                        FILTER_ALPHA * filtered_pos_cm
                        + (1.0 - FILTER_ALPHA)
                        * raw_pos_cm
                    )

                    pos_0p1cm = int(
                        filtered_pos_cm * 10
                    )

                    x = int(
                        x1 * DISPLAY_WIDTH
                        // OUT_RGB888P_WIDTH
                    )

                    y = int(
                        y1 * DISPLAY_HEIGHT
                        // OUT_RGB888P_HEIGHT
                    )

                    w = int(
                        (x2 - x1)
                        * DISPLAY_WIDTH
                        // OUT_RGB888P_WIDTH
                    )

                    h = int(
                        (y2 - y1)
                        * DISPLAY_HEIGHT
                        // OUT_RGB888P_HEIGHT
                    )

                    cx = int(
                        center_x
                        * DISPLAY_WIDTH
                        // OUT_RGB888P_WIDTH
                    )

                    cy = int(
                        center_y
                        * DISPLAY_HEIGHT
                        // OUT_RGB888P_HEIGHT
                    )

                    osd_img.draw_rectangle(
                        x,
                        y,
                        w,
                        h,
                        color=(255, 0, 0),
                        thickness=2
                    )

                    osd_img.draw_circle(
                        cx,
                        cy,
                        6,
                        color=(255, 0, 0),
                        thickness=2
                    )

                    text = "X:{} Y:{} P:{:.2f}cm".format(
                        center_x,
                        center_y,
                        filtered_pos_cm
                    )

                    osd_img.draw_string_advanced(
                        20,
                        20,
                        28,
                        text,
                        color=(255, 255, 0)
                    )

                    if should_send:
                        send_ball_pos(
                            uart2,
                            1,
                            pos_0p1cm
                        )

                        last_uart_send_time = now

                else:
                    osd_img.draw_string_advanced(
                        20,
                        20,
                        28,
                        "Ball lost",
                        color=(255, 0, 0)
                    )

                    if should_send:
                        send_ball_pos(uart2, 0, 0)
                        last_uart_send_time = now

                osd_img.draw_string_advanced(
                    20,
                    55,
                    24,
                    "Touch ball to set O",
                    color=(0, 255, 0)
                )

                osd_img.draw_string_advanced(
                    20,
                    85,
                    24,
                    "O:{} {}".format(
                        ZERO_X,
                        ZERO_Y
                    ),
                    color=(255, 255, 0)
                )

                osd_img.draw_string_advanced(
                    20,
                    115,
                    24,
                    "FPS:{:.1f}".format(clock.fps()),
                    color=(255, 0, 0)
                )

                # 绿框：实际送入 AI 的裁剪区域。
                crop_lcd_x = CROP_X * DISPLAY_WIDTH // OUT_RGB888P_WIDTH
                crop_lcd_y = CROP_Y * DISPLAY_HEIGHT // OUT_RGB888P_HEIGHT
                crop_lcd_w = CROP_W * DISPLAY_WIDTH // OUT_RGB888P_WIDTH
                crop_lcd_h = CROP_H * DISPLAY_HEIGHT // OUT_RGB888P_HEIGHT

                osd_img.draw_rectangle(
                    crop_lcd_x,
                    crop_lcd_y,
                    crop_lcd_w,
                    crop_lcd_h,
                    color=(0, 255, 0),
                    thickness=2
                )

                Display.show_image(
                    osd_img,
                    0,
                    0,
                    Display.LAYER_OSD3
                )

                del ai2d_input_tensor
                rgb888p_img = None

                gc.collect()

    except KeyboardInterrupt:
        print("Program stopped")

    finally:
        sensor.stop()
        Display.deinit()
        MediaManager.deinit()

        del ai2d_output_tensor

        gc.collect()
        time.sleep_ms(100)
        nn.shrink_memory_pool()

        print("det_infer end")

    return 0


if __name__ == "__main__":
    detection()
