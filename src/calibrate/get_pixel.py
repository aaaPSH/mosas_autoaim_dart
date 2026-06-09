import cv2
import numpy as np
import sys
import tkinter as tk
from tkinter import filedialog

# 缩放与平移状态
zoom = 1.0
offset_x, offset_y = 0, 0
MIN_ZOOM, MAX_ZOOM = 0.1, 10.0
dragging = False
drag_start = (0, 0)
drag_offset = (0, 0)
view_w, view_h = 960, 720  # 窗口大小


def get_display_size(img, zoom):
    h, w = img.shape[:2]
    return int(w * zoom), int(h * zoom)


def render(img, zoom, offset_x, offset_y):
    dw, dh = get_display_size(img, zoom)
    resized = cv2.resize(img, (dw, dh), interpolation=cv2.INTER_LINEAR)
    # 计算可见区域
    x1 = max(0, offset_x)
    y1 = max(0, offset_y)
    x2 = min(dw, offset_x + view_w)
    y2 = min(dh, offset_y + view_h)
    if x2 <= x1 or y2 <= y1:
        return np.zeros((view_h, view_w, 3), dtype=np.uint8)
    crop = resized[y1:y2, x1:x2]
    canvas = np.zeros((view_h, view_w, 3), dtype=np.uint8)
    ch, cw = crop.shape[:2]
    canvas[0:ch, 0:cw] = crop
    return canvas


def clamp_offset(img, zoom):
    global offset_x, offset_y
    dw, dh = get_display_size(img, zoom)
    offset_x = max(0, min(offset_x, max(0, dw - view_w)))
    offset_y = max(0, min(offset_y, max(0, dh - view_h)))


def zoom_at(img, x, y, factor):
    global zoom, offset_x, offset_y
    old_zoom = zoom
    zoom = max(MIN_ZOOM, min(zoom * factor, MAX_ZOOM))
    scale = zoom / old_zoom
    offset_x = int((offset_x + x) * scale - x)
    offset_y = int((offset_y + y) * scale - y)
    clamp_offset(img, zoom)


def mouse_callback(event, x, y, flags, param):
    global offset_x, offset_y, dragging, drag_start, drag_offset

    img = param

    if event == cv2.EVENT_MOUSEWHEEL:
        zoom_at(img, x, y, 1.2 if flags > 0 else 1 / 1.2)

    elif event == cv2.EVENT_LBUTTONDOWN:
        dragging = True
        drag_start = (x, y)
        drag_offset = (offset_x, offset_y)

    elif event == cv2.EVENT_MOUSEMOVE and dragging:
        offset_x = drag_offset[0] + drag_start[0] - x
        offset_y = drag_offset[1] + drag_start[1] - y
        clamp_offset(img, zoom)

    elif event == cv2.EVENT_LBUTTONUP:
        dx = abs(x - drag_start[0])
        dy = abs(y - drag_start[1])
        dragging = False
        if dx < 5 and dy < 5:  # 移动很小算点击取色
            orig_x = int((offset_x + x) / zoom)
            orig_y = int((offset_y + y) / zoom)
            ih, iw = img.shape[:2]
            if 0 <= orig_x < iw and 0 <= orig_y < ih:
                b, g, r = img[orig_y, orig_x]
                print(f"坐标: ({orig_x}, {orig_y})  像素值: R={r} G={g} B={b}  缩放: {zoom:.1f}x")


def main():
    global view_w, view_h, zoom, offset_x, offset_y

    if len(sys.argv) < 2:
        print("用法: python get_pixel.py 3.jpg")
        print("滚轮缩放 | 左键拖动平移 | 左键点击取色 | +/- 键缩放 | r 重置 | q 退出")
        sys.exit(1)

    img = cv2.imread(sys.argv[1])
    if img is None:
        print(f"无法读取图片: {sys.argv[1]}")
        sys.exit(1)

    ih, iw = img.shape[:2]
    view_w = min(960, iw)
    view_h = min(720, ih)

    # 初始自适应缩放
    zoom = min(view_w / iw, view_h / ih, 1.0)
    dw, dh = get_display_size(img, zoom)
    offset_x = max(0, (dw - view_w) // 2)
    offset_y = max(0, (dh - view_h) // 2)

    win = "image [scroll=zoom drag=pan click=color +/-=zoom r=reset q=quit]"
    cv2.namedWindow(win)
    cv2.setMouseCallback(win, mouse_callback, img)
    print("滚轮缩放 | 左键拖动平移 | 左键点击取色 | +/- 键缩放 | r 重置 | q 退出")

    while True:
        canvas = render(img, zoom, offset_x, offset_y)
        info = f"{zoom:.1f}x  ({iw}x{ih})"
        cv2.putText(canvas, info, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 1)
        cv2.imshow(win, canvas)

        key = cv2.waitKey(30) & 0xFF
        if key == ord("q"):
            break
        elif key == ord("+") or key == ord("="):
            zoom_at(img, view_w // 2, view_h // 2, 1.2)
        elif key == ord("-"):
            zoom_at(img, view_w // 2, view_h // 2, 1 / 1.2)
        elif key == ord("r"):
            zoom = min(view_w / iw, view_h / ih, 1.0)
            dw, dh = get_display_size(img, zoom)
            offset_x = max(0, (dw - view_w) // 2)
            offset_y = max(0, (dh - view_h) // 2)

    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
