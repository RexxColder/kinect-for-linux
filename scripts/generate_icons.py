#!/usr/bin/env python3
"""Generate all 5 k4w-ctl icons at 1024x1024 RGBA."""
from PIL import Image, ImageDraw, ImageFont
import math, os

SIZE = 1536
OUT = os.path.join(os.path.dirname(__file__), "..", "src", "ctl", "icons")
SCALE = SIZE / 1024  # 1.5x — Kinect fills same proportion as original 1024 design
os.makedirs(OUT, exist_ok=True)

# Colors
BG = (0, 0, 0, 0)          # transparent
BODY = (59, 59, 59, 255)    # dark gray sensor bar
BODY2 = (50, 50, 50, 255)   # slightly darker for base
LENS = (230, 230, 230, 255) # white lenses
LENS_INNER = (100, 100, 100, 255)  # lens center dot
NECK = (80, 80, 80, 255)    # connector
LED_GREEN = (76, 175, 80, 255)
LED_RED = (211, 47, 47, 255)
LED_YELLOW = (255, 213, 79, 255)
WHITE = (255, 255, 255, 255)
DARK_LINE = (80, 80, 80, 255)


def draw_kinect_base(draw, cx, cy, scale=1.0):
    """Draw the Kinect sensor bar + base. cx,cy = center of the sensor bar."""
    s = scale

    # ── Sensor bar (wide rounded rect) ──
    bar_w = int(520 * s)
    bar_h = int(140 * s)
    bar_x0 = cx - bar_w // 2
    bar_y0 = cy - bar_h // 2
    bar_x1 = cx + bar_w // 2
    bar_y1 = cy + bar_h // 2
    r = int(20 * s)  # corner radius
    draw.rounded_rectangle([bar_x0, bar_y0, bar_x1, bar_y1], radius=r, fill=BODY)

    # Top highlight line
    draw.line([(bar_x0 + r, bar_y0 + 3), (bar_x1 - r, bar_y0 + 3)],
              fill=(80, 80, 80, 255), width=2)

    # Bottom shadow line
    draw.line([(bar_x0 + r, bar_y1 - 2), (bar_x1 - r, bar_y1 - 2)],
              fill=(40, 40, 40, 255), width=2)

    # ── 3 Lenses ──
    lens_y = cy
    lens_spacing = int(110 * s)
    lens_r = int(32 * s)
    lens_inner_r = int(12 * s)

    for i, offset in enumerate([-lens_spacing, 0, lens_spacing]):
        lx = cx + offset
        # Outer ring
        draw.ellipse([lx - lens_r, lens_y - lens_r, lx + lens_r, lens_y + lens_r],
                     fill=LENS)
        # Inner dark circle
        draw.ellipse([lx - lens_inner_r, lens_y - lens_inner_r,
                      lx + lens_inner_r, lens_y + lens_inner_r],
                     fill=LENS_INNER)

    # ── Neck / connector ──
    neck_w = int(60 * s)
    neck_h = int(50 * s)
    neck_x0 = cx - neck_w // 2
    neck_y0 = bar_y1
    neck_x1 = cx + neck_w // 2
    neck_y1 = bar_y1 + neck_h
    draw.rounded_rectangle([neck_x0, neck_y0, neck_x1, neck_y1],
                           radius=int(6 * s), fill=NECK)

    # ── Base / pedestal ──
    base_w = int(340 * s)
    base_h = int(90 * s)
    base_cx = cx
    base_cy = neck_y1 + base_h // 2 - int(10 * s)
    base_x0 = base_cx - base_w // 2
    base_y0 = base_cy - base_h // 2
    base_x1 = base_cx + base_w // 2
    base_y1 = base_cy + base_h // 2
    base_r = int(40 * s)
    draw.rounded_rectangle([base_x0, base_y0, base_x1, base_y1],
                           radius=base_r, fill=BODY2)

    # Base top highlight
    draw.line([(base_x0 + base_r + 10, base_y0 + 3),
               (base_x1 - base_r - 10, base_y0 + 3)],
              fill=(70, 70, 70, 255), width=2)

    # Base bottom shadow
    draw.line([(base_x0 + base_r + 10, base_y1 - 2),
               (base_x1 - base_r - 10, base_y1 - 2)],
              fill=(35, 35, 35, 255), width=2)

    return bar_x0, bar_y0, bar_x1, bar_y1, base_x0, base_y0, base_x1, base_y1


def draw_led(draw, x, y, radius, color):
    """Draw a colored LED dot."""
    # Outer glow
    glow_r = radius + int(radius * 0.4)
    glow_color = (color[0], color[1], color[2], 60)
    draw.ellipse([x - glow_r, y - glow_r, x + glow_r, y + glow_r], fill=glow_color)
    # Main dot
    draw.ellipse([x - radius, y - radius, x + radius, y + radius], fill=color)
    # Highlight
    hl_r = int(radius * 0.35)
    hl_x = x - int(radius * 0.25)
    hl_y = y - int(radius * 0.25)
    draw.ellipse([hl_x - hl_r, hl_y - hl_r, hl_x + hl_r, hl_y + hl_r],
                 fill=(255, 255, 255, 120))


def draw_tilt_arrows(draw, cx, cy, base_coords, scale=1.0):
    """Draw dotted outline of tilted Kinect + curved arrows."""
    s = scale
    _, _, _, _, base_x0, base_y0, base_x1, base_y1 = base_coords
    bar_top = base_coords[1]  # bar_y0

    # Draw two tilted outlines (dotted) + curved arrows on right
    arrow_x = base_x1 + int(30 * s)
    arrow_top = bar_top - int(20 * s)
    arrow_bot = base_y1 + int(20 * s)
    arrow_mid = (arrow_top + arrow_bot) // 2

    # Curved arrow line
    arrow_color = (100, 100, 100, 200)
    # Simple arc-like path with arrowheads
    steps = 40
    for i in range(steps):
        t = i / steps
        # Right side arc
        x = arrow_x + int(20 * s * math.sin(t * math.pi))
        y = int(arrow_top + t * (arrow_bot - arrow_top))
        if i > 0:
            px = arrow_x + int(20 * s * math.sin((i - 1) / steps * math.pi))
            py = int(arrow_top + (i - 1) / steps * (arrow_bot - arrow_top))
            draw.line([(px, py), (x, y)], fill=arrow_color, width=int(3 * s))

    # Arrowhead top (pointing up)
    ah = int(20 * s)
    draw.polygon([
        (arrow_x, arrow_top),
        (arrow_x - ah // 2, arrow_top + ah),
        (arrow_x + ah // 2, arrow_top + ah),
    ], fill=arrow_color)

    # Arrowhead bottom (pointing down)
    draw.polygon([
        (arrow_x, arrow_bot),
        (arrow_x - ah // 2, arrow_bot - ah),
        (arrow_x + ah // 2, arrow_bot - ah),
    ], fill=arrow_color)

    # Dotted tilted outlines (upper and lower position)
    for tilt_angle in [-15, 15]:
        # Simplified: draw a dashed rectangle at an offset
        offset_y = int(tilt_angle * 1.5 * s)
        dash_len = int(12 * s)
        gap = int(8 * s)
        rect_w = int(400 * s)
        rect_h = int(100 * s)
        rx = cx - rect_w // 2
        ry = cy - rect_h // 2 + offset_y

        # Top and bottom edges
        for edge_y in [ry, ry + rect_h]:
            x = rx
            while x < rx + rect_w:
                draw.line([(x, edge_y), (min(x + dash_len, rx + rect_w), edge_y)],
                          fill=(120, 120, 120, 140), width=int(2 * s))
                x += dash_len + gap

        # Left and right edges
        for edge_x in [rx, rx + rect_w]:
            y = ry
            while y < ry + rect_h:
                draw.line([(edge_x, y), (edge_x, min(y + dash_len, ry + rect_h))],
                          fill=(120, 120, 120, 140), width=int(2 * s))
                y += dash_len + gap


def gen_no_dot():
    """Icon 1: Kinect WITHOUT any LED dot — for app icon, dock, motor upper."""
    img = Image.new("RGBA", (SIZE, SIZE), BG)
    draw = ImageDraw.Draw(img)
    cx, cy = SIZE // 2, SIZE // 2 - int(40 * SCALE)
    draw_kinect_base(draw, cx, cy, SCALE)
    path = os.path.join(OUT, "kinection.png")
    img.save(path, "PNG")
    print(f"  {path}")


def gen_tilt():
    """Icon 2: Kinect with tilt arrows — for motor section lower."""
    img = Image.new("RGBA", (SIZE, SIZE), BG)
    draw = ImageDraw.Draw(img)
    cx, cy = SIZE // 2, SIZE // 2 - int(20 * SCALE)
    coords = draw_kinect_base(draw, cx, cy, SCALE)
    draw_tilt_arrows(draw, cx, cy, coords, SCALE)
    path = os.path.join(OUT, "kinection-tilt.png")
    img.save(path, "PNG")
    print(f"  {path}")


def gen_led(color, filename, label):
    """Icons 3-5: Kinect with colored LED dot."""
    img = Image.new("RGBA", (SIZE, SIZE), BG)
    draw = ImageDraw.Draw(img)
    cx, cy = SIZE // 2, SIZE // 2 - int(40 * SCALE)
    coords = draw_kinect_base(draw, cx, cy, SCALE)
    # LED dot: to the right of the base, scaled
    base_x1 = coords[6]
    base_cy = (coords[5] + coords[7]) // 2
    led_x = base_x1 + int(55 * SCALE)
    led_y = base_cy + int(10 * SCALE)
    draw_led(draw, led_x, led_y, int(38 * SCALE), color)
    path = os.path.join(OUT, filename)
    img.save(path, "PNG")
    print(f"  {path}")


def gen_tray_sizes(color, base_filename, prefix):
    """Generate tray-specific icons at fixed sizes KDE/GNOME expects."""
    for sz in [16, 22, 24, 32, 48, 64]:
        img = Image.new("RGBA", (sz, sz), BG)
        draw = ImageDraw.Draw(img)
        s = sz / 1024.0
        cx, cy = sz // 2, sz // 2 - int(40 * s)
        coords = draw_kinect_base(draw, cx, cy, s)
        base_x1 = coords[6]
        base_cy = (coords[5] + coords[7]) // 2
        led_x = base_x1 + int(55 * s)
        led_y = base_cy + int(10 * s)
        draw_led(draw, led_x, led_y, max(int(38 * s), 2), color)
        path = os.path.join(OUT, f"{prefix}-{sz}.png")
        img.save(path, "PNG")
    print(f"  {prefix}-{{16..64}}.png")


if __name__ == "__main__":
    print("Generating k4w-ctl icons...")
    gen_no_dot()
    gen_tilt()
    gen_led(LED_GREEN, "kinection-enabled.png", "connected")
    gen_led(LED_RED, "kinection-disabled.png", "disconnected")
    gen_led(LED_YELLOW, "kinection-error.png", "connecting")
    print("Generating tray-specific sizes...")
    gen_tray_sizes(LED_GREEN, "kinection-enabled.png", "kinection-tray-green")
    gen_tray_sizes(LED_RED, "kinection-disabled.png", "kinection-tray-red")
    gen_tray_sizes(LED_YELLOW, "kinection-error.png", "kinection-tray-yellow")
    print("Done!")
