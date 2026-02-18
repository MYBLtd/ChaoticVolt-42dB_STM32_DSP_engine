#!/usr/bin/env python3
"""
Generate compact PCB silkscreen-style pinout labels.
Output: H1_pinout.png at 300 DPI.
"""

from PIL import Image, ImageDraw, ImageFont

DPI = 300
FONT_PATH = "/usr/share/fonts/truetype/dejavu/DejaVuSansMono-Bold.ttf"

HEADERS = {
    "H1": [
        (1, "UART TX"),
        (2, "UART RX"),
        (3, "RST_N"),
        (4, "PWR/MFB"),
        (5, "VCC"),
        (6, "GND"),
    ],
    "H2": [
        (1, "GND"),
        (2, "MCLK"),
        (3, "DT1/DATA"),
        (4, "SCLK/BCLK"),
        (5, "RFS/LRCK"),
        (6, "DR1/DIN"),
    ],
}

FONT_SIZE = 18
MARGIN = 10
ROW_HEIGHT = 24
NUM_WIDTH = 24
GAP = 6

font = ImageFont.truetype(FONT_PATH, FONT_SIZE)
title_font = ImageFont.truetype(FONT_PATH, 20)

dummy = ImageDraw.Draw(Image.new("L", (1, 1)))

for name, pins in HEADERS.items():
    max_w = max(dummy.textbbox((0, 0), lbl, font=font)[2] for _, lbl in pins)
    title_w = dummy.textbbox((0, 0), name, font=title_font)[2]

    img_w = MARGIN + NUM_WIDTH + GAP + max_w + MARGIN
    img_h = MARGIN + 22 + 4 + ROW_HEIGHT * len(pins) + MARGIN - 4

    img = Image.new("RGB", (img_w, img_h), "white")
    d = ImageDraw.Draw(img)

    d.rectangle([0, 0, img_w - 1, img_h - 1], outline="black", width=2)
    d.text(((img_w - title_w) // 2, MARGIN - 2), name, fill="black", font=title_font)
    sep_y = MARGIN + 20
    d.line([(MARGIN, sep_y), (img_w - MARGIN, sep_y)], fill="black", width=1)

    for i, (num, lbl) in enumerate(pins):
        y = sep_y + 4 + i * ROW_HEIGHT
        d.text((MARGIN, y), str(num), fill="black", font=font)
        d.text((MARGIN + NUM_WIDTH + GAP, y), lbl, fill="black", font=font)

    out = f"/home/icz8922/Projects/42dB_H753_DSP_engine/{name}_pinout.png"
    img.save(out, dpi=(DPI, DPI))
    print(f"Saved: {out} ({img_w}x{img_h}px, {img_w/DPI:.2f}x{img_h/DPI:.2f} inch)")
