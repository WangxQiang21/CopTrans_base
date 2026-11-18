#!/usr/bin/env python3
"""Add an evidence caption strip below a screenshot (original kept)."""
import argparse

from PIL import Image, ImageDraw, ImageFont


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--input", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--line", action="append", required=True)
    args = parser.parse_args()
    image = Image.open(args.input).convert("RGB")
    font = ImageFont.truetype(
        "/usr/share/fonts/truetype/dejavu/DejaVuSansMono.ttf", 15)
    line_height = 20
    strip = 8 + line_height * len(args.line)
    canvas = Image.new("RGB", (image.width, image.height + strip),
                       (12, 12, 12))
    canvas.paste(image, (0, 0))
    draw = ImageDraw.Draw(canvas)
    for index, line in enumerate(args.line):
        draw.text((10, image.height + 4 + index * line_height), line,
                  fill=(235, 235, 235), font=font)
    canvas.save(args.output)


if __name__ == "__main__":
    main()