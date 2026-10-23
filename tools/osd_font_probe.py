#!/usr/bin/env python3
"""Inspect the MAX7456 (.mcm) OSD fonts the configurator ships.

The UE HUD (Source/FPVSkyline/.../BfOsdFont.cpp) draws its glyphs from these
files, so this is the ground truth when a symbol "looks wrong" in game: the
configurator ships two sets (resources/osd/1 and /2) of the same ten families,
and its own font menu always uses set 2 (OsdTab.vue: fontVer = 2).

Usage (paths default to the configurator copy staged in the UE project):
    python tools/osd_font_probe.py glyph   0x73            # one glyph, ASCII
    python tools/osd_font_probe.py glyph   0x41 -s 1 -f bold
    python tools/osd_font_probe.py element 0x72,0x73,0x74  # multi-cell element
    python tools/osd_font_probe.py compare                 # set 1 vs set 2
    python tools/osd_font_probe.py png                     # atlas PNG vs .mcm

Legend for the ASCII output: '#' = white ink (value 2), 'x' = black (value 0,
the OSD's outline), '.' = transparent (value 1).
"""

import argparse
import os
import sys

DEFAULT_BASE = r"E:/Epic Games/FPVSkyline/Binaries/Win64/configurator/resources/osd"
FAMILIES = ["default", "bold", "large", "extra_large", "betaflight",
            "digital", "clarity", "vision", "impact", "impact_mini"]

WIDTH, HEIGHT = 12, 18        # glyph cell, in font pixels
GLYPH_LINES = 64              # lines per glyph in the file (54 used + 10 pad)
BITS_PER_LINE = 8             # one line = 8 ASCII bits = 4 pixels of 2 bits


def parse_mcm(path):
    """Return 256 glyphs as [row][col] arrays of 2-bit pixel values."""
    with open(path, "r", encoding="utf-8", errors="replace") as fh:
        lines = fh.read().splitlines()
    if not lines or lines[0].strip() != "MAX7456":
        raise ValueError(f"{path}: not a MAX7456 font (missing header)")

    glyphs = []
    index = 1
    for _ in range(256):
        grid = [[1] * WIDTH for _ in range(HEIGHT)]
        for row in range(HEIGHT):
            for byte_in_row in range(3):
                text = lines[index + row * 3 + byte_in_row].strip()
                byte = 0
                for ch in text[:BITS_PER_LINE]:
                    byte = (byte << 1) | (1 if ch == "1" else 0)
                for pair in range(4):
                    grid[row][byte_in_row * 4 + pair] = (byte >> (6 - pair * 2)) & 0x3
        index += GLYPH_LINES
        glyphs.append(grid)
    return glyphs


def art(grid):
    return ["".join("." if v == 1 else ("#" if v == 2 else "x") for v in row)
            for row in grid]


def print_glyph(glyphs, char, label):
    print(f"--- {label} 0x{char:02X} ---")
    for line in art(glyphs[char]):
        print(line)


def print_element(glyphs, chars, label):
    """Lay adjacent cells out side by side, the way the OSD shows them."""
    print(f"--- {label} {', '.join('0x%02X' % c for c in chars)} ---")
    combined = {}
    for col, char in enumerate(chars):
        for row, line in enumerate(art(glyphs[char])):
            if line.strip("."):
                combined.setdefault(row, ["."] * (WIDTH * len(chars)))
                for i, ch in enumerate(line):
                    if ch != ".":
                        combined[row][col * WIDTH + i] = ch
    for row in sorted(combined):
        print("".join(combined[row]))


def load(base, set_index, family):
    path = os.path.join(base, str(set_index), family + ".mcm")
    if not os.path.isfile(path):
        sys.exit(f"no such font: {path}")
    return parse_mcm(path)


def cmd_compare(args):
    """Report which glyphs differ between the two sets (set 2 is the one the
    configurator uses)."""
    for family in FAMILIES:
        a = load(args.base, 1, family)
        b = load(args.base, 2, family)
        differing = [c for c in range(256) if a[c] != b[c]]
        print(f"{family:12s} {len(differing):3d} glyphs differ: "
              + " ".join(f"0x{c:02X}" for c in differing[:12])
              + (" ..." if len(differing) > 12 else ""))


def cmd_png(args):
    """Cross-check the .mcm parser against the atlas PNG shipped beside it."""
    from PIL import Image  # optional dependency, only needed here

    for set_index in (1, 2):
        path = os.path.join(args.base, str(set_index), args.family + ".png")
        img = Image.open(path).convert("L")
        glyphs = load(args.base, set_index, args.family)
        # 16x16 cells of 12x18 with 1px separators: cell (r,c) starts at
        # (c * 13, r * 19).
        bad = 0
        for char in range(256):
            cx, cy = (char % 16) * 13, (char // 16) * 19
            for row in range(HEIGHT):
                for col in range(WIDTH):
                    png_ink = img.getpixel((cx + col, cy + row)) < 128
                    mcm_ink = glyphs[char][row][col] != 1
                    if png_ink != mcm_ink:
                        bad += 1
        total = 256 * WIDTH * HEIGHT
        print(f"set {set_index}: {bad}/{total} texels differ from {os.path.basename(path)}")


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--base", default=DEFAULT_BASE,
                        help="configurator resources/osd directory")
    sub = parser.add_subparsers(dest="command", required=True)

    p_glyph = sub.add_parser("glyph", help="print one glyph as ASCII")
    p_glyph.add_argument("char", help="character code, e.g. 0x73")
    p_glyph.add_argument("-s", "--set", type=int, default=2, dest="set_index")
    p_glyph.add_argument("-f", "--family", default="default")

    p_elem = sub.add_parser("element", help="print adjacent glyphs combined")
    p_elem.add_argument("chars", help="comma-separated codes, e.g. 0x72,0x73,0x74")
    p_elem.add_argument("-s", "--set", type=int, default=2, dest="set_index")
    p_elem.add_argument("-f", "--family", default="default")

    sub.add_parser("compare", help="list glyphs that differ between set 1 and 2")

    p_png = sub.add_parser("png", help="verify the parser against the atlas PNG")
    p_png.add_argument("-f", "--family", default="betaflight")

    args = parser.parse_args()

    if args.command == "glyph":
        chars = [int(args.char, 0)]
        print_glyph(load(args.base, args.set_index, args.family), chars[0],
                    f"set {args.set_index}/{args.family}")
    elif args.command == "element":
        chars = [int(c, 0) for c in args.chars.split(",")]
        print_element(load(args.base, args.set_index, args.family), chars,
                      f"set {args.set_index}/{args.family}")
    elif args.command == "compare":
        cmd_compare(args)
    elif args.command == "png":
        cmd_png(args)


if __name__ == "__main__":
    main()
