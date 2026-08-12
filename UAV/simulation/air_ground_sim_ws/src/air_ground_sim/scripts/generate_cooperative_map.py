#!/usr/bin/env python3
"""Generate the Nav2 occupancy map paired with the cooperative mission world."""

from pathlib import Path


RESOLUTION = 0.05
MIN_X = -15.0
MIN_Y = -11.0
WIDTH = 600
HEIGHT = 440

# center_x, center_y, size_x, size_y. Keep synchronized with the world file.
MAPPED_BOXES = (
    (-14.75, 0.0, 0.5, 22.0),
    (14.75, 0.0, 0.5, 22.0),
    (0.0, 10.75, 30.0, 0.5),
    (0.0, -10.75, 30.0, 0.5),
    (-5.5, -3.8, 2.9, 2.6),
    (-2.0, 2.0, 2.0, 4.8),
    (3.1, 0.2, 2.3, 3.2),
    (7.6, 3.7, 2.8, 2.2),
    (8.0, -1.8, 2.5, 3.5),
    (-6.5, 1.65, 0.55, 0.55),
    (-6.5, -1.65, 0.55, 0.55),
)


def world_to_column(x: float) -> int:
    return int((x - MIN_X) / RESOLUTION)


def world_to_row(y: float) -> int:
    return HEIGHT - 1 - int((y - MIN_Y) / RESOLUTION)


def draw_box(
    pixels: bytearray,
    center_x: float,
    center_y: float,
    size_x: float,
    size_y: float,
) -> None:
    min_column = max(0, world_to_column(center_x - size_x / 2.0))
    max_column = min(WIDTH - 1, world_to_column(center_x + size_x / 2.0))
    min_row = max(0, world_to_row(center_y + size_y / 2.0))
    max_row = min(HEIGHT - 1, world_to_row(center_y - size_y / 2.0))
    for row in range(min_row, max_row + 1):
        offset = row * WIDTH
        pixels[offset + min_column : offset + max_column + 1] = bytes(
            max_column - min_column + 1
        )


def main() -> None:
    output = Path(__file__).resolve().parent.parent / "maps" / "cooperative_map.pgm"
    pixels = bytearray([254]) * (WIDTH * HEIGHT)
    for box in MAPPED_BOXES:
        draw_box(pixels, *box)
    with output.open("wb") as stream:
        stream.write(f"P5\n{WIDTH} {HEIGHT}\n255\n".encode("ascii"))
        stream.write(pixels)
    print(f"generated {output} ({WIDTH}x{HEIGHT} at {RESOLUTION} m/cell)")


if __name__ == "__main__":
    main()

