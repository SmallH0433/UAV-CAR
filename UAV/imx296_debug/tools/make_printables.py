"""Create full-scale AprilTag and camera-calibration printables.

The AprilTag code/bit layout comes from the repository's tag36h11 family
definition. The output is vector PDF, so the black tag edge remains exact when
printed at 100% / Actual size.
"""

from pathlib import Path

from reportlab.lib import colors
from reportlab.lib.pagesizes import A3, A4, landscape
from reportlab.pdfgen import canvas
from reportlab.lib.units import mm


ROOT = Path(__file__).resolve().parents[2]
OUT = ROOT / "output" / "pdf"


def april_tag_grid():
    """Return the 10x10 tag36h11 ID 0 grid, row 0 at the top.

    AprilTag 36h11 uses a 1-cell white quiet border, a 1-cell black border,
    a 6x6 code, and a 1-cell black border. The configured tag edge is the
    black 8x8 square, not the outside quiet border.
    """
    code = 0x0000000D7E00984B  # tag36h11 ID 0
    bit_xy = [
        (1, 1), (2, 1), (3, 1), (4, 1), (5, 1),
        (2, 2), (3, 2), (4, 2), (3, 3),
        (6, 1), (6, 2), (6, 3), (6, 4), (6, 5),
        (5, 2), (5, 3), (5, 4), (4, 3),
        (6, 6), (5, 6), (4, 6), (3, 6), (2, 6),
        (5, 5), (4, 5), (3, 5), (4, 4),
        (1, 6), (1, 5), (1, 4), (1, 3), (1, 2),
        (2, 5), (2, 4), (2, 3), (3, 4),
    ]
    grid = [[1 for _ in range(10)] for _ in range(10)]
    for y in range(1, 9):
        for x in range(1, 9):
            grid[y][x] = 0
    for bit_index, (x, y) in enumerate(bit_xy):
        bit = (code >> (35 - bit_index)) & 1
        grid[y + 1][x + 1] = bit
    return grid


def draw_text(c, text, x, y, size=10, bold=False):
    c.setFont("Helvetica-Bold" if bold else "Helvetica", size)
    c.setFillColor(colors.black)
    c.drawString(x, y, text)


def make_apriltag(path: Path):
    page_w, page_h = A3
    c = canvas.Canvas(str(path), pagesize=A3)
    c.setTitle("AprilTag tag36h11 ID 0 - 200 mm")

    total = 250 * mm
    cell = 25 * mm
    x0 = (page_w - total) / 2
    y0 = (page_h - total) / 2 + 18 * mm
    grid = april_tag_grid()

    # Draw vector cells; y=0 in the tag definition is the top row.
    for row in range(10):
        for col in range(10):
            if grid[row][col] == 0:
                c.setFillColor(colors.black)
                c.rect(x0 + col * cell, y0 + (9 - row) * cell, cell, cell, stroke=0, fill=1)

    # Light outline around the black 200 mm tag edge for physical checking.
    c.setStrokeColor(colors.HexColor("#707070"))
    c.setLineWidth(0.5)
    c.rect(x0 + cell, y0 + cell, 8 * cell, 8 * cell, stroke=1, fill=0)

    draw_text(c, "AprilTag tag36h11 / ID 0", 20 * mm, page_h - 22 * mm, 16, True)
    draw_text(c, "Black tag edge = 200 mm; outer white quiet border = 25 mm on each side", 20 * mm, page_h - 31 * mm, 10)
    draw_text(c, "Print at 100% / Actual size. Do not use Fit to page or Scale to fit.", 20 * mm, page_h - 39 * mm, 10, True)

    # A separate 200 mm reference line makes a wrong printer scale obvious.
    ref_x = 20 * mm
    ref_y = 22 * mm
    c.setStrokeColor(colors.black)
    c.setLineWidth(1)
    c.line(ref_x, ref_y, ref_x + 200 * mm, ref_y)
    c.line(ref_x, ref_y - 2 * mm, ref_x, ref_y + 2 * mm)
    c.line(ref_x + 200 * mm, ref_y - 2 * mm, ref_x + 200 * mm, ref_y + 2 * mm)
    draw_text(c, "Reference: 200 mm", ref_x + 72 * mm, ref_y + 4 * mm, 9)
    draw_text(c, "Use tag_size_m: 0.200 in the observer configuration.", 20 * mm, 11 * mm, 9)

    c.showPage()
    c.save()


def make_chessboard(path: Path):
    page_w, page_h = landscape(A4)
    c = canvas.Canvas(str(path), pagesize=(page_w, page_h))
    c.setTitle("IMX296 calibration chessboard - 9x6 inner corners, 25 mm")

    cell = 25 * mm
    cols, rows = 10, 7
    board_w, board_h = cols * cell, rows * cell
    x0 = (page_w - board_w) / 2
    # Leave enough white space above the board for the subtitle and below it
    # for the print instruction; the 250 x 175 mm pattern still fits A4.
    y0 = 14 * mm

    for row in range(rows):
        for col in range(cols):
            if (row + col) % 2 == 0:
                c.setFillColor(colors.black)
                c.rect(x0 + col * cell, y0 + (rows - 1 - row) * cell, cell, cell, stroke=0, fill=1)

    c.setStrokeColor(colors.black)
    c.setLineWidth(0.6)
    c.rect(x0, y0, board_w, board_h, stroke=1, fill=0)
    draw_text(c, "IMX296 camera calibration chessboard", 15 * mm, page_h - 11 * mm, 13, True)
    draw_text(c, "10 x 7 squares / 9 x 6 inner corners / square size = 25 mm", 15 * mm, page_h - 18 * mm, 9)
    draw_text(c, "Print at 100% / Actual size. Mount flat; do not laminate with a glossy reflection before calibration.", 15 * mm, 6 * mm, 8)

    c.showPage()
    c.save()


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    make_apriltag(OUT / "apriltag_tag36h11_id0_200mm.pdf")
    make_chessboard(OUT / "imx296_calibration_chessboard_9x6_25mm.pdf")
    print(f"created: {OUT / 'apriltag_tag36h11_id0_200mm.pdf'}")
    print(f"created: {OUT / 'imx296_calibration_chessboard_9x6_25mm.pdf'}")


if __name__ == "__main__":
    main()
