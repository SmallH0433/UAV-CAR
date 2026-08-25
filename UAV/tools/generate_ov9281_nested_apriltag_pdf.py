#!/usr/bin/env python3
"""Generate the exact-size OV9281 nested tag36h11 landing target PDF."""

from __future__ import annotations

import argparse
from pathlib import Path

from reportlab.lib.colors import Color, black, white
from reportlab.lib.pagesizes import A4
from reportlab.lib.units import mm
from reportlab.pdfgen import canvas


# 1 is black.  Matrices are OpenCV DICT_APRILTAG_36h11 markers with one
# encoded black border cell and no printable quiet-zone cells.
OUTER_ID0 = (
    "11111111",
    "11101111",
    "11001011",
    "11110101",
    "11110011",
    "10100011",
    "10101001",
    "11111111",
)
INNER_ID1 = (
    "11111111",
    "10110111",
    "10100101",
    "11110011",
    "11100001",
    "10001011",
    "11001001",
    "11111111",
)


def draw_marker(pdf: canvas.Canvas, matrix: tuple[str, ...], side_mm: float) -> None:
    side = side_mm * mm
    cell = side / len(matrix)
    pdf.setFillColor(white)
    pdf.rect(-side / 2, -side / 2, side, side, stroke=0, fill=1)
    pdf.setFillColor(black)
    for row_index, row in enumerate(matrix):
        for column_index, value in enumerate(row):
            if value != "1":
                continue
            x = -side / 2 + column_index * cell
            y = side / 2 - (row_index + 1) * cell
            pdf.rect(x, y, cell, cell, stroke=0, fill=1)


def crop_mark(pdf: canvas.Canvas, x: float, y: float, dx: int, dy: int) -> None:
    length = 4 * mm
    gap = 1.5 * mm
    pdf.line(x + dx * gap, y, x + dx * (gap + length), y)
    pdf.line(x, y + dy * gap, x, y + dy * (gap + length))


def create_pdf(output_path: Path) -> None:
    output_path.parent.mkdir(parents=True, exist_ok=True)
    pdf = canvas.Canvas(str(output_path), pagesize=A4, pageCompression=1)
    width, height = A4
    pdf.setTitle("OV9281 Nested AprilTag tag36h11 ID0 100mm ID1 20mm")
    pdf.setAuthor("Air Ground Landing Project")
    pdf.setSubject("Exact-size concentric multi-scale AprilTag landing target")

    center_x = width / 2
    center_y = 166 * mm
    board_side = 135 * mm
    board_left = center_x - board_side / 2
    board_bottom = center_y - board_side / 2

    pdf.setFont("Helvetica-Bold", 16)
    pdf.drawCentredString(center_x, height - 18 * mm, "OV9281 NESTED APRILTAG LANDING TARGET")
    pdf.setFont("Helvetica", 9)
    pdf.drawCentredString(
        center_x,
        height - 24 * mm,
        "tag36h11 - outer ID 0: 100.0 mm - inner ID 1: 20.0 mm, rotated 45 deg CCW",
    )
    pdf.setFillColor(Color(0.75, 0.05, 0.05))
    pdf.setFont("Helvetica-Bold", 10)
    pdf.drawCentredString(center_x, height - 30 * mm, "PRINT AT 100% / ACTUAL SIZE - DISABLE FIT TO PAGE")

    pdf.setStrokeColor(Color(0.55, 0.55, 0.55))
    pdf.setLineWidth(0.25 * mm)
    pdf.setDash(2 * mm, 1.5 * mm)
    pdf.rect(board_left, board_bottom, board_side, board_side, stroke=1, fill=0)
    pdf.setDash()
    crop_mark(pdf, board_left, board_bottom, -1, -1)
    crop_mark(pdf, board_left + board_side, board_bottom, 1, -1)
    crop_mark(pdf, board_left, board_bottom + board_side, -1, 1)
    crop_mark(pdf, board_left + board_side, board_bottom + board_side, 1, 1)

    # The outer quiet zone is one 12.5 mm cell.  The inner quiet zone is one
    # 2.5 mm cell and is rotated together with the 20 mm marker.
    pdf.saveState()
    pdf.translate(center_x, center_y)
    pdf.setFillColor(white)
    pdf.rect(-62.5 * mm, -62.5 * mm, 125 * mm, 125 * mm, stroke=0, fill=1)
    draw_marker(pdf, OUTER_ID0, 100.0)
    pdf.rotate(45.0)
    pdf.setFillColor(white)
    pdf.rect(-12.5 * mm, -12.5 * mm, 25 * mm, 25 * mm, stroke=0, fill=1)
    draw_marker(pdf, INNER_ID1, 20.0)
    pdf.restoreState()

    info_y = 73 * mm
    pdf.setFillColor(black)
    pdf.setFont("Helvetica-Bold", 10)
    pdf.drawString(25 * mm, info_y, "MEASURE AFTER PRINTING")
    pdf.setFont("Helvetica", 8.5)
    pdf.drawString(25 * mm, info_y - 5 * mm, "Measure the BLACK square edge, not the white quiet zone or cut line.")
    pdf.drawString(25 * mm, info_y - 10 * mm, "Required: outer = 100.0 mm; inner diamond edge = 20.0 mm. Use a flat matte board.")
    pdf.drawString(25 * mm, info_y - 15 * mm, "Tag centers are concentric; do not crop, cover, laminate with glare, or redraw the pattern.")

    bar_x = 55 * mm
    bar_y = 35 * mm
    pdf.setLineWidth(0.5 * mm)
    pdf.line(bar_x, bar_y, bar_x + 100 * mm, bar_y)
    for index in range(11):
        tick = 4 * mm if index in (0, 10) else 2 * mm
        x = bar_x + index * 10 * mm
        pdf.line(x, bar_y - tick / 2, x, bar_y + tick / 2)
    pdf.setFont("Helvetica-Bold", 8)
    pdf.drawCentredString(bar_x + 50 * mm, bar_y - 6 * mm, "100 mm verification bar")
    pdf.setFont("Helvetica", 7.5)
    pdf.drawCentredString(center_x, 14 * mm, "Generated deterministically from tag36h11 ID 0 and ID 1 bit matrices")

    pdf.showPage()
    pdf.save()


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--output",
        type=Path,
        default=Path(__file__).resolve().parents[1]
        / "output"
        / "pdf"
        / "ov9281_nested_tag36h11_id0_100mm_id1_20mm.pdf",
    )
    args = parser.parse_args()
    create_pdf(args.output.resolve())
    print(args.output.resolve())


if __name__ == "__main__":
    main()
