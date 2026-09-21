from __future__ import annotations

from pathlib import Path
from typing import Iterable, Sequence

from docx import Document
from docx.enum.table import WD_ALIGN_VERTICAL, WD_TABLE_ALIGNMENT
from docx.enum.text import WD_ALIGN_PARAGRAPH
from docx.oxml import OxmlElement
from docx.oxml.ns import qn
from docx.shared import Inches, Pt, RGBColor
from PIL import Image, ImageDraw, ImageFont


ROOT = Path(__file__).resolve().parents[1]
REPORT_DIR = ROOT / "reports"
ASSET_DIR = REPORT_DIR / "assets"
REPORT_PATH = REPORT_DIR / "poweron-rc-car-wiring-report.ko.docx"
DIAGRAM_PATH = ASSET_DIR / "poweron_wiring_topology.png"

# Resolved compact_reference_guide tokens.
PAGE_WIDTH_IN = 8.5
PAGE_HEIGHT_IN = 11.0
MARGIN_IN = 1.0
CONTENT_WIDTH_DXA = 9360
TABLE_INDENT_DXA = 120
CELL_MARGINS_DXA = {"top": 80, "bottom": 80, "start": 120, "end": 120}
ASCII_FONT = "Calibri"
KOREAN_FONT = "Malgun Gothic"  # Named override: reliable Hangul rendering.
BODY_SIZE_PT = 11
BODY_AFTER_PT = 6
BODY_LINE = 1.25
NAVY = "0B2545"
BLUE = "2E74B5"
DARK_BLUE = "1F4D78"
MUTED = "667085"
TABLE_HEADER = "E8EEF5"
LIGHT_GRAY = "F4F6F9"
CAUTION_FILL = "FFF4CC"
CAUTION_BORDER = "D6A800"
RISK_RED = "9B1C1C"
WHITE = "FFFFFF"
GRID = "B8C2CC"


def hex_rgb(value: str) -> RGBColor:
    return RGBColor.from_string(value)


def set_run_font(
    run,
    *,
    name: str = ASCII_FONT,
    east_asia: str = KOREAN_FONT,
    size: float | None = None,
    color: str | None = None,
    bold: bool | None = None,
    italic: bool | None = None,
):
    run.font.name = name
    rpr = run._element.get_or_add_rPr()
    rfonts = rpr.rFonts
    if rfonts is None:
        rfonts = OxmlElement("w:rFonts")
        rpr.insert(0, rfonts)
    rfonts.set(qn("w:ascii"), name)
    rfonts.set(qn("w:hAnsi"), name)
    rfonts.set(qn("w:eastAsia"), east_asia)
    if size is not None:
        run.font.size = Pt(size)
    if color is not None:
        run.font.color.rgb = hex_rgb(color)
    if bold is not None:
        run.bold = bold
    if italic is not None:
        run.italic = italic
    return run


def set_style_font(style, size: float, color: str, bold: bool = False):
    style.font.name = ASCII_FONT
    style.font.size = Pt(size)
    style.font.color.rgb = hex_rgb(color)
    style.font.bold = bold
    rpr = style.element.get_or_add_rPr()
    rfonts = rpr.rFonts
    if rfonts is None:
        rfonts = OxmlElement("w:rFonts")
        rpr.insert(0, rfonts)
    rfonts.set(qn("w:ascii"), ASCII_FONT)
    rfonts.set(qn("w:hAnsi"), ASCII_FONT)
    rfonts.set(qn("w:eastAsia"), KOREAN_FONT)


def apply_styles(doc: Document):
    normal = doc.styles["Normal"]
    set_style_font(normal, BODY_SIZE_PT, "000000")
    normal.paragraph_format.space_before = Pt(0)
    normal.paragraph_format.space_after = Pt(BODY_AFTER_PT)
    normal.paragraph_format.line_spacing = BODY_LINE

    heading_tokens = {
        "Heading 1": (16, BLUE, 18, 10),
        "Heading 2": (13, BLUE, 14, 7),
        "Heading 3": (12, DARK_BLUE, 10, 5),
    }
    for name, (size, color, before, after) in heading_tokens.items():
        style = doc.styles[name]
        set_style_font(style, size, color, True)
        style.paragraph_format.space_before = Pt(before)
        style.paragraph_format.space_after = Pt(after)
        style.paragraph_format.keep_with_next = True
        style.paragraph_format.line_spacing = 1.0

    title = doc.styles["Title"]
    set_style_font(title, 25, NAVY, True)
    title.paragraph_format.space_before = Pt(0)
    title.paragraph_format.space_after = Pt(8)
    title.paragraph_format.line_spacing = 1.0

    subtitle = doc.styles["Subtitle"]
    set_style_font(subtitle, 12.5, MUTED, False)
    subtitle.paragraph_format.space_before = Pt(0)
    subtitle.paragraph_format.space_after = Pt(16)
    subtitle.paragraph_format.line_spacing = 1.15

    caption = doc.styles["Caption"]
    set_style_font(caption, 9, MUTED, False)
    caption.paragraph_format.space_before = Pt(4)
    caption.paragraph_format.space_after = Pt(6)
    caption.paragraph_format.keep_with_next = True

    if "Code Block" not in [s.name for s in doc.styles]:
        code = doc.styles.add_style("Code Block", 1)
    else:
        code = doc.styles["Code Block"]
    set_style_font(code, 9.5, "1D2939", False)
    code.font.name = "Consolas"
    rpr = code.element.get_or_add_rPr()
    rpr.rFonts.set(qn("w:ascii"), "Consolas")
    rpr.rFonts.set(qn("w:hAnsi"), "Consolas")
    code.paragraph_format.space_before = Pt(4)
    code.paragraph_format.space_after = Pt(4)
    code.paragraph_format.line_spacing = 1.05


def set_cell_margins(cell, top=80, start=120, bottom=80, end=120):
    tc = cell._tc
    tc_pr = tc.get_or_add_tcPr()
    tc_mar = tc_pr.first_child_found_in("w:tcMar")
    if tc_mar is None:
        tc_mar = OxmlElement("w:tcMar")
        tc_pr.append(tc_mar)
    for margin, value in (("top", top), ("start", start), ("bottom", bottom), ("end", end)):
        node = tc_mar.find(qn(f"w:{margin}"))
        if node is None:
            node = OxmlElement(f"w:{margin}")
            tc_mar.append(node)
        node.set(qn("w:w"), str(value))
        node.set(qn("w:type"), "dxa")


def shade_cell(cell, fill: str):
    tc_pr = cell._tc.get_or_add_tcPr()
    shd = tc_pr.find(qn("w:shd"))
    if shd is None:
        shd = OxmlElement("w:shd")
        tc_pr.append(shd)
    shd.set(qn("w:fill"), fill)
    shd.set(qn("w:val"), "clear")


def set_cell_border(cell, **edges):
    tc_pr = cell._tc.get_or_add_tcPr()
    tc_borders = tc_pr.first_child_found_in("w:tcBorders")
    if tc_borders is None:
        tc_borders = OxmlElement("w:tcBorders")
        tc_pr.append(tc_borders)
    for edge_name, attrs in edges.items():
        edge = tc_borders.find(qn(f"w:{edge_name}"))
        if edge is None:
            edge = OxmlElement(f"w:{edge_name}")
            tc_borders.append(edge)
        for key, value in attrs.items():
            edge.set(qn(f"w:{key}"), str(value))


def set_table_borders(table, color=GRID, size=4):
    tbl_pr = table._tbl.tblPr
    borders = tbl_pr.find(qn("w:tblBorders"))
    if borders is None:
        borders = OxmlElement("w:tblBorders")
        tbl_pr.append(borders)
    for edge_name in ("top", "left", "bottom", "right", "insideH", "insideV"):
        edge = borders.find(qn(f"w:{edge_name}"))
        if edge is None:
            edge = OxmlElement(f"w:{edge_name}")
            borders.append(edge)
        edge.set(qn("w:val"), "single")
        edge.set(qn("w:sz"), str(size))
        edge.set(qn("w:space"), "0")
        edge.set(qn("w:color"), color)


def set_repeat_table_header(row):
    tr_pr = row._tr.get_or_add_trPr()
    tbl_header = tr_pr.find(qn("w:tblHeader"))
    if tbl_header is None:
        tbl_header = OxmlElement("w:tblHeader")
        tr_pr.append(tbl_header)
    tbl_header.set(qn("w:val"), "true")


def set_row_cant_split(row):
    tr_pr = row._tr.get_or_add_trPr()
    cant_split = tr_pr.find(qn("w:cantSplit"))
    if cant_split is None:
        cant_split = OxmlElement("w:cantSplit")
        tr_pr.append(cant_split)
    cant_split.set(qn("w:val"), "true")


def set_table_geometry(table, widths_dxa: Sequence[int], indent_dxa: int = TABLE_INDENT_DXA):
    if sum(widths_dxa) != CONTENT_WIDTH_DXA:
        raise ValueError(f"Column widths must total {CONTENT_WIDTH_DXA}: {widths_dxa}")
    table.alignment = WD_TABLE_ALIGNMENT.LEFT
    table.autofit = False
    tbl_pr = table._tbl.tblPr

    tbl_w = tbl_pr.find(qn("w:tblW"))
    if tbl_w is None:
        tbl_w = OxmlElement("w:tblW")
        tbl_pr.append(tbl_w)
    tbl_w.set(qn("w:w"), str(CONTENT_WIDTH_DXA))
    tbl_w.set(qn("w:type"), "dxa")

    tbl_ind = tbl_pr.find(qn("w:tblInd"))
    if tbl_ind is None:
        tbl_ind = OxmlElement("w:tblInd")
        tbl_pr.append(tbl_ind)
    tbl_ind.set(qn("w:w"), str(indent_dxa))
    tbl_ind.set(qn("w:type"), "dxa")

    layout = tbl_pr.find(qn("w:tblLayout"))
    if layout is None:
        layout = OxmlElement("w:tblLayout")
        tbl_pr.append(layout)
    layout.set(qn("w:type"), "fixed")

    grid = table._tbl.tblGrid
    for child in list(grid):
        grid.remove(child)
    for width in widths_dxa:
        col = OxmlElement("w:gridCol")
        col.set(qn("w:w"), str(width))
        grid.append(col)

    for row in table.rows:
        set_row_cant_split(row)
        for idx, cell in enumerate(row.cells):
            tc_pr = cell._tc.get_or_add_tcPr()
            tc_w = tc_pr.find(qn("w:tcW"))
            if tc_w is None:
                tc_w = OxmlElement("w:tcW")
                tc_pr.append(tc_w)
            tc_w.set(qn("w:w"), str(widths_dxa[idx]))
            tc_w.set(qn("w:type"), "dxa")
            cell.width = Inches(widths_dxa[idx] / 1440)
            set_cell_margins(cell, **CELL_MARGINS_DXA)


def format_table_text(cell, size=9, bold=False, color="000000", align=WD_ALIGN_PARAGRAPH.LEFT):
    cell.vertical_alignment = WD_ALIGN_VERTICAL.CENTER
    for paragraph in cell.paragraphs:
        paragraph.alignment = align
        paragraph.paragraph_format.space_before = Pt(0)
        paragraph.paragraph_format.space_after = Pt(0)
        paragraph.paragraph_format.line_spacing = 1.15
        for run in paragraph.runs:
            set_run_font(run, size=size, color=color, bold=bold)


def add_table(
    doc: Document,
    headers: Sequence[str],
    rows: Iterable[Sequence[str]],
    widths_dxa: Sequence[int],
    *,
    font_size=9,
    center_columns: set[int] | None = None,
):
    rows = list(rows)
    table = doc.add_table(rows=1, cols=len(headers))
    table.style = "Table Grid"
    hdr = table.rows[0]
    for idx, text in enumerate(headers):
        hdr.cells[idx].text = text
        shade_cell(hdr.cells[idx], TABLE_HEADER)
        format_table_text(hdr.cells[idx], size=font_size, bold=True, color=NAVY, align=WD_ALIGN_PARAGRAPH.CENTER)
    set_repeat_table_header(hdr)

    center_columns = center_columns or set()
    for row_data in rows:
        cells = table.add_row().cells
        for idx, text in enumerate(row_data):
            cells[idx].text = str(text)
            align = WD_ALIGN_PARAGRAPH.CENTER if idx in center_columns else WD_ALIGN_PARAGRAPH.LEFT
            format_table_text(cells[idx], size=font_size, align=align)
    set_table_geometry(table, widths_dxa)
    set_table_borders(table)
    doc.add_paragraph().paragraph_format.space_after = Pt(0)
    return table


def add_paragraph_border_bottom(paragraph, color=BLUE, size=10, space=6):
    p_pr = paragraph._p.get_or_add_pPr()
    p_bdr = p_pr.find(qn("w:pBdr"))
    if p_bdr is None:
        p_bdr = OxmlElement("w:pBdr")
        p_pr.append(p_bdr)
    bottom = p_bdr.find(qn("w:bottom"))
    if bottom is None:
        bottom = OxmlElement("w:bottom")
        p_bdr.append(bottom)
    bottom.set(qn("w:val"), "single")
    bottom.set(qn("w:sz"), str(size))
    bottom.set(qn("w:space"), str(space))
    bottom.set(qn("w:color"), color)


def set_keep_with_next(paragraph, value=True):
    paragraph.paragraph_format.keep_with_next = value


def add_heading(doc: Document, text: str, level: int):
    paragraph = doc.add_heading(text, level=level)
    set_keep_with_next(paragraph)
    return paragraph


def add_body(doc: Document, text: str, *, bold_prefix: str | None = None):
    paragraph = doc.add_paragraph()
    if bold_prefix and text.startswith(bold_prefix):
        set_run_font(paragraph.add_run(bold_prefix), size=BODY_SIZE_PT, bold=True, color=NAVY)
        set_run_font(paragraph.add_run(text[len(bold_prefix):]), size=BODY_SIZE_PT)
    else:
        set_run_font(paragraph.add_run(text), size=BODY_SIZE_PT)
    return paragraph


def create_numbering(doc: Document, *, bullet: bool) -> int:
    numbering = doc.part.numbering_part.element
    abstract_ids = [int(e.get(qn("w:abstractNumId"))) for e in numbering.findall(qn("w:abstractNum"))]
    num_ids = [int(e.get(qn("w:numId"))) for e in numbering.findall(qn("w:num"))]
    abstract_id = max(abstract_ids, default=0) + 1
    num_id = max(num_ids, default=0) + 1

    abstract = OxmlElement("w:abstractNum")
    abstract.set(qn("w:abstractNumId"), str(abstract_id))
    multi = OxmlElement("w:multiLevelType")
    multi.set(qn("w:val"), "singleLevel")
    abstract.append(multi)
    lvl = OxmlElement("w:lvl")
    lvl.set(qn("w:ilvl"), "0")
    start = OxmlElement("w:start")
    start.set(qn("w:val"), "1")
    lvl.append(start)
    num_fmt = OxmlElement("w:numFmt")
    num_fmt.set(qn("w:val"), "bullet" if bullet else "decimal")
    lvl.append(num_fmt)
    lvl_text = OxmlElement("w:lvlText")
    lvl_text.set(qn("w:val"), "•" if bullet else "%1.")
    lvl.append(lvl_text)
    lvl_jc = OxmlElement("w:lvlJc")
    lvl_jc.set(qn("w:val"), "left")
    lvl.append(lvl_jc)
    p_pr = OxmlElement("w:pPr")
    tabs = OxmlElement("w:tabs")
    tab = OxmlElement("w:tab")
    tab.set(qn("w:val"), "num")
    tab.set(qn("w:pos"), "540")
    tabs.append(tab)
    p_pr.append(tabs)
    ind = OxmlElement("w:ind")
    ind.set(qn("w:left"), "540")
    ind.set(qn("w:hanging"), "270")
    p_pr.append(ind)
    spacing = OxmlElement("w:spacing")
    spacing.set(qn("w:after"), "80")
    spacing.set(qn("w:line"), "300")
    spacing.set(qn("w:lineRule"), "auto")
    p_pr.append(spacing)
    lvl.append(p_pr)
    if bullet:
        r_pr = OxmlElement("w:rPr")
        fonts = OxmlElement("w:rFonts")
        fonts.set(qn("w:ascii"), ASCII_FONT)
        fonts.set(qn("w:hAnsi"), ASCII_FONT)
        r_pr.append(fonts)
        lvl.append(r_pr)
    abstract.append(lvl)
    # OOXML requires every abstractNum to appear before the concrete num
    # instances. Appending an abstract after existing nums makes Word repair
    # the list and can turn bullets into a continuing decimal sequence.
    first_num = numbering.find(qn("w:num"))
    if first_num is None:
        numbering.append(abstract)
    else:
        numbering.insert(numbering.index(first_num), abstract)

    num = OxmlElement("w:num")
    num.set(qn("w:numId"), str(num_id))
    abstract_ref = OxmlElement("w:abstractNumId")
    abstract_ref.set(qn("w:val"), str(abstract_id))
    num.append(abstract_ref)
    numbering.append(num)
    return num_id


def add_list_item(doc: Document, text: str, num_id: int, *, bold_prefix: str | None = None):
    paragraph = doc.add_paragraph()
    p_pr = paragraph._p.get_or_add_pPr()
    num_pr = OxmlElement("w:numPr")
    ilvl = OxmlElement("w:ilvl")
    ilvl.set(qn("w:val"), "0")
    num_id_node = OxmlElement("w:numId")
    num_id_node.set(qn("w:val"), str(num_id))
    num_pr.append(ilvl)
    num_pr.append(num_id_node)
    p_pr.append(num_pr)
    paragraph.paragraph_format.space_after = Pt(4)
    paragraph.paragraph_format.line_spacing = BODY_LINE
    if bold_prefix and text.startswith(bold_prefix):
        set_run_font(paragraph.add_run(bold_prefix), size=BODY_SIZE_PT, bold=True, color=NAVY)
        set_run_font(paragraph.add_run(text[len(bold_prefix):]), size=BODY_SIZE_PT)
    else:
        set_run_font(paragraph.add_run(text), size=BODY_SIZE_PT)
    return paragraph


def add_callout(doc: Document, title: str, body: str, *, caution=True):
    table = doc.add_table(rows=1, cols=1)
    cell = table.cell(0, 0)
    shade_cell(cell, CAUTION_FILL if caution else LIGHT_GRAY)
    border = CAUTION_BORDER if caution else "9AA6B2"
    set_cell_border(
        cell,
        top={"val": "single", "sz": "8", "color": border},
        left={"val": "single", "sz": "18", "color": border},
        bottom={"val": "single", "sz": "8", "color": border},
        right={"val": "single", "sz": "8", "color": border},
    )
    paragraph = cell.paragraphs[0]
    paragraph.paragraph_format.space_after = Pt(2)
    set_run_font(paragraph.add_run(title + "  "), size=10.5, bold=True, color=RISK_RED if caution else NAVY)
    set_run_font(paragraph.add_run(body), size=10.5, color="1D2939")
    set_table_geometry(table, [CONTENT_WIDTH_DXA])
    doc.add_paragraph().paragraph_format.space_after = Pt(0)
    return table


def add_code_block(doc: Document, lines: Sequence[str]):
    table = doc.add_table(rows=1, cols=1)
    cell = table.cell(0, 0)
    shade_cell(cell, "F2F4F7")
    set_cell_border(
        cell,
        top={"val": "single", "sz": "4", "color": "D0D5DD"},
        left={"val": "single", "sz": "4", "color": "D0D5DD"},
        bottom={"val": "single", "sz": "4", "color": "D0D5DD"},
        right={"val": "single", "sz": "4", "color": "D0D5DD"},
    )
    paragraph = cell.paragraphs[0]
    paragraph.style = doc.styles["Code Block"]
    for idx, line in enumerate(lines):
        if idx:
            paragraph.add_run().add_break()
        set_run_font(paragraph.add_run(line), name="Consolas", east_asia=KOREAN_FONT, size=9.5, color="1D2939")
    set_table_geometry(table, [CONTENT_WIDTH_DXA])
    doc.add_paragraph().paragraph_format.space_after = Pt(0)


def add_field(paragraph, instruction: str):
    run = paragraph.add_run()
    fld_char_begin = OxmlElement("w:fldChar")
    fld_char_begin.set(qn("w:fldCharType"), "begin")
    instr_text = OxmlElement("w:instrText")
    instr_text.set(qn("xml:space"), "preserve")
    instr_text.text = instruction
    fld_char_end = OxmlElement("w:fldChar")
    fld_char_end.set(qn("w:fldCharType"), "end")
    run._r.append(fld_char_begin)
    run._r.append(instr_text)
    run._r.append(fld_char_end)
    set_run_font(run, size=9, color=MUTED)


def add_hyperlink(paragraph, text: str, url: str):
    part = paragraph.part
    rel_id = part.relate_to(url, "http://schemas.openxmlformats.org/officeDocument/2006/relationships/hyperlink", is_external=True)
    hyperlink = OxmlElement("w:hyperlink")
    hyperlink.set(qn("r:id"), rel_id)
    new_run = OxmlElement("w:r")
    r_pr = OxmlElement("w:rPr")
    color = OxmlElement("w:color")
    color.set(qn("w:val"), BLUE)
    r_pr.append(color)
    underline = OxmlElement("w:u")
    underline.set(qn("w:val"), "single")
    r_pr.append(underline)
    r_fonts = OxmlElement("w:rFonts")
    r_fonts.set(qn("w:ascii"), ASCII_FONT)
    r_fonts.set(qn("w:hAnsi"), ASCII_FONT)
    r_fonts.set(qn("w:eastAsia"), KOREAN_FONT)
    r_pr.append(r_fonts)
    new_run.append(r_pr)
    text_node = OxmlElement("w:t")
    text_node.text = text
    new_run.append(text_node)
    hyperlink.append(new_run)
    paragraph._p.append(hyperlink)


def configure_page(doc: Document):
    doc.settings.odd_and_even_pages_header_footer = False
    for section in doc.sections:
        section.different_first_page_header_footer = False
        section.page_width = Inches(PAGE_WIDTH_IN)
        section.page_height = Inches(PAGE_HEIGHT_IN)
        section.top_margin = Inches(MARGIN_IN)
        section.bottom_margin = Inches(MARGIN_IN)
        section.left_margin = Inches(MARGIN_IN)
        section.right_margin = Inches(MARGIN_IN)
        section.header_distance = Inches(0.492)
        section.footer_distance = Inches(0.492)

        header_p = section.header.paragraphs[0]
        header_p.alignment = WD_ALIGN_PARAGRAPH.LEFT
        header_p.paragraph_format.space_after = Pt(0)
        header_p.paragraph_format.tab_stops.add_tab_stop(Inches(6.5))
        set_run_font(header_p.add_run("POWERON | WIRING REPORT"), size=8.5, color=MUTED, bold=True)
        set_run_font(header_p.add_run("\tNUCLEO-F103RB"), size=8.5, color=MUTED)

        footer_p = section.footer.paragraphs[0]
        footer_p.alignment = WD_ALIGN_PARAGRAPH.RIGHT
        footer_p.paragraph_format.space_before = Pt(0)
        set_run_font(footer_p.add_run("PowerOn 배선 보고서  |  "), size=8.5, color=MUTED)
        add_field(footer_p, "PAGE")


def load_font(size: int, bold=False):
    candidates = [
        Path("C:/Windows/Fonts/malgunbd.ttf" if bold else "C:/Windows/Fonts/malgun.ttf"),
        Path("C:/Windows/Fonts/arialbd.ttf" if bold else "C:/Windows/Fonts/arial.ttf"),
    ]
    for candidate in candidates:
        if candidate.exists():
            return ImageFont.truetype(str(candidate), size)
    return ImageFont.load_default()


def draw_centered(draw, box, lines, *, title=False, fill="#102A43"):
    font = load_font(34 if title else 26, bold=title)
    spacing = 9
    text = "\n".join(lines)
    bbox = draw.multiline_textbbox((0, 0), text, font=font, spacing=spacing, align="center")
    x = (box[0] + box[2] - (bbox[2] - bbox[0])) / 2
    y = (box[1] + box[3] - (bbox[3] - bbox[1])) / 2
    draw.multiline_text((x, y), text, font=font, fill=fill, spacing=spacing, align="center")


def arrow(draw, start, end, color, width=7):
    draw.line([start, end], fill=color, width=width)
    x2, y2 = end
    x1, y1 = start
    dx, dy = x2 - x1, y2 - y1
    length = max((dx * dx + dy * dy) ** 0.5, 1)
    ux, uy = dx / length, dy / length
    px, py = -uy, ux
    size = 18
    points = [
        (x2, y2),
        (x2 - ux * size + px * size * 0.55, y2 - uy * size + py * size * 0.55),
        (x2 - ux * size - px * size * 0.55, y2 - uy * size - py * size * 0.55),
    ]
    draw.polygon(points, fill=color)


def poly_arrow(draw, points, color, width=7):
    draw.line(points, fill=color, width=width, joint="curve")
    arrow(draw, points[-2], points[-1], color, width=width)


def build_diagram(path: Path):
    image = Image.new("RGB", (1600, 930), "white")
    draw = ImageDraw.Draw(image)
    title_font = load_font(38, bold=True)
    label_font = load_font(23, bold=True)
    note_font = load_font(20)
    draw.text((58, 36), "PowerOn 전원·신호 배선 토폴로지", font=title_font, fill="#0B2545")

    def box(coords, fill, outline, lines, title=True):
        draw.rounded_rectangle(coords, radius=24, fill=fill, outline=outline, width=4)
        draw_centered(draw, coords, lines, title=title)

    nucleo = (70, 250, 500, 570)
    driver = (625, 250, 1060, 570)
    motors = (1190, 220, 1530, 445)
    encoders = (1190, 540, 1530, 755)
    servos = (625, 690, 1060, 845)

    box(nucleo, "#E8F1FA", "#2E74B5", ["NUCLEO-F103RB", "PWM / IN / EN", "Encoder A/B", "Servo PWM"])
    box(driver, "#F2F4F7", "#667085", ["MAI-2MT-DC V3.0", "L298N Dual H-Bridge", "MOTOR A / MOTOR B"])
    box(motors, "#EAF7EF", "#2E7D4F", ["좌·우 RB35GM", "24 V 모터"])
    box(encoders, "#FFF7E0", "#D6A800", ["좌·우 엔코더", "A/B Quadrature", "52 count 가정"])
    box(servos, "#F4EEFF", "#7B61A8", ["좌·우 HS-311", "전륜 조향 서보"])

    arrow(draw, (500, 350), (625, 350), "#2E74B5")
    draw.text((520, 305), "8개 제어 신호", font=note_font, fill="#2E74B5")
    arrow(draw, (1060, 350), (1190, 350), "#2E7D4F")
    draw.text((1090, 305), "MOTOR A/B", font=note_font, fill="#2E7D4F")
    poly_arrow(draw, [(1190, 650), (1110, 650), (1110, 625), (565, 625), (500, 505)], "#D28B00")
    draw.text((760, 590), "엔코더 A/B 피드백", font=note_font, fill="#A06A00")
    arrow(draw, (500, 535), (625, 765), "#7B61A8")
    draw.text((480, 660), "서보 PWM", font=note_font, fill="#7B61A8")

    draw.rounded_rectangle((90, 105, 420, 180), radius=14, fill="#FFE7E7", outline="#9B1C1C", width=3)
    draw.text((132, 124), "24 V 모터 전원", font=label_font, fill="#9B1C1C")
    arrow(draw, (420, 143), (840, 250), "#9B1C1C", width=8)

    draw.rounded_rectangle((535, 105, 875, 180), radius=14, fill="#EAF7EF", outline="#2E7D4F", width=3)
    draw.text((578, 124), "5 V 로직 전원", font=label_font, fill="#2E7D4F")
    arrow(draw, (705, 180), (780, 250), "#2E7D4F")

    draw.rounded_rectangle((990, 105, 1445, 180), radius=14, fill="#F4EEFF", outline="#7B61A8", width=3)
    draw.text((1032, 124), "4.8~6.0 V 서보 전원", font=label_font, fill="#7B61A8")
    arrow(draw, (1210, 180), (930, 690), "#7B61A8")

    draw.line((85, 885, 1515, 885), fill="#20252B", width=9)
    draw.text((585, 850), "공통 GND: NUCLEO · 드라이버 · 엔코더 · 서보 전원", font=label_font, fill="#20252B")
    for x in (260, 840, 1360):
        draw.line((x, 845, x, 885), fill="#20252B", width=5)

    image.save(path, quality=95)


def add_metadata_line(doc: Document, label: str, value: str, *, rule=False):
    p = doc.add_paragraph()
    p.paragraph_format.space_before = Pt(0)
    p.paragraph_format.space_after = Pt(2 if not rule else 10)
    set_run_font(p.add_run(label + ": "), size=10, bold=True, color=NAVY)
    set_run_font(p.add_run(value), size=10, color="1D2939")
    if rule:
        add_paragraph_border_bottom(p, color=BLUE, size=8, space=7)
    return p


def build_report():
    REPORT_DIR.mkdir(parents=True, exist_ok=True)
    ASSET_DIR.mkdir(parents=True, exist_ok=True)
    build_diagram(DIAGRAM_PATH)

    doc = Document()
    apply_styles(doc)
    configure_page(doc)
    bullet_id = create_numbering(doc, bullet=True)

    kicker = doc.add_paragraph()
    kicker.paragraph_format.space_after = Pt(4)
    set_run_font(kicker.add_run("기술 배선 보고서"), size=10, bold=True, color=BLUE)
    title = doc.add_paragraph(style="Title")
    set_run_font(title.add_run("PowerOn RC카 후륜 구동계 및 조향계 배선 보고서"), size=25, color=NAVY, bold=True)
    subtitle = doc.add_paragraph(style="Subtitle")
    set_run_font(
        subtitle.add_run("NUCLEO-F103RB · MAI-2MT-DC V3.0 · RB-35GM 09TYPE W/EC 26P · Hitec HS-311"),
        size=12.5,
        color=MUTED,
    )
    add_metadata_line(doc, "문서 버전", "2.0")
    add_metadata_line(doc, "작성일", "2026-09-15")
    add_metadata_line(doc, "적용 펌웨어", "motor-v7-rb35gm-dual-20260915")
    add_metadata_line(doc, "적용 범위", "좌·우 후륜 독립 구동, 좌·우 엔코더 피드백, 전륜 듀얼 서보", rule=True)

    add_heading(doc, "요약", 1)
    add_body(
        doc,
        "본 보고서는 PowerOn RC카의 NUCLEO-F103RB, MAI-2MT-DC 듀얼 모터 드라이버, "
        "좌·우 RB35GM 엔코더 모터 및 전륜 HS-311 조향 서보를 안전하게 결선하고 최초 동작을 "
        "검증하기 위한 기준 문서이다. 펌웨어의 실제 핀 설정과 ST 공식 NUCLEO-64 커넥터 표를 "
        "대조하여 Arduino 헤더와 ST Morpho 헤더 위치를 함께 제시한다."
    )
    add_callout(
        doc,
        "핵심 안전 조건",
        "24 V는 모터 드라이버의 MOTOR VCC에만 연결한다. PA0/PA1 좌측 엔코더 입력에는 "
        "5 V 신호를 직접 연결하지 말고 3.3 V 이하로 변환하며, 모든 로직 계통의 GND를 공통화한다.",
    )

    caption = doc.add_paragraph("그림 1. PowerOn 전원 및 신호 흐름", style="Caption")
    caption.paragraph_format.page_break_before = True
    caption.alignment = WD_ALIGN_PARAGRAPH.CENTER
    picture_p = doc.add_paragraph()
    picture_p.alignment = WD_ALIGN_PARAGRAPH.CENTER
    run = picture_p.add_run()
    inline = run.add_picture(str(DIAGRAM_PATH), width=Inches(6.35))
    inline._inline.docPr.set("descr", "PowerOn RC카의 NUCLEO, 듀얼 모터 드라이버, 모터, 엔코더, 서보 및 전원 연결 토폴로지")

    add_heading(doc, "1. 시스템 구성 및 결선 원칙", 1)
    add_body(
        doc,
        "제어 신호는 NUCLEO의 3.3 V GPIO에서 출력되고, 모터 드라이버는 별도의 5 V 로직 전원과 "
        "24 V 모터 전원을 사용한다. 서보는 4.8~6.0 V 별도 전원을 사용하며 두 개의 합산 스톨 전류 "
        "약 1.6 A보다 충분한 여유를 둔다."
    )
    doc.add_paragraph("표 1. 주요 구성품과 역할", style="Caption")
    add_table(
        doc,
        ["구성품", "수량", "전원", "역할"],
        [
            ("NUCLEO-F103RB", "1", "USB 또는 규격 전원", "PWM·방향·활성 신호 출력, 엔코더 계수, UART 통신"),
            ("MAI-2MT-DC V3.0", "1", "로직 4.5~6 V / 모터 24 V", "좌·우 브러시드 DC 모터 독립 구동"),
            ("RB-35GM 09TYPE W/EC 26P", "2", "모터 24 V / 엔코더 사양 확인", "좌·우 후륜 구동 및 A/B 피드백"),
            ("Hitec HS-311", "2", "4.8~6.0 V", "좌·우 전륜 조향"),
            ("5→3.3 V 레벨 변환", "2채널", "3.3 V 계통", "좌측 엔코더 A/B 보호"),
        ],
        [2200, 800, 2200, 4160],
        center_columns={1},
    )

    add_heading(doc, "1.1 반드시 지켜야 할 결선 원칙", 2)
    principles = [
        "주전원과 USB를 모두 제거한 상태에서 배선하고, 결선이 끝난 뒤 연속성 및 단락을 확인한다.",
        "모터 24 V, 드라이버 로직 5 V, 서보 4.8~6.0 V를 분리한다. 어느 전원도 STM32 GPIO에 공급하지 않는다.",
        "NUCLEO GND, 드라이버 GND, 엔코더 GND, 서보 전원 GND를 한 기준점에서 공통화한다.",
        "PA0/PA1은 표준 I/O이므로 좌측 엔코더 출력이 5 V이면 반드시 레벨 변환한다. PA8/PA9는 FT(5 V tolerant) 입력이다.",
        "ENABLE은 Low-active이다. 정지 상태의 정상값은 ENABLE=HIGH, PWM=0이며 전원을 넣기 전에 이 논리를 확인한다.",
        "모터 전원 입력에는 적정 퓨즈와 비상 정지 수단을 두고, L298N 방열판 주변 통풍을 확보한다.",
    ]
    for item in principles:
        add_list_item(doc, item, bullet_id)

    add_heading(doc, "2. 전원 및 접지 배선", 1)
    add_body(
        doc,
        "전원은 기능별로 분리하되 GND만 공통으로 연결한다. 모터 기동 전류가 USB 또는 서보 전원으로 "
        "되돌아가지 않도록 전원선을 굵게 배치하고, 신호선·엔코더선과 물리적으로 분리한다."
    )
    doc.add_paragraph("표 2. 전원 배선표", style="Caption")
    add_table(
        doc,
        ["전원 계통", "정격", "연결 대상", "배선 및 보호 조건"],
        [
            ("모터 전원", "DC 24 V", "MAI-2MT-DC MOTOR VCC/GND", "퓨즈·비상정지 경유. 극성 확인 후 연결"),
            ("드라이버 로직", "DC 4.5~6 V", "드라이버 제어 커넥터 VCC/GND", "현재 보드 기준 VCC=9번, GND=10번. GPIO에서 공급 금지"),
            ("엔코더 전원", "출력 사양에 따름", "좌·우 엔코더 VCC/GND", "색상별 기능은 모두 TBD. 실측/데이터시트 확인 전 연결 금지"),
            ("서보 전원", "DC 4.8~6.0 V", "좌·우 HS-311 전원", "합산 스톨 약 1.6 A보다 충분한 여유. NUCLEO 5 V 핀 직결 금지"),
            ("제어 보드", "USB 또는 규격 전원", "NUCLEO-F103RB", "ST-LINK VCP 및 플래싱에 사용"),
            ("공통 접지", "0 V 기준", "모든 로직 GND", "스타형 접지 권장. 모터 대전류 귀환과 엔코더 기준선을 분리 후 한 점에서 결합"),
        ],
        [1700, 1300, 2600, 3760],
        center_columns={1},
    )
    add_callout(
        doc,
        "주의",
        "드라이버 9번 또는 NUCLEO +5 V에서 엔코더와 드라이버 로직을 함께 공급하려면 USB/ST-LINK "
        "전류 용량과 보드 점퍼 구성을 먼저 확인한다. 불확실하면 별도 안정화 5 V 전원을 사용하고 GND만 공통화한다.",
    )

    add_heading(doc, "2.1 권장 전원 투입 순서", 2)
    power_sequence_id = create_numbering(doc, bullet=False)
    for item in [
        "모터 전원을 끄고 바퀴를 지면에서 띄운다.",
        "NUCLEO와 드라이버 로직/엔코더 전원만 투입하여 GPIO 및 엔코더 상태를 확인한다.",
        "서보 전원을 투입하고 IDLE 상태에서 전륜이 일자로 정렬되는지 확인한다.",
        "마지막으로 24 V 모터 전원을 투입하고 10% 이하 저출력 시험을 수행한다.",
    ]:
        add_list_item(doc, item, power_sequence_id)

    add_heading(doc, "3. NUCLEO-F103RB 전체 신호 결선", 1)
    add_body(
        doc,
        "아래 표의 Arduino 헤더와 ST Morpho 헤더는 동일 MCU 핀의 대체 접근 지점이다. 동일 신호를 두 "
        "헤더에서 동시에 외부 회로에 연결하지 말고, 실제 조립에 편한 한 위치만 사용한다."
    )

    doc.add_paragraph("표 3. 좌·우 후륜 모터 제어 신호", style="Caption")
    add_table(
        doc,
        ["측", "신호", "MCU / 타이머", "Arduino 헤더", "Morpho", "드라이버 연결"],
        [
            ("좌", "L_PWM", "PA6 / TIM3_CH1", "D12 / CN5-5", "CN10-13", "L_PWM 실크"),
            ("좌", "L_IN1", "PB0", "A3 / CN8-4", "CN7-34", "L_IN1 실크"),
            ("좌", "L_IN2", "PB12", "-", "CN10-16", "L_IN2 실크"),
            ("좌", "L_ENABLE", "PB1", "-", "CN10-24", "L_ENABLE 실크, Low-active"),
            ("우", "R_PWM", "PA7 / TIM3_CH2", "D11 / CN5-4", "CN10-15", "6번 R_PWM"),
            ("우", "R_IN1", "PB10", "D6 / CN9-7", "CN10-25", "4번 R_IN1"),
            ("우", "R_IN2", "PB13", "-", "CN10-30", "7번 R_IN2"),
            ("우", "R_ENABLE", "PB11", "-", "CN10-18", "8번 R_ENABLE, Low-active"),
        ],
        [650, 1250, 1800, 1700, 1300, 2660],
        font_size=8.7,
        center_columns={0, 1, 2, 3, 4},
    )
    add_callout(
        doc,
        "좌측 커넥터 표기",
        "프로젝트 자료에는 우측 제어 커넥터 번호(4/6/7/8)만 확정되어 있다. 좌측은 번호를 임의로 "
        "추정하지 말고 드라이버 PCB의 L_IN1, L_IN2, L_PWM, L_ENABLE 실크를 기준으로 연결한다.",
        caution=False,
    )

    doc.add_paragraph("표 4. 엔코더·서보·통신 신호", style="Caption")
    add_table(
        doc,
        ["기능", "신호", "MCU / 타이머", "Arduino 헤더", "Morpho", "연결 대상 / 비고"],
        [
            ("좌 엔코더", "A", "PA0 / TIM2_CH1", "A0 / CN8-1", "CN7-28", "5→3.3 V 변환 후 연결"),
            ("좌 엔코더", "B", "PA1 / TIM2_CH2", "A1 / CN8-2", "CN7-30", "5→3.3 V 변환 후 연결"),
            ("우 엔코더", "A", "PA8 / TIM1_CH1", "D7 / CN9-8", "CN10-23", "확인된 A 연결, FT 입력"),
            ("우 엔코더", "B", "PA9 / TIM1_CH2", "D8 / CN5-1", "CN10-21", "확인된 B 연결, FT 입력"),
            ("좌 서보", "PWM", "PB8 / TIM4_CH3", "D15 / CN5-10", "CN10-3", "HS-311 좌측 신호"),
            ("우 서보", "PWM", "PB9 / TIM4_CH4", "D14 / CN5-9", "CN10-5", "HS-311 우측 신호"),
            ("PC 통신", "TX", "PA2 / USART2", "D1 / CN9-2", "CN10-35", "ST-LINK VCP 송신"),
            ("PC 통신", "RX", "PA3 / USART2", "D0 / CN9-1", "CN10-37", "ST-LINK VCP 수신"),
        ],
        [1300, 700, 1800, 1700, 1300, 2560],
        font_size=8.7,
        center_columns={0, 1, 2, 3, 4},
    )

    add_heading(doc, "3.1 좌측 엔코더 레벨 변환", 2)
    add_body(
        doc,
        "STM32F103RB 데이터시트의 I/O level 표에서 PA0/PA1은 FT 표시가 없는 표준 I/O이고, "
        "PA8/PA9는 FT로 표시된다. 따라서 좌측 엔코더가 5 V push-pull 출력이면 양방향이 아닌 "
        "단방향 5→3.3 V 버퍼 또는 적절한 저항 분압/레벨 시프터를 A와 B 각각에 적용한다. "
        "open-collector 출력이라면 출력 사양을 확인한 뒤 A/B를 3.3 V로 풀업하는 방식이 우선이다."
    )
    add_callout(
        doc,
        "금지 사항",
        "PA0/PA1에 5 V 풀업을 직접 연결하거나, 5 V push-pull A/B 신호를 직결하지 않는다. "
        "PB0/PB1에도 5 V 풀업을 직접 연결하지 않는다.",
    )

    add_heading(doc, "4. 장치별 배선 방법", 1)
    add_heading(doc, "4.1 MAI-2MT-DC 듀얼 모터 드라이버", 2)
    driver_sequence_id = create_numbering(doc, bullet=False)
    for item in [
        "전원 차단: MOTOR VCC, 로직 전원, USB가 모두 제거되었는지 확인한다.",
        "모터 출력: 좌측 후륜 모터를 MOTOR A, 우측 후륜 모터를 MOTOR B에 연결하고 두 선의 극성은 최초 시험에서 확정한다.",
        "제어 신호: 표 3에 따라 L/R의 PWM, IN1, IN2, ENABLE을 각각 연결한다.",
        "로직 전원: 드라이버 로직 VCC(현재 자료상 9번)에 4.5~6 V, GND(10번)에 공통 접지를 연결한다.",
        "모터 전원: 퓨즈 및 비상정지를 거친 24 V를 MOTOR VCC에 연결한다.",
        "정지 논리: 전원을 넣기 전 ENABLE이 HIGH로 유지되고 PWM이 0인지 확인한다. 드라이버 ENABLE은 Low-active이다.",
    ]:
        add_list_item(doc, item, driver_sequence_id)

    add_heading(doc, "4.2 RB-35GM 09TYPE 엔코더 모터", 2)
    for item in [
        "실제 여섯 선은 보라색 (Purple), 파란색 (Blue), 민트색 (Mint), 갈색 (Brown), 빨간색 (Red), 검은색 (Black)이며 모든 기능은 TBD이다.",
        "Motor terminal 1/2와 Encoder VCC/GND/A/B를 실물 측정 또는 데이터시트로 확인하기 전에는 어느 선도 24 V나 MCU GPIO에 연결하지 않는다.",
        "확인된 모터 두 선은 드라이버 MOTOR A/B 출력에 연결하고, 엔코더 전원선과 물리적으로 분리한다.",
        "확인된 좌측 A/B는 필요 시 레벨 변환기를 거쳐 PA0/PA1, 우측 A/B는 PA8/PA9에 연결한다.",
        "open-collector 출력이면 내부 풀업 유무를 확인하고, 필요한 경우 좌측은 3.3 V, 우측은 허용 전압 범위에 맞춰 외부 풀업한다.",
        "A/B가 뒤바뀌어도 RPM 절댓값은 보일 수 있으므로, 방향 정보를 사용할 계획이면 정회전 카운트 부호도 별도로 검증한다.",
    ]:
        add_list_item(doc, item, bullet_id)

    add_heading(doc, "4.3 HS-311 전륜 조향 서보", 2)
    for item in [
        "좌측 신호선을 PB8(D15), 우측 신호선을 PB9(D14)에 연결한다.",
        "서보 전원은 4.8~6.0 V 별도 공급을 사용하고, 두 서보 합산 스톨 전류 약 1.6 A보다 여유 있게 선정한다.",
        "서보 전원 GND를 NUCLEO GND와 공통화한다. 서보 전원을 NUCLEO GPIO 또는 3.3 V 핀에서 공급하지 않는다.",
        "최초 투입은 링크가 간섭하지 않는 상태에서 수행하며, 펌웨어의 안전 조향 범위 -45°~+55°를 유지한다.",
    ]:
        add_list_item(doc, item, bullet_id)

    add_heading(doc, "5. 실제 결선 작업 절차", 1)
    work_steps = [
        "배터리·24 V 전원·서보 전원·USB를 모두 분리하고 바퀴를 지면에서 띄운다.",
        "배선 라벨을 준비하여 L/R, PWM, IN1, IN2, ENABLE, A, B, VCC, GND를 양 끝에 표시한다.",
        "NUCLEO GND, 드라이버 로직 GND, 엔코더 GND, 서보 전원 GND를 공통 기준점에 연결한다.",
        "좌측 엔코더 A/B 경로에 5→3.3 V 레벨 변환 회로를 먼저 설치한다.",
        "표 3에 따라 좌·우 모터 제어 신호 8개를 연결한다. 좌측 드라이버 핀은 실크명을 확인한다.",
        "표 4에 따라 엔코더 A/B 4개와 서보 PWM 2개를 연결한다.",
        "좌·우 모터를 MOTOR A/B에 연결하고, 24 V 입력에는 퓨즈와 비상정지 수단을 배치한다.",
        "서보와 엔코더 전원을 연결하되 모터 24 V는 아직 투입하지 않는다.",
        "멀티미터로 24 V와 5 V/3.3 V 사이 단락, 각 전원 극성, 공통 GND 연속성을 확인한다.",
        "논리 전원부터 순차 투입하고 6장의 절차로 기능을 검증한 뒤 마지막에 모터 24 V를 투입한다.",
    ]
    work_sequence_id = create_numbering(doc, bullet=False)
    for item in work_steps:
        add_list_item(doc, item, work_sequence_id)

    add_heading(doc, "6. 전원 투입 전 검사표", 1)
    doc.add_paragraph("표 5. Pre-power 체크리스트", style="Caption")
    checklist = [
        ("□", "24 V 경로", "MOTOR VCC에만 연결되고 NUCLEO·서보·엔코더 전원과 분리됨"),
        ("□", "극성", "24 V, 5 V, 3.3 V, 서보 전원 극성이 모두 정확함"),
        ("□", "공통 GND", "NUCLEO·드라이버·엔코더·서보 전원 GND 사이 연속성 확보"),
        ("□", "좌측 레벨 변환", "PA0/PA1 입력이 0~3.3 V 범위이며 5 V 직결 없음"),
        ("□", "ENABLE 논리", "정지 시 L_ENABLE/R_ENABLE=HIGH, PWM=0"),
        ("□", "모터 출력", "MOTOR A/B 출력선과 엔코더선이 혼선 없이 분리됨"),
        ("□", "서보 전원", "4.8~6.0 V 별도 공급, 합산 전류 여유 확보"),
        ("□", "기계 안전", "바퀴가 지면에서 떠 있고 조향 링크 간섭이 없음"),
        ("□", "보호 장치", "퓨즈·비상정지·방열·배선 고정 상태 확인"),
    ]
    add_table(doc, ["확인", "검사항목", "합격 기준"], checklist, [700, 1900, 6760], font_size=9, center_columns={0, 1})

    add_heading(doc, "7. 최초 구동 및 기능 검증", 1)
    add_heading(doc, "7.1 논리 및 엔코더 확인", 2)
    test_sequence_id = create_numbering(doc, bullet=False)
    for item in [
        "모터 24 V를 끈 상태에서 펌웨어를 업로드하고 115200 8N1로 ST-LINK VCP에 접속한다.",
        "@STATUS에서 fw=motor-v7-rb35gm-dual-20260915, motor=RB35GM_09TYPE_26P, closed_loop_ready=0을 확인한다.",
        "좌측 바퀴를 손으로 돌려 measured_l_rpm, 우측 바퀴를 돌려 measured_r_rpm이 변하는지 확인한다.",
        "정지 상태에서 l_en_pin=1, r_en_pin=1, l_enabled=0, r_enabled=0, l_ccr=0, r_ccr=0인지 확인한다.",
    ]:
        add_list_item(doc, item, test_sequence_id)

    add_heading(doc, "7.2 저출력 모터 시험", 2)
    add_body(doc, "바퀴를 띄우고 즉시 정지할 수 있는 상태에서 다음 명령을 한 줄씩 실행한다.")
    add_code_block(
        doc,
        [
            "@STATUS",
            "@MOTOR 10 0     # 좌측 10%",
            "@MOTOR 0 0      # 정지",
            "@MOTOR 0 10     # 우측 10%",
            "@MOTOR 0 0      # 정지",
            "@MOTOR 10 10    # 양쪽 동시 10%",
            "@MOTOR 0 0      # 정지",
            "@STOP            # 보정 전 속도 기반 주행은 잠김",
        ],
    )
    add_body(
        doc,
        "양수 명령에서 특정 바퀴가 차량 전진 방향과 반대로 회전하면 모터선을 무작정 교환하기 전에 "
        "STM32_F103_RC_Car/Inc/rc_config.h의 해당 RC_LEFT_MOTOR_FORWARD_IN1_HIGH 또는 "
        "RC_RIGHT_MOTOR_FORWARD_IN1_HIGH 값을 반전하여 다시 빌드한다."
    )
    add_callout(
        doc,
        "자동 정지",
        "기본 명령 타임아웃은 1,500 ms이다. 실제 주행에서 @TIMEOUT 0으로 안전 정지를 해제하지 않는다.",
        caution=False,
    )

    add_heading(doc, "8. 이상 증상 및 점검 방법", 1)
    doc.add_paragraph("표 6. 고장 진단표", style="Caption")
    troubleshooting = [
        ("양쪽 모터 무반응", "24 V/5 V 미공급, 공통 GND 누락, ENABLE 극성 오류", "MOTOR VCC, 로직 VCC, GND, en_pin 상태 순서로 확인"),
        ("한쪽만 동작", "해당 PWM/IN/EN 단선 또는 드라이버 채널 문제", "@MOTOR 10 0 / 0 10으로 채널 분리, STATUS의 ccr·in·en 비교"),
        ("방향이 반대", "기계 장착 방향과 펌웨어 정회전 극성 불일치", "해당 FORWARD_IN1_HIGH 설정 반전 후 재빌드"),
        ("좌측 RPM이 0", "레벨 변환/PA0·PA1/A·B/전원 문제", "변환기 입출력 전압, TIM2 입력 배선, 공통 GND 확인"),
        ("우측 RPM이 0", "PA8·PA9/A·B/전원 문제", "확인된 VCC/GND/A/B와 출력 전압 점검"),
        ("RPM이 2배/4배", "엔코더 edge multiplier 불일치", "1회전 실측 후 DRIVE_ENCODER_TIMER_EDGE_MULTIPLIER 조정"),
        ("MCU 리셋/통신 오류", "모터 노이즈, 접지 전위차, 전원 강하", "스타 접지, 디커플링, 신호선 분리, uart_errors 확인"),
        ("드라이버 과열", "기동/정지 전류 초과, L298N 손실, 통풍 부족", "전류·온도 실측, 부하 감소, 방열 개선 또는 드라이버 변경"),
    ]
    add_table(
        doc,
        ["증상", "가능 원인", "점검 방법"],
        troubleshooting[:4],
        [2200, 3100, 4060],
        font_size=8.8,
    )
    doc.add_paragraph("표 6. 고장 진단표(계속)", style="Caption")
    add_table(
        doc,
        ["증상", "가능 원인", "점검 방법"],
        troubleshooting[4:],
        [2200, 3100, 4060],
        font_size=8.8,
    )

    add_heading(doc, "9. 결론 및 현장 기록", 1)
    add_body(
        doc,
        "본 결선은 좌·우 후륜 모터를 TIM3의 두 PWM 채널로 독립 구동하고, 좌측 TIM2와 우측 TIM1의 "
        "quadrature 입력으로 원시 카운트를 측정하도록 구성된다. 감속비·counts/rev·A/B 부호·PID를 "
        "확정하기 전에는 폐루프 주행이 잠기며 @MOTOR도 10%로 제한된다. 정상 동작의 전제는 전원 분리, "
        "공통 접지, 필요한 레벨 변환, Low-active ENABLE 논리 및 충분한 모터/서보 전원 용량이다."
    )
    doc.add_paragraph("표 7. 현장 확인 기록", style="Caption")
    add_table(
        doc,
        ["항목", "기록"],
        [
            ("작업 일시", ""),
            ("작업자 / 검토자", ""),
            ("보드 리비전", ""),
            ("모터 전원 / 무부하 전류", ""),
            ("좌·우 모터 라벨 감속비 1/xx", ""),
            ("좌·우 엔코더 1회전 카운트", ""),
            ("좌·우 정회전 극성 설정", ""),
            ("최종 점검 결과", "□ 합격   □ 조건부 합격   □ 재작업"),
        ],
        [2700, 6660],
        font_size=9.2,
    )

    add_heading(doc, "참고 자료", 1)
    refs = [
        ("프로젝트 설정", "STM32_F103_RC_Car/Inc/rc_config.h"),
        ("하드웨어 조건", "docs/hardware-requirements.ko.md"),
        ("모터/엔코더 상수", "Config/drive_hardware.h"),
        ("서보 상수", "Config/steering_servo_hardware.h"),
        ("ST UM1724 Rev.17", "STM32 Nucleo-64 boards (MB1136), Table 12 및 Table 27"),
        ("ST DS5319 Rev.20", "STM32F103x8/xB datasheet, Table 5 I/O level 및 alternate function"),
    ]
    for label, value in refs:
        p = doc.add_paragraph()
        p.paragraph_format.space_after = Pt(3)
        set_run_font(p.add_run(label + ": "), size=9.5, bold=True, color=NAVY)
        set_run_font(p.add_run(value), size=9.5, color="1D2939")
    p = doc.add_paragraph()
    p.paragraph_format.space_after = Pt(3)
    set_run_font(p.add_run("공식 보드 매뉴얼: "), size=9.5, bold=True, color=NAVY)
    add_hyperlink(p, "ST UM1724", "https://www.st.com/resource/en/user_manual/dm00105823.pdf")
    p = doc.add_paragraph()
    p.paragraph_format.space_after = Pt(3)
    set_run_font(p.add_run("공식 MCU 데이터시트: "), size=9.5, bold=True, color=NAVY)
    add_hyperlink(p, "STM32F103RB datasheet", "https://www.st.com/resource/en/datasheet/stm32f103rb.pdf")

    # Deterministic geometry audit before saving.
    for table_index, table in enumerate(doc.tables, start=1):
        tbl_w = table._tbl.tblPr.find(qn("w:tblW"))
        tbl_ind = table._tbl.tblPr.find(qn("w:tblInd"))
        if tbl_w is None or tbl_w.get(qn("w:w")) != str(CONTENT_WIDTH_DXA):
            raise RuntimeError(f"Table {table_index}: invalid table width")
        if tbl_ind is None or tbl_ind.get(qn("w:w")) != str(TABLE_INDENT_DXA):
            raise RuntimeError(f"Table {table_index}: invalid table indent")
        grid_widths = [int(c.get(qn("w:w"))) for c in table._tbl.tblGrid]
        if sum(grid_widths) != CONTENT_WIDTH_DXA:
            raise RuntimeError(f"Table {table_index}: invalid grid width sum")
        for row in table.rows:
            cell_widths = [int(cell._tc.get_or_add_tcPr().find(qn("w:tcW")).get(qn("w:w"))) for cell in row.cells]
            if cell_widths != grid_widths:
                raise RuntimeError(f"Table {table_index}: cell/grid width mismatch")

    core = doc.core_properties
    core.title = "PowerOn RC카 후륜 구동계 및 조향계 배선 보고서"
    core.subject = "NUCLEO-F103RB 기반 RC카의 전원, 모터, 엔코더 및 서보 배선"
    core.author = "PowerOn Project"
    core.keywords = "PowerOn, NUCLEO-F103RB, MAI-2MT-DC, wiring, 배선"
    doc.save(REPORT_PATH)
    print(REPORT_PATH)


if __name__ == "__main__":
    build_report()
