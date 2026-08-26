#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Builds the ESOGÜ Engineering and Architecture Faculty Journal submission.

The document is generated from the journal's own template
(backup_anomaly_detection/Dosya.docx) so that page setup, the two-column body
section and every named style (Makale Başlığı, Başlık-1, Paragraf, Eşitlik,
Kaynaklar) are the journal's, not ours.

    python3 makale_uret.py

Output: docs/ESOGU_MMF_Makale.docx
Figures come from docs/figures/ and are produced by makale_figurler.py.
"""
import copy
from pathlib import Path

from docx import Document
from docx.enum.table import WD_TABLE_ALIGNMENT
from docx.enum.text import WD_ALIGN_PARAGRAPH, WD_TAB_ALIGNMENT
from docx.oxml import OxmlElement
from docx.oxml.ns import qn
from docx.shared import Cm, Pt, RGBColor

HERE = Path(__file__).resolve().parent
PKG = HERE.parent
TEMPLATE = PKG / "backup_anomaly_detection" / "Dosya.docx"
FIG = HERE / "figures"
OUT = HERE / "ESOGU_MMF_Makale.docx"

FONT = "Cambria"
BODY_PT = 10
SMALL_PT = 9
COL_W = 8.35      # cm — one column of the two-column body
                  # (210 − 2·15 mm text width, less the 1.25 cm gutter)
PAGE_W = 17.9     # cm — full text width

INK = RGBColor(0x00, 0x00, 0x00)
GREY = RGBColor(0x44, 0x44, 0x44)


# ═════════════════════════════════════════════════════════════════════
# document skeleton
# ═════════════════════════════════════════════════════════════════════
def open_template():
    """Load the journal template and strip its body, keeping both sectPr."""
    doc = Document(str(TEMPLATE))
    body = doc.element.body
    keep_break = None
    for child in list(body.iterchildren()):
        if child.tag == qn("w:sectPr"):
            continue
        if child.find(".//" + qn("w:sectPr")) is not None:
            keep_break = child          # paragraph that closes section 0
            continue
        body.remove(child)
    return doc, keep_break


def style_run(run, size=BODY_PT, bold=False, italic=False, colour=INK,
              name=FONT):
    run.font.name = name
    run.font.size = Pt(size)
    run.bold = bold
    run.italic = italic
    run.font.color.rgb = colour
    rpr = run._element.get_or_add_rPr()
    rfonts = rpr.find(qn("w:rFonts"))
    if rfonts is None:
        rfonts = OxmlElement("w:rFonts")
        rpr.append(rfonts)
    for attr in ("w:ascii", "w:hAnsi", "w:cs", "w:eastAsia"):
        rfonts.set(qn(attr), name)
    return run


def set_cols(paragraph, num):
    """Close the section at `paragraph` with a `num`-column layout."""
    src = paragraph.part.document.element.body.find(qn("w:sectPr"))
    sect = copy.deepcopy(src)
    cols = sect.find(qn("w:cols"))
    if cols is None:
        cols = OxmlElement("w:cols")
        sect.append(cols)
    if num == 1:
        for attr in ("w:num", "w:equalWidth"):
            if cols.get(qn(attr)) is not None:
                del cols.attrib[qn(attr)]
        cols.set(qn("w:space"), "708")
    else:
        cols.set(qn("w:num"), str(num))
        cols.set(qn("w:equalWidth"), "1")
        cols.set(qn("w:space"), "709")
    ppr = paragraph._p.get_or_add_pPr()
    for old in ppr.findall(qn("w:sectPr")):
        ppr.remove(old)
    ppr.append(sect)


# ═════════════════════════════════════════════════════════════════════
# building blocks
# ═════════════════════════════════════════════════════════════════════
class Builder:
    def __init__(self, doc, anchor):
        self.doc = doc
        self.anchor = anchor          # section-0 closing paragraph
        self.body = doc.element.body
        self.fig_no = 0
        self.tab_no = 0
        self.eq_no = 0
        self.last = None
        self.pending_wide = None      # last paragraph of an open 1-column block

    # -- column-span bookkeeping --------------------------------------
    def _begin_wide(self):
        """Open a full-width (single-column) block, unless one is open."""
        if self.pending_wide is not None:
            return                    # consecutive wide elements share a block
        if self.last is not None:
            set_cols(self.last, 2)

    def _end_wide(self, paragraph):
        self.pending_wide = paragraph

    def _leave_wide(self):
        if self.pending_wide is not None:
            set_cols(self.pending_wide, 1)
            self.pending_wide = None

    # -- placement -----------------------------------------------------
    def _add(self, element, front=False):
        if front:
            self.anchor.addprevious(element)
        else:
            self.body.append(element)
            # keep the final sectPr last
            sect = self.body.find(qn("w:sectPr"))
            if sect is not None:
                self.body.remove(sect)
                self.body.append(sect)

    def para(self, text="", style="Paragraf", align=None, front=False,
             size=BODY_PT, bold=False, italic=False, colour=INK,
             space_before=None, space_after=None, keep_with_next=False,
             keep_wide=False):
        if not keep_wide and not front:
            self._leave_wide()
        p = self.doc.add_paragraph()
        self.body.remove(p._p)
        self._add(p._p, front)
        try:
            p.style = self.doc.styles[style]
        except KeyError:
            p.style = self.doc.styles["Normal"]
        if align is not None:
            p.alignment = align
        if space_before is not None:
            p.paragraph_format.space_before = Pt(space_before)
        if space_after is not None:
            p.paragraph_format.space_after = Pt(space_after)
        if keep_with_next:
            p.paragraph_format.keep_with_next = True
        if text:
            self.rich(p, text, size=size, bold=bold, italic=italic,
                      colour=colour)
        self.last = p
        return p

    def rich(self, paragraph, text, size=BODY_PT, bold=False, italic=False,
             colour=INK):
        """`**bold**` and `*italic*` inline markers."""
        buf = ""
        i = 0
        while i < len(text):
            if text.startswith("**", i):
                j = text.find("**", i + 2)
                if j > 0:
                    if buf:
                        style_run(paragraph.add_run(buf), size, bold, italic, colour)
                        buf = ""
                    style_run(paragraph.add_run(text[i + 2:j]), size, True,
                              italic, colour)
                    i = j + 2
                    continue
            if text[i] == "*":
                j = text.find("*", i + 1)
                if j > 0:
                    if buf:
                        style_run(paragraph.add_run(buf), size, bold, italic, colour)
                        buf = ""
                    style_run(paragraph.add_run(text[i + 1:j]), size, bold,
                              True, colour)
                    i = j + 1
                    continue
            buf += text[i]
            i += 1
        if buf:
            style_run(paragraph.add_run(buf), size, bold, italic, colour)
        return paragraph

    # -- headings ------------------------------------------------------
    def h1(self, text):
        return self.para(text, style="Başlık-1", size=BODY_PT, bold=True,
                         space_before=12, space_after=2, keep_with_next=True)

    def h2(self, text):
        return self.para(text, style="Başlık-1", size=BODY_PT, bold=True,
                         space_before=8, space_after=2, keep_with_next=True)

    def p(self, text):
        return self.para(text, style="Paragraf",
                         align=WD_ALIGN_PARAGRAPH.JUSTIFY,
                         space_before=6, space_after=0)

    # -- equations -----------------------------------------------------
    def equation(self, text):
        self.eq_no += 1
        self.para("", style="Normal", space_before=0, space_after=0)
        p = self.para(style="Eşitlik", space_before=0, space_after=0)
        pf = p.paragraph_format
        for existing in list(pf.tab_stops):
            pf.tab_stops.clear_all()
            break
        pf.tab_stops.add_tab_stop(Cm(COL_W), WD_TAB_ALIGNMENT.RIGHT)
        style_run(p.add_run("\t" if text.startswith("\t") else ""), BODY_PT)
        self.rich(p, text, size=BODY_PT, italic=True)
        style_run(p.add_run("\t"), BODY_PT)
        style_run(p.add_run(f"({self.eq_no})"), BODY_PT)
        self.para("", style="Normal", space_before=0, space_after=0)
        return self.eq_no

    # -- figures -------------------------------------------------------
    def figure(self, filename, caption, wide=True, width_cm=None):
        path = FIG / filename
        if not path.exists():
            print(f"  ! missing figure {filename}")
            return None
        self.fig_no += 1
        if wide:
            self._begin_wide()
        else:
            self._leave_wide()
        width = width_cm or (PAGE_W if wide else COL_W)
        pic = self.para("", style="Normal", align=WD_ALIGN_PARAGRAPH.CENTER,
                        space_before=6, space_after=2, keep_with_next=True,
                        keep_wide=True)
        pic.add_run().add_picture(str(path), width=Cm(width))
        cap = self.para("", style="Normal", align=WD_ALIGN_PARAGRAPH.CENTER,
                        space_before=0, space_after=6, keep_wide=True)
        name, _, extra = caption.partition("|")
        self.rich(cap, f"**Figure {self.fig_no}.** {name.strip()}",
                  size=SMALL_PT)
        if extra.strip():
            self.rich(cap, "  " + extra.strip(), size=SMALL_PT - 0.5,
                      colour=GREY)
        if wide:
            self._end_wide(cap)
        self.last = cap
        return self.fig_no

    # -- tables --------------------------------------------------------
    def table(self, caption, header, rows, widths=None, wide=False,
              align_right=None, note=None):
        self.tab_no += 1
        if wide:
            self._begin_wide()
        else:
            self._leave_wide()
        cap = self.para("", style="Normal", align=WD_ALIGN_PARAGRAPH.LEFT,
                        space_before=8, space_after=3, keep_with_next=True,
                        keep_wide=True)
        self.rich(cap, f"**Table {self.tab_no}.** {caption}", size=SMALL_PT)

        ncol = len(header)
        tbl = self.doc.add_table(rows=1 + len(rows), cols=ncol)
        self.body.remove(tbl._tbl)
        self._add(tbl._tbl)
        tbl.alignment = WD_TABLE_ALIGNMENT.CENTER
        tbl.autofit = False
        total = PAGE_W if wide else COL_W
        if widths is None:
            widths = [total / ncol] * ncol
        scale = total / sum(widths)
        widths = [w * scale for w in widths]

        align_right = align_right or []
        data = [header] + list(rows)
        for r, row in enumerate(data):
            for c, val in enumerate(row):
                cell = tbl.cell(r, c)
                cell.width = Cm(widths[c])
                par = cell.paragraphs[0]
                par.paragraph_format.space_before = Pt(1)
                par.paragraph_format.space_after = Pt(1)
                par.alignment = (WD_ALIGN_PARAGRAPH.RIGHT if c in align_right
                                 and r > 0 else WD_ALIGN_PARAGRAPH.LEFT)
                self.rich(par, str(val), size=SMALL_PT - 1.0, bold=(r == 0))
        self._fix_layout(tbl, widths)
        self._rule_table(tbl)
        if note:
            n = self.para("", style="Normal", space_before=2, space_after=6,
                          align=WD_ALIGN_PARAGRAPH.JUSTIFY, keep_wide=True)
            self.rich(n, note, size=SMALL_PT - 1, italic=True, colour=GREY)
            self.last = n
        else:
            self.last = self.para("", style="Normal", space_before=0,
                                  space_after=4, keep_wide=True)
        if wide:
            self._end_wide(self.last)
        return self.tab_no

    @staticmethod
    def _fix_layout(tbl, widths_cm):
        """Pin the column grid so Word and LibreOffice stop autofitting."""
        twips = [int(round(w * 567)) for w in widths_cm]
        tblpr = tbl._tbl.tblPr
        for tag in ("w:tblW", "w:tblLayout"):
            for old in tblpr.findall(qn(tag)):
                tblpr.remove(old)
        tw = OxmlElement("w:tblW")
        tw.set(qn("w:w"), str(sum(twips)))
        tw.set(qn("w:type"), "dxa")
        tblpr.append(tw)
        lay = OxmlElement("w:tblLayout")
        lay.set(qn("w:type"), "fixed")
        tblpr.append(lay)
        for old in tbl._tbl.findall(qn("w:tblGrid")):
            tbl._tbl.remove(old)
        grid = OxmlElement("w:tblGrid")
        for t in twips:
            gc = OxmlElement("w:gridCol")
            gc.set(qn("w:w"), str(t))
            grid.append(gc)
        tblpr.addnext(grid)
        for row in tbl.rows:
            for cell, t in zip(row.cells, twips):
                tcpr = cell._tc.get_or_add_tcPr()
                for old in tcpr.findall(qn("w:tcW")):
                    tcpr.remove(old)
                el = OxmlElement("w:tcW")
                el.set(qn("w:w"), str(t))
                el.set(qn("w:type"), "dxa")
                tcpr.append(el)

    @staticmethod
    def _rule_table(tbl):
        """Horizontal rules only: above and below the header, below the last row."""
        rows = tbl.rows
        for idx, row in enumerate(rows):
            for cell in row.cells:
                tcpr = cell._tc.get_or_add_tcPr()
                for old in tcpr.findall(qn("w:tcBorders")):
                    tcpr.remove(old)
                borders = OxmlElement("w:tcBorders")
                for edge in ("top", "left", "bottom", "right"):
                    el = OxmlElement(f"w:{edge}")
                    show = ((edge == "top" and idx <= 1)
                            or (edge == "bottom" and idx == len(rows) - 1))
                    el.set(qn("w:val"), "single" if show else "nil")
                    if show:
                        el.set(qn("w:sz"), "8" if idx == 0 or
                               idx == len(rows) - 1 else "4")
                        el.set(qn("w:space"), "0")
                        el.set(qn("w:color"), "000000")
                    borders.append(el)
                tcpr.append(borders)


# ═════════════════════════════════════════════════════════════════════
# front matter
# ═════════════════════════════════════════════════════════════════════
TITLE_EN = "FROM OFFLINE FUSION TO ONLINE DEPLOYMENT: ANOMALY DETECTION ON A UR10e COBOT"
TITLE_TR = "ÇEVRİMDIŞI BİRLEŞİMDEN ÇEVRİMİÇİ GERÇEKLEMEYE UR10e ANOMALİ TESPİTİ"

KEYWORDS_EN = ["Anomaly detection", "Collaborative robots", "LSTM autoencoder",
               "Score-level fusion", "Real-time deployment"]
KEYWORDS_TR = ["Anomali tespiti", "İşbirlikçi robotlar", "LSTM özkodlayıcı",
               "Skor birleşimi", "Gerçek zamanlı sistem"]

ABSTRACT_EN = (
    "Early detection of anomalies in collaborative robots matters for operator "
    "safety and production continuity. Previous work introduced a score-level "
    "fusion of a physics-informed residual autoencoder and a data-driven raw signal "
    "autoencoder, evaluated entirely offline. This paper reports the path from "
    "that result to a detector running on a physical UR10e cell. The pipeline "
    "was rebuilt and audited at measurement level; seven findings are documented, "
    "the decisive one being that the driver writes motor current, not torque, into "
    "the effort field, so the previously published figures came from a mixed-unit "
    "pipeline. That arrangement is reproduced explicitly and a corrected pipeline "
    "is rebuilt on 600 continuous runs. Under the original protocol fusion still "
    "outperforms both single models "
    "(F1 0.792 against 0.613 and 0.579) and the fault-type asymmetry motivating "
    "it is preserved. Reporting the evaluation split separately, however, shows "
    "that four fifths of the evaluation windows were also training windows, and "
    "that on windows never seen in training the two models exchange roles. The "
    "detector was then implemented as a 500 Hz ROS 2 node and commissioned on the "
    "robot over a two-hour session covering two production use cases with manually "
    "provoked anomalies. The threshold derived from offline data proved seven times "
    "too low, because the residual is pose dependent and spans three orders of "
    "magnitude across operating regimes, and the adaptive alarm rule that helped "
    "offline caught no verified event. The threshold was re-measured from "
    "all logged decisions, after which the detector raises no false alarm in 37 "
    "minutes of autonomous operation."
)

ABSTRACT_TR = (
    "İşbirlikçi robotlarda anomalilerin erken tespiti operatör güvenliği ve üretim "
    "sürekliliği açısından kritiktir. Önceki çalışmada, fizik tabanlı bir kalıntı "
    "özkodlayıcısı ile veri güdümlü bir ham sinyal özkodlayıcısının skor düzeyinde "
    "birleşimi sunulmuş ve yalnızca çevrimdışı değerlendirilmişti. Bu makale, o "
    "sonuçtan gerçek bir UR10e hücresinde çalışan dedektöre uzanan sürecin tamamını "
    "aktarmaktadır. Hat yeniden kurulmuş ve ölçüm düzeyinde denetlenmiştir; yedi "
    "bulgu belgelenmiş olup belirleyici olanı, robot sürücüsünün effort alanına "
    "tork değil motor akımı yazmasıdır — daha önce yayımlanan sayılar birimleri "
    "karışık bir hattan gelmektedir. Bu düzen açıkça yeniden üretilmiş, düzeltilmiş "
    "ve sızıntısız hat 600 kesintisiz koşu üzerinde kurulmuştur. Özgün "
    "değerlendirme protokolü altında birleşim iki tekil modeli de geçmekte "
    "(F1 0,792'ye karşılık 0,613 ve 0,579) ve arıza tipi asimetrisi korunmaktadır. "
    "Buna karşılık bölünme ayrı raporlandığında, değerlendirme pencerelerinin beşte "
    "dördünün aynı zamanda eğitim penceresi olduğu ve hiç görülmemiş pencerelerde "
    "iki modelin rollerini değiştirdiği görülmektedir. Dedektör ardından 500 Hz'lik "
    "bir ROS 2 düğümü olarak gerçeklenmiş ve iki üretim senaryosunun elle "
    "kışkırtılan anomalilerle koşturulduğu iki saatlik bir oturumda devreye "
    "alınmıştır. Çevrimdışı veriden türetilen eşik gerçek donanımda yedi kat düşük "
    "kalmıştır; sebebi kalıntının poza bağlı olması ve rejimler arasında üç "
    "büyüklük mertebesi değişmesidir. Çevrimdışı ortamda yardımcı olan uyarlanabilir "
    "kural doğrulanmış hiçbir olayı yakalamamıştır. Eşik tüm karar kayıtlarından "
    "yeniden ölçülmüş, sonrasında dedektör 37 dakikalık otonom çalışmada tek bir "
    "yanlış alarm üretmemiştir."
)


def front_matter(b):
    doc = b.doc

    def title(text):
        p = b.para("", style="Makale Başlığı",
                   align=WD_ALIGN_PARAGRAPH.CENTER, front=True,
                   space_before=0, space_after=0)
        style_run(p.add_run(text), 12, bold=True)
        return p

    title(TITLE_EN)
    b.para("", style="Normal", front=True)
    p = b.para("", style="Normal", align=WD_ALIGN_PARAGRAPH.CENTER, front=True)
    style_run(p.add_run("Adı SOYADI"), BODY_PT)
    style_run(p.add_run("1*"), BODY_PT - 3.5).font.superscript = True
    style_run(p.add_run(",  Adı SOYADI"), BODY_PT)
    style_run(p.add_run("2"), BODY_PT - 3.5).font.superscript = True
    style_run(p.add_run("  (Dergi editörlüğü tarafından basım aşamasında "
                        "yazılacaktır.)"), BODY_PT)
    for i in (1, 2):
        p = b.para("", style="Normal", align=WD_ALIGN_PARAGRAPH.CENTER,
                   front=True)
        style_run(p.add_run(f"{i} "), SMALL_PT - 1)
        style_run(p.add_run(f"Yazar {i} Adresi, ORCID No : "
                            "https://orcid.org/"), SMALL_PT - 1)
    b.para("", style="Normal", front=True)

    abstract_block(b, "Keywords", "Abstract", KEYWORDS_EN, ABSTRACT_EN,
                   "Research Article")
    b.para("", style="Normal", front=True)
    title(TITLE_TR)
    b.para("", style="Normal", front=True)
    abstract_block(b, "Anahtar Kelimeler", "Öz", KEYWORDS_TR, ABSTRACT_TR,
                   "Araştırma Makalesi")
    b.para("", style="Normal", front=True)


def abstract_block(b, kw_head, ab_head, keywords, abstract, kind):
    tbl = b.doc.add_table(rows=3, cols=2)
    b.body.remove(tbl._tbl)
    b.anchor.addprevious(tbl._tbl)
    tbl.alignment = WD_TABLE_ALIGNMENT.CENTER
    tbl.autofit = False
    widths = [4.6, 13.3]
    heads = [kw_head, ab_head]
    for c in range(2):
        cell = tbl.cell(0, c)
        cell.width = Cm(widths[c])
        par = cell.paragraphs[0]
        par.paragraph_format.space_before = Pt(2)
        par.paragraph_format.space_after = Pt(2)
        style_run(par.add_run(heads[c]), BODY_PT, bold=True)

    cell = tbl.cell(1, 0)
    cell.width = Cm(widths[0])
    for i, kw in enumerate(keywords):
        par = cell.paragraphs[0] if i == 0 else cell.add_paragraph()
        par.paragraph_format.space_before = Pt(0)
        par.paragraph_format.space_after = Pt(0)
        par.alignment = WD_ALIGN_PARAGRAPH.LEFT
        style_run(par.add_run(kw), SMALL_PT, italic=True)

    cell = tbl.cell(1, 1)
    cell.width = Cm(widths[1])
    par = cell.paragraphs[0]
    par.alignment = WD_ALIGN_PARAGRAPH.JUSTIFY
    par.paragraph_format.space_before = Pt(2)
    par.paragraph_format.space_after = Pt(2)
    style_run(par.add_run(abstract), BODY_PT, italic=True)

    cell = tbl.cell(2, 0)
    par = cell.paragraphs[0]
    style_run(par.add_run(kind), SMALL_PT, bold=True)
    par = tbl.cell(2, 1).paragraphs[0]
    style_run(par.add_run("Received / Geliş :               "
                          "Accepted / Kabul :"), SMALL_PT, colour=GREY)
    Builder._fix_layout(tbl, widths)
    Builder._rule_table(tbl)


# ═════════════════════════════════════════════════════════════════════
# body
# ═════════════════════════════════════════════════════════════════════
def body(b):
    # ─────────────────────────────────────────── 1. Introduction ─────
    b.h1("1. Introduction")
    b.p(
        "Collaborative robots share their working volume with human operators, so "
        "anomalies such as a collision, a degrading motor or a failing sensor have "
        "to be detected before they become incidents. Two families of methods "
        "dominate. Physics-based residual methods compare a dynamic model of the "
        "robot with the measured forces and treat the difference as a fault "
        "indicator; they reduce dimensionality and suppress noise, but they also "
        "suppress low-amplitude sensor anomalies. Data-driven methods learn complex "
        "patterns directly from high-dimensional raw signals, but they lack the "
        "structure a physical model provides. The two families therefore fail on "
        "different fault types, and that asymmetry can be exploited by fusing them."
    )
    b.p(
        "In earlier work we proposed exactly such a fusion for a UR10e cobot "
        "(Yılmaz, Kahraman, Yılmaz, Yavuz and Yayan, 2026). A residual Long "
        "Short-Term Memory (LSTM) autoencoder operating on twelve intrinsic and "
        "extrinsic residual channels derived from a Functional Mock-up Unit (FMU) "
        "inverse dynamics model was combined, at score level, with a raw LSTM "
        "autoencoder operating on twenty-four direct sensor channels. That study was "
        "evaluated entirely offline on a recorded dataset, and its own future-work "
        "list named a single most important open item: running the system online and "
        "simultaneously with the robot."
    )
    b.p(
        "This paper closes that item and presents a comprehensive empirical analysis "
        "of the costs incurred by the transfer. The pipeline was rebuilt from "
        "scratch, audited against the original at "
        "measurement level rather than at claim level, corrected where the audit "
        "found defects, implemented as a real-time Robot Operating System 2 (ROS 2) "
        "node, and commissioned on a physical UR10e cell. Every stage produced "
        "measurements that do not appear in the offline literature, and some of them "
        "invert conclusions that the offline evaluation had supported."
    )
    b.p(
        "The gap this study fills is therefore not a new architecture. Fusion "
        "frameworks for robot anomaly detection are numerous and are almost always "
        "evaluated on recorded data with injected faults. What is rarely reported is "
        "which parts of such a framework survive contact with hardware: whether the "
        "operating threshold transfers, whether the normalisation is even causal, "
        "whether the auxiliary decision rules still help, and what the residual "
        "actually does when the arm changes pose. This paper reports those "
        "measurements for one complete system."
    )
    b.p("The contributions are:")
    for item in (
        "an audit of an independently rebuilt pipeline against the reference study, "
        "documenting seven measurable findings, of which five were corrected, one "
        "was deliberately accepted and one is a typesetting inconsistency;",
        "an explicit reproduction of the data path from which the previously "
        "published figures originate, together with a corrected, physically "
        "consistent and leakage-free pipeline and its re-evaluation;",
        "a causal formulation of the fusion normalisation and an operating "
        "threshold for the fused score, neither of which the offline method defines;",
        "a real-time ROS 2 implementation whose feature engine is numerically "
        "identical to the offline pipeline, with a measured latency budget;",
        "a commissioning report from a physical UR10e cell in which the "
        "simulation-derived threshold is shown to be seven times too low, the "
        "residual is shown to be pose dependent over three orders of magnitude, and "
        "the adaptive alarm rule is shown to invert its usefulness.",
    ):
        p = b.para("", style="Paragraf", align=WD_ALIGN_PARAGRAPH.JUSTIFY,
                   space_before=3, space_after=0)
        p.paragraph_format.left_indent = Cm(0.4)
        b.rich(p, "•  " + item)
    b.p(
        "The remainder of the paper is organised as follows. Section 2 reviews the "
        "related literature. Section 3 describes the platform, the data, the audit "
        "and the corrected pipeline, and the online implementation. Section 4 "
        "presents the offline and the real-cell findings. Section 5 discusses them "
        "and states the limitations, and Section 6 concludes."
    )

    # ──────────────────────────────────── 2. Literature review ───────
    b.h1("2. Literature Review")
    b.p(
        "Physics-based detection for manipulators is a mature field. Haddadin, De "
        "Luca and Albu-Schäffer (2017) survey collision detection, isolation and "
        "identification and establish the residual observer as the canonical tool. "
        "Li, Han and Xiong (2020) place a force/torque sensor at the bedplate and "
        "detect collisions from the resulting residual, while Zhang, Chen and Zou "
        "(2024) use an external torque observer for the same purpose. "
        "Katsampiris-Salgado et al. (2024) address high-payload collaborative "
        "assembly, where the model error itself becomes the limiting factor. The "
        "common weakness of this family is that whatever the model does not "
        "represent — friction, payload, joint elasticity — is indistinguishable from "
        "a fault, and whatever the model filters well is also filtered away when it "
        "is the fault."
    )
    b.p(
        "Data-driven detection for multivariate time series is surveyed by Darban, "
        "Webb, Pan, Aggarwal and Salehi (2024). Reconstruction-based autoencoders "
        "are the dominant design: Malhotra, Vig, Shroff and Agarwal (2015) "
        "introduced LSTM networks for time-series anomaly detection, and Malhotra, "
        "Ramakrishnan, Anand, Vig, Agarwal and Shroff (2016) extended the idea to a "
        "multi-sensor encoder–decoder. Park, Hoshi and Kemp (2018) applied an "
        "LSTM-based variational autoencoder to robot-assisted feeding, one of the "
        "few studies to report on a physical robot. These methods need no fault "
        "labels, but they inherit whatever bias the training distribution carries."
    )
    b.p(
        "Hybrid approaches attempt to combine the two. Liu et al. (2025) constrain "
        "an LSTM autoencoder with mechanism knowledge for pump operations, but do "
        "not study the fusion of two separate models' scores. Huang, Chen, Deng and "
        "Huang (2024) apply graph attention and a Transformer to multivariate "
        "anomaly detection without any physics-based preprocessing stage. Correia, "
        "Goos, Klein, Bäck and Kononova (2024) survey online model-based anomaly "
        "detection specifically, and identify threshold selection and non-stationary "
        "operating conditions as open research challenges — precisely the two "
        "problems that this paper measures on hardware."
    )
    b.p(
        "The transfer of a model calibrated on simulated or recorded data to a "
        "physical robot is studied mostly in the control and reinforcement learning "
        "literature, where it is known as the reality gap (Zhao, Queralta and "
        "Westerlund, 2020). For anomaly detection the equivalent question — does the "
        "decision threshold transfer? — is seldom asked, because most studies never "
        "define an operating threshold at all, reporting instead the best achievable "
        "F1 over all thresholds. That statistic requires the labels it is supposed "
        "to predict and therefore does not exist online. The present study makes "
        "this gap explicit and closes it with a causal threshold rule, then measures "
        "what happens when the rule is applied to real hardware."
    )

    # ───────────────────────────────────────────── 3. Method ─────────
    b.h1("3. Method")

    b.h2("3.1. Platform And Data Collection")
    b.p(
        "Experiments were carried out on a robotic quality inspection platform "
        "consisting of a Universal Robots UR10e collaborative robot (six joints, "
        "12.5 kg payload) mounted on a Festo linear axis, together with an "
        "inspection chassis frame, shown in Figure 1. The target application is "
        "automated visual quality inspection of automotive body panels. A "
        "force/torque sensor (FTS) is mounted at the end effector. The cell runs "
        "ROS 2 Humble with MoveIt 2, and the recorded motions were produced by "
        "eleven different motion planning algorithms so that the normal operating "
        "distribution is not dominated by a single planner."
    )
    b.figure(
        "fig1_platform.png",
        "Robotic Quality Inspection Platform | UR10e robot on a Festo linear "
        "axis, with the inspection chassis frame on the right.",
        wide=False, width_cm=6.2)
    b.p(
        "The source dataset is a 79-day export of joint states and wrench "
        "measurements, 1,124,432 samples nominally at 500 Hz. Model training was "
        "performed on a laptop with an Intel Core i7-13650HX processor and an "
        "NVIDIA RTX 4060 GPU; the deployment workstation is an Intel Core i9-14900 "
        "with 64 GB of memory and an NVIDIA RTX 4000 Ada GPU."
    )
    b.p(
        "The study uses no human subjects and required no ethics committee "
        "approval. All measurements were collected on laboratory equipment of the "
        "ESOGÜ Intelligent Systems Application and Research Centre, and the "
        "principles of research and publication ethics were observed throughout. "
        "The anomalies of the commissioning campaign were provoked deliberately by "
        "trained operators at reduced speed, with the robot's own protective-stop "
        "function active at all times; one of them, reported in Section 4.7, "
        "involved an operator's hand being caught between two links, and the "
        "protective stop halted the robot without injury."
    )

    b.h2("3.2. Data Investigation And Preparation")
    b.p(
        "The first stage of the audit was the data itself, and it produced the "
        "largest single correction. The export is not globally ordered by "
        "timestamp. Measured over the 1,124,432 samples, 321,841 consecutive pairs "
        "(28.6 %) are discontinuous, 96,107 of them with a negative or zero time "
        "step; the median step is the expected 2.00 ms but the range spans from "
        "−5.5·10⁹ ms to +3.6·10⁹ ms. Three quarters of the sliding windows "
        "(33,700 of 44,974, 74.9 %) therefore splice together two recording "
        "sessions that may be hours or days apart."
    )
    b.p(
        "This matters twice. It puts an arbitrary jump in the middle of the dynamics "
        "an LSTM autoencoder is asked to learn, and it corrupts the joint "
        "acceleration. Acceleration is obtained from the velocity channel by a "
        "Savitzky–Golay derivative (window 51, order 3) (Savitzky and Golay, 1964), "
        "which is a convolution: across a splice it produces a non-physical spike. "
        "Since acceleration enters the inverse dynamics directly through the M(q)q̈ "
        "term, the error propagates into the residual. Measured on the same 200,000 "
        "samples, the standard deviation of q̈ derived across splices is two to six "
        "times larger than when the derivative is taken inside continuous runs."
    )
    b.p(
        "The correction is to segment the export into physically continuous runs. "
        "Two consecutive samples belong to the same run only if the time step lies "
        "in (0, 8 ms] and the joint displacement stays below what the joint "
        "velocity limit allows over that step; the second condition catches gaps "
        "that a plausible-looking timestamp hides. "
        "Runs shorter than 150 samples are discarded, and each run is resampled onto "
        "a uniform 2 ms grid. Table 1 gives the outcome. The reduction in sample "
        "count is not a loss: the discarded samples are exactly those between which "
        "no physical continuity exists."
    )
    b.table(
        "Outcome Of Data Preparation.",
        ["Quantity", "Value"],
        [["Raw exported samples", "1,124,432"],
         ["Physically continuous runs", "600"],
         ["Prepared samples", "342,480  (685 s of robot time)"],
         ["Interpolated samples", "5.02 %"],
         ["Run length (median / min / max)", "227 / 155 / 35,860"]],
        widths=[4.9, 3.65], align_right=[1])

    b.h2("3.3. Current-To-Torque Calibration")
    b.p(
        "The UR ROS 2 driver writes the motor current, in amperes, into the effort "
        "field of the joint state message; the field is filled directly from the "
        "actual current reported by the controller. The inverse dynamics model, in "
        "contrast, produces newton metres. A conversion coefficient is therefore "
        "required before the residual of Equation (2) can be formed at all."
    )
    b.p(
        "In a first rebuild this coefficient was obtained by regressing measured "
        "current on model torque, and verified by repeating the same regression. "
        "That verification is circular: the coefficient is chosen so as to satisfy "
        "the equality that is then tested. The coefficient was therefore re-derived "
        "from a source independent of the residual under test. When the robot is "
        "nearly at rest (|q̇| < 0.005 rad/s) the joint torque is dominated by "
        "gravity, so g(q) ≈ k·(i − i₀) with g(q) taken from the closed-form gravity "
        "vector rather than from the model being audited. Of 197,090 quasi-static "
        "samples, 65,697 entered the regression. Table 2 lists the result."
    )
    b.table(
        "Measured Current-To-Torque Coefficients And Their Sensitivity.",
        ["Joint", "Nm/A", "R²", "Meas.", "ρ −50 %", "ρ +50 %"],
        [["shoulder_pan", "10.522", "—", "no", "0.968", "0.995"],
         ["shoulder_lift", "10.522", "0.984", "yes", "0.162", "0.637"],
         ["elbow", "9.130", "0.993", "yes", "0.023", "0.758"],
         ["wrist_1", "3.409", "0.000", "no", "0.890", "0.968"],
         ["wrist_2", "3.409", "0.012", "no", "0.957", "0.993"],
         ["wrist_3", "3.409", "0.010", "no", "0.994", "0.999"]],
        widths=[3.3, 2.5, 2.3, 2.2, 3.7, 3.9], wide=True,
        align_right=[1, 2, 4, 5],
        note="R² is that of the quasi-static gravity regression and “Meas.” marks "
             "the joints on which the coefficient could be measured directly. The "
             "last two columns give the correlation between the intrinsic residual "
             "recomputed with the coefficient perturbed by ∓50 % and the nominal "
             "one, over all 342,480 prepared samples. Coefficients that could not "
             "be measured are copied within the same module family or scaled by "
             "the torque-limit ratio, and the node warns about them at start-up.")

    b.p(
        "The consequence of the four assumed coefficients was measured rather than "
        "argued. Each coefficient was perturbed by ±50 % and the intrinsic residual "
        "recomputed, which requires no further evaluation of the inverse dynamics "
        "because the model torque is stored separately from the measurement. On the "
        "four joints whose coefficient could not be measured, the perturbation "
        "leaves the residual almost unchanged in shape: the correlation with the "
        "nominal residual is between 0.890 and 0.999 and the effect is close to a "
        "pure rescaling, with the standard deviation changing by factors of 0.55 to "
        "1.45. Since the autoencoders standardise their inputs, a transformation of "
        "that form cannot alter the learned dynamics. On the two joints whose "
        "coefficient was measured, the same perturbation is destructive: the "
        "correlation falls to 0.162 for shoulder_lift and to 0.023 for the elbow at "
        "−50 %. The asymmetry is itself diagnostic. Where the current term carries "
        "the gravity torque that the model is meant to cancel, mis-scaling it "
        "destroys the cancellation; where the model contributes little, as at the "
        "wrists, the coefficient acts as a scale factor only. The assumed "
        "coefficients therefore affect the physical interpretation of a fault "
        "amplitude in newton metres rather than the detection performance."
    )

    b.h2("3.4. Hybrid Residual Decomposition")
    b.p(
        "The inverse dynamics model packages the Newton–Euler formulation of the "
        "UR10e in the Functional Mock-up Interface format (Blochwitz et al., 2011):"
    )
    b.equation("τ̂_model  =  M(q)·q̈ + C(q, q̇)·q̇ + g(q)")
    b.p("The total residual is the difference between measurement and model,")
    b.equation("r_total  =  τ_meas − τ̂_model")
    b.p(
        "the extrinsic residual is the projection of the end-effector wrench into "
        "joint space,"
    )
    b.equation("r_ext  =  J(q)ᵀ · F_FTS")
    b.p("and the intrinsic residual is the difference of the two,")
    b.equation("r_int  =  r_total − r_ext")
    b.p(
        "where M(q) is the inertia matrix, C(q, q̇) the Coriolis and centrifugal "
        "matrix, g(q) the gravity vector, J(q) the geometric Jacobian, τ_meas the "
        "measured joint torque and F_FTS the measured wrench. This decomposition "
        "makes r_int sensitive to internal faults such as motor degradation or an "
        "encoder glitch, and r_ext sensitive to external interaction such as a "
        "collision or a payload change. Because Equation (4) is a definition, the "
        "numerical consistency of the decomposition can be tested directly: over "
        "200,000 samples the identity holds to a residue of 7.1·10⁻¹⁵, and the "
        "analytical Jacobian agrees with a numerical one to machine precision."
    )
    b.p(
        "Equation (3) requires an agreement of reference frames whose violation "
        "produces no diagnostic. "
        "The UR e-Series driver rotates the wrench into the tool frame and publishes "
        "it with the tool frame identifier, whereas J(q) is defined in the base "
        "frame. Multiplying the two directly produces a wrong residual with no error "
        "message. Measured over 40,000 samples, the correlation between the correct "
        "and the unrotated extrinsic residual is −0.995 on the first joint — the "
        "sign inverts — and −0.143 across all channels; on the second joint the "
        "amplitude triples. Both force and moment halves are therefore rotated into "
        "the base frame before the Jacobian transfer, in the offline pipeline and in "
        "the online engine through the same code path."
    )
    b.p(
        "One property of the solver was measured and deliberately not corrected. "
        "Over 500 random joint poses the inertia matrix is asymmetric in every pose "
        "and its symmetric part is not positive definite, the friction vector is "
        "identically zero and no payload model is present. The solver had been "
        "validated against the physical robot and was treated as fixed, so its cost "
        "is reported instead of repaired: on wrist_2 and wrist_3 the standard "
        "deviation of the model torque is only 10 % and 2 % of the measured torque "
        "(0.095 and 0.030 Nm against 0.959 and 1.246 Nm). In those two channels the "
        "residual is effectively the raw measurement and carries no model "
        "information."
    )

    b.h2("3.5. Dual LSTM Autoencoders")
    b.p(
        "Both models share a symmetric encoder–decoder architecture that compresses "
        "a sliding window of T = 100 samples (0.2 s) into a latent vector. A "
        "two-layer LSTM encoder reduces the input sequence to a fixed-size latent "
        "vector; the decoder repeats it over T steps and reconstructs it with "
        "symmetric LSTM layers. Dropout of 15 % is applied between layers. Both "
        "models are trained on normal operating data only, with a mean squared error "
        "loss, and the anomaly score of a window is its reconstruction error. Each "
        "model's own threshold is the 97th percentile of the reconstruction errors "
        "on the validation set."
    )
    b.p(
        "Windowing uses a stride of 25 samples, that is 75 % overlap. Two rules are "
        "enforced jointly: no window may span two runs, and windows containing "
        "samples invalidated by the derivative edge margin are dropped. The "
        "train/validation split is made at 80 %, but the cut is aligned to the "
        "nearest run boundary. Without that alignment, overlapping windows of the "
        "same run fall into both sets and the validation loss is systematically "
        "optimistic. Table 3 reports the architectures and the training outcome."
    )
    b.table(
        "Model Architectures And Training Outcome On The Corrected Pipeline.",
        ["", "Residual AE", "Raw AE"],
        [["Input channels", "12", "24"],
         ["Hidden / latent", "128 / 32", "256 / 64"],
         ["Parameters", "478,892", "1,907,032"],
         ["Train / validation windows", "8,294 / 2,128", "8,294 / 2,128"],
         ["Best epoch (early stop)", "21  (46)", "22  (47)"],
         ["Best validation loss", "0.582", "0.930"],
         ["Threshold θ (validation P97)", "1.6008", "3.4461"]],
        widths=[3.75, 2.5, 2.3], align_right=[1, 2],
        note="Optimiser Adam (lr = 10⁻³, β = 0.9/0.999), batch size 256, gradient "
             "clipping at 1.0, ReduceLROnPlateau (factor 0.5, patience 8), early "
             "stopping with patience 25 over at most 300 epochs; both models were "
             "trained once, with a fixed seed, in under five minutes each on the "
             "RTX 4060 laptop. The thresholds exceed those of the reference study "
             "(0.420 and 0.854) because the residuals are now on a true "
             "newton-metre scale.")

    b.h2("3.6. Score-Level Fusion And Causal Normalisation")
    b.p(
        "The normalised scores of the two models are combined by a weighted average,"
    )
    b.equation("S_fused  =  w_res · z_res  +  w_raw · z_raw")
    b.p(
        "with w_res + w_raw = 1 and the weight w_res swept over [0, 1] in steps of "
        "0.05. The reference "
        "study normalises each score by a min–max transform whose bounds come from "
        "the fault-injected test set. That is not usable online for two reasons: the "
        "bounds contain future maxima, and those maxima are never seen in normal "
        "operation. Measured consequence: because the test-set maxima are 4341 and "
        "7752, the entire normal operating range compresses to the order of 10⁻⁴, "
        "the fused threshold becomes stricter than either single threshold, and "
        "motor drift and encoder glitches are never detected at all."
    )
    b.p(
        "Both normalisations are affine, so the online engine implements a single "
        "code path,"
    )
    b.equation("z  =  (S − lo) / span")
    b.p(
        "and two causal candidates for (lo, span) were measured against each other: "
        "the scale z = S/θ, and the reference study's own min–max formula with the "
        "bounds taken from clean validation windows rather than from the test set. "
        "The two are practically equal (F1 0.791 against 0.792) and the second was "
        "chosen, because it keeps the published formula and reduces the deviation "
        "from the reference method to a change of bounding set only. The fused "
        "operating threshold is then obtained by extending the study's own rule to "
        "the fused score: it is the 97th percentile of the fused score on clean "
        "validation windows, which on the offline data gives θ = 0.6436 and the "
        "expected 3 % false-alarm rate."
    )

    b.h2("3.7. Fault Injection And Evaluation Protocol")
    b.p(
        "Ground-truth labels do not exist, so four synthetic fault scenarios are "
        "injected in measurement space, listed in Table 4. Joint 3 (elbow) and joint "
        "5 (wrist 2) are chosen because they carry the largest gravity load and the "
        "widest motion range. A window is labelled anomalous when at least 10 % of "
        "its samples overlap the fault mask."
    )
    b.table(
        "Synthetic Fault Scenarios.",
        ["Scenario", "Affected channel", "Amplitude", "Extent"],
        [["Motor drift", "Joint 3 torque, linear ramp", "15 Nm", "30–38 %"],
         ["Collision", "All FTS channels, Gaussian pulse", "30 N",
          "centre 65 %"],
         ["Encoder glitch", "Joint 5 position, step", "1.5 rad", "from 92 %"],
         ["Sensor noise", "All FTS channels, Gaussian noise", "3.5 N",
          "15–23 %"]],
        widths=[2.05, 3.55, 1.5, 1.45],
        note="Sensor noise is deliberately kept at a low amplitude so that it "
             "affects the raw signal only; it is the scenario that tests "
             "complementarity. In the corrected pipeline the amplitudes are "
             "converted into measurement space through the calibration file, "
             "because the channels are genuinely in newton metres.")

    b.h2("3.8. Reproducing The Reference Pipeline")
    b.p(
        "Taken separately the audit findings are defects; taken together they form a "
        "consistent pattern, and that pattern is testable. The hypothesis is that "
        "the previously published numbers come from a pipeline with mixed units: "
        "τ_meas in amperes, τ̂_model in newton metres, r_ext in newton metres and "
        "computed with an unrotated wrench, the three subtracted directly, and fault "
        "amplitudes specified in newton metres added as bare numbers to ampere "
        "channels. A further finding compounds it: the residual generator zeroes the "
        "model torque on joints whose fit quality falls below a threshold, and under "
        "mixed units that guard fires on all six joints, silently reducing the "
        "residual to the measurement itself without any message."
    )
    b.p(
        "The hypothesis makes a falsifiable prediction: rebuilt in that "
        "configuration, the signature findings of the reference study should return "
        "— specifically the blindness of the residual model to sensor noise and the "
        "roughly one-quarter share of anomalies detected by the residual model "
        "alone. The reproduction is scripted and was rerun for this paper; its "
        "result is reported in Section 4.5."
    )

    b.h2("3.9. Online Implementation")
    b.p(
        "The online system differs structurally from the offline pipeline in three "
        "ways, and its design is entirely the management of those three differences. "
        "Causality: nothing computed over a whole dataset may be used live. Time "
        "budget: at 500 Hz there are 2 ms per sample and, with a stride of 25, 50 ms "
        "per decision. Continuity: the offline pipeline may segment the data into "
        "runs, whereas the live system notices an interruption only after it has "
        "happened and must reset its state. Figure 2 shows the resulting data path."
    )
    b.figure(
        "fig2_architecture.png",
        "Data Path Of The Online Detector | every stage between the robot and "
        "the alarm runs once per decision, twenty times a second.")
    b.p(
        "The feature engine is the sample-by-sample equivalent of the offline "
        "computation. It keeps a 51-sample ring buffer and, for every new sample, "
        "emits the features of the sample at the centre of the buffer: the "
        "Savitzky–Golay derivative applied as an inner product with precomputed "
        "coefficients, the current-to-torque conversion, the inverse dynamics "
        "evaluation, the base-frame rotation and the Jacobian transfer. The centred "
        "derivative costs a structural delay of 25 samples (50 ms), which is the "
        "price of a noise-free acceleration and is identical to the offline "
        "definition."
    )
    b.p(
        "Equivalence was verified rather than assumed. The prepared dataset was "
        "pushed through the online engine sample by sample and compared with the "
        "offline feature file: over 8 runs and 61,524 samples the largest absolute "
        "differences are 1.4·10⁻¹⁴ for torque, exactly zero for model torque, "
        "8.0·10⁻¹⁵ for the total residual, 2.7·10⁻¹⁵ for the extrinsic and "
        "8.9·10⁻¹⁵ for the intrinsic residual. These are floating-point rounding "
        "levels, so the deployed node sees the features the models were trained on, "
        "not merely similar ones."
    )
    b.p(
        "Two optional rules were added on top of the reference method. An adaptive "
        "rule fires when the fused score exceeds median + k·1.4826·MAD of the last "
        "600 decisions, where 1.4826·MAD is a robust estimate of the standard "
        "deviation (Leys, Ley, Klein, Bernard and Licata, 2013); it exists because "
        "an absolute threshold cannot see a fault that raises the score a "
        "hundredfold inside a run whose baseline sits far below that threshold. The "
        "scale is floored so that a nearly constant baseline does not make the rule "
        "hypersensitive, the baseline is frozen while an alarm lasts, and the freeze "
        "expires after 3 s so that a long alarm is treated as a regime change rather "
        "than as an attack. The second rule requires two consecutive decisions "
        "(100 ms) before an alarm is raised. Both can be switched off with a single "
        "parameter, which restores the reference behaviour exactly."
    )
    b.p(
        "The detector is packaged as a ROS 2 node (Macenski, Foote, Gerkey, "
        "Lalancette and Woodall, 2022) that keeps the whole computation in a "
        "ROS-independent core; the node is a thin wrapper around it, so replay tests "
        "exercise the class the node actually runs. The models are exported to ONNX "
        "and executed with ONNX Runtime, with two threads per session so that a "
        "greedy inference cannot starve the 500 Hz callback loop. Every decision is "
        "written to a comma-separated log and every alarm to a line-buffered "
        "JSON-lines event log; these two files are the raw material from which the "
        "real-cell thresholds of Section 4.7 were re-measured."
    )
    b.p(
        "Deployment surfaced three connection-layer measurements, all of them "
        "silent. The wrench topic advertised in the controller configuration is not "
        "the one the running broadcaster uses, and subscribing to the wrong name "
        "yields no wrench and therefore no score at all. The joint state topic "
        "carries three different name sets from three publishers, of which the UR "
        "data arrives in a seven-element set in scrambled order at about 112 non-UR "
        "messages per second; the joint mapping must therefore be resolved per "
        "message and cached per name set, since resolving it once and applying it "
        "blindly reads five of six channels from the wrong joints. Finally the "
        "measured sample rate is 495 Hz rather than 500 Hz, which the fixed-step "
        "derivative tolerates at the one per cent level."
    )

    b.h2("3.10. Operator Interface")
    b.p(
        "The detector was integrated into the laboratory's existing web dashboard as "
        "a third tab. A collector subscribes to the node's decision, alarm and score "
        "topics, keeps a 60-second ring buffer and pushes it to the browser at 5 Hz. "
        "Each collector runs its own single-threaded executor: in a first version "
        "both collectors used the shared global executor, two threads then raced on "
        "the same wait set, the node disappeared from the graph, the subscriptions "
        "received nothing and no error was printed."
    )
    b.p(
        "Three interface decisions follow directly from measurements rather than "
        "from taste. Alarms are latched on the rising edge and held for five "
        "seconds, because the clearest verified real event of the campaign lasted "
        "0.25 s — five decisions — and level sampling at 5 Hz misses it entirely; in "
        "a controlled test with synthetic 0.25 s alarms, edge capture saw four "
        "events out of four and level sampling only two. The vertical axis is linear "
        "from 0 to 30 and grows in round steps when a peak exceeds it, because "
        "normal operation sits near 1.3 while the threshold is at 18 and a "
        "logarithmic axis visually flattens the region above the threshold, which is "
        "the only region that matters. In the event table the peak score is the "
        "primary column and the entry score is secondary, because reading an entry "
        "value of 8.5 without seeing the peak of 90.55 led to a misjudged event "
        "during commissioning. Operator labels are written atomically to a separate "
        "file that the detector never touches, and they accumulate the retraining "
        "set that Section 5 identifies as the next step."
    )

    # ──────────────────────────────────────────── 4. Findings ────────
    b.h1("4. Findings")

    b.h2("4.1. Offline Performance Of The Corrected Pipeline")
    b.p(
        "The corrected pipeline was evaluated on 41,688 test windows of which 3,623 "
        "(8.7 %) are anomalous, a class balance consistent with the 8.0 % of the "
        "reference study. Table 5 places the two side by side and Figure 3 shows the "
        "corresponding curves."
    )
    b.table(
        "Overall Performance Of The Corrected Pipeline, With The Reference Study "
        "For Comparison.",
        ["Model", "AUC", "ref.", "PR-AUC", "ref.", "Best F1", "ref."],
        [["Residual LSTM AE", "0.812", "0.908", "0.553", "0.695", "0.584", "0.692"],
         ["Raw LSTM AE", "0.837", "0.952", "0.627", "0.761", "0.647", "0.698"],
         ["**Fusion (0.95/0.05)**", "**0.939**", "0.980", "**0.780**", "0.905",
          "**0.800**", "0.859"],
         ["Fusion MAX", "0.930", "0.976", "0.803", "0.889", "0.792", "0.824"],
         ["Residual norm threshold", "0.646", "0.849", "0.383", "0.805", "0.398",
          "0.707"],
         ["Isolation Forest", "0.652", "0.688", "0.130", "0.459", "0.241", "0.547"],
         ["One-Class SVM", "0.734", "0.815", "0.534", "0.816", "0.596", "0.760"]],
        widths=[4.3, 1.5, 1.5, 1.6, 1.5, 1.6, 1.5], wide=True,
        align_right=[1, 2, 3, 4, 5, 6],
        note="The reference study also lists an OR row, with values identical to "
             "its MAX row in all three metrics. OR is a decision-level combination: "
             "it produces a binary outcome, not a ranking, so AUC, PR-AUC and Best "
             "F1 are not defined for it, and the row in fact reports the MAX score. "
             "The row is therefore omitted here; the operating-point behaviour of "
             "the OR rule is reported in Section 4.2 instead, where its recall of "
             "0.757 is meaningful. Best F1 is listed only for comparability with "
             "the reference study: it is the maximum over all thresholds and so "
             "requires the labels it is meant to predict, which is why Section 3.6 "
             "derives a causal operating threshold instead. The baselines are "
             "trained on the residual features.")
    b.figure(
        "fig3_pr_roc.png",
        "Precision–Recall And ROC Curves Of The Corrected Pipeline | 41,688 test "
        "windows, 8.7 % anomalous. Fusion dominates both single models on both "
        "curves; the margin is larger on the precision–recall panel because ROC is "
        "optimistic under class imbalance.")
    b.p(
        "At the causal operating point θ = 0.6436 the fused detector produces 2,731 "
        "true positives, 546 false positives, 892 false negatives and 37,519 true "
        "negatives, that is precision 0.833, recall 0.754, F1 0.792 and specificity "
        "0.986. The false-alarm rate on clean validation data is 3.0 %, as the "
        "97th-percentile rule intends."
    )
    b.p(
        "The absolute values are below those of the reference study, and this is an "
        "expected consequence of the corrections rather than a regression. Three "
        "measurable reasons are given in Section 5.1. The property that both "
        "pipelines share is structural: in each of them the fusion exceeds both "
        "single models and every baseline by a clear margin."
    )

    b.h2("4.2. Fault-Type Asymmetry And Complementarity")
    b.p(
        "Figure 4(b) breaks the result down by fault type and shows the asymmetry "
        "that justifies fusion in the first place. On the encoder glitch the "
        "physical model reflects the joint position deviation as a large residual "
        "and the residual autoencoder dominates (AUC 0.988 against 0.491). On sensor "
        "noise the situation inverts exactly: the low-amplitude wrench noise is "
        "filtered out by the inverse dynamics model, so the residual autoencoder is "
        "blind (0.296) while the raw autoencoder is perfect (1.000). The fused score "
        "follows the stronger of the two in both columns. The two models' blind "
        "spots do not overlap, which is precisely the reason to combine them."
    )
    b.figure(
        "fig4_fusion_value.png",
        "Value Of The Fusion | (a) sweep of the residual weight; the inset "
        "expands the interval in which the contribution of the raw model "
        "disappears. (b) detection by fault type; the two models cover each "
        "other's blind spots.")
    b.p(
        "At window level, with each model using its own 97th-percentile threshold, "
        "24.3 % of the 3,623 anomalous windows are detected by the residual model "
        "only and 28.7 % by the raw model only, against 24.1 % and 25.9 % in the "
        "reference study. The recall of the OR combination is 0.757, well above "
        "either single model (0.470 and 0.514). Table 6 adds the decision-level "
        "ablation at the selected operating point."
    )
    b.table(
        "Complementarity And Decision-Level Ablation On 41,688 Windows.",
        ["Configuration", "Precision", "Recall", "F1"],
        [["Raw only  (w_res = 0.00)", "0.759", "0.514", "0.613"],
         ["Residual only  (w_res = 1.00)", "0.756", "0.470", "0.579"],
         ["**Fusion  (w_res = 0.95)**", "**0.833**", "**0.754**", "**0.792**"]],
        widths=[3.35, 1.85, 1.6, 1.55], align_right=[1, 2, 3],
        note="Removing the five per cent weight of the raw model lowers recall from "
             "0.754 to 0.470. The gain over the better single model is ΔF1 = +0.178; "
             "the reference study reports +0.161 at a different absolute level.")

    b.h2("4.3. Sensitivity To The Fusion Weight")
    b.p(
        "The weight was swept from 0.00 to 1.00, with the fused threshold taken at "
        "every point from clean validation windows only, so that the test set never "
        "sees the threshold. Figure 4(a) shows a broad plateau: between 0.25 and "
        "0.95 the performance is stable, which reproduces the stable region reported "
        "by the reference study. The informative part is the edge. At w_res = 0.95 "
        "the PR-AUC is 0.780; at 0.98 it is 0.718, at 0.99 it is 0.621 and at 1.00 "
        "it collapses to 0.553. A weight of five per cent on the raw model is "
        "therefore associated with a gain of +0.227 PR-AUC relative to the pure "
        "residual detector. This is the strongest numerical evidence that the "
        "fusion is functional rather than nominal, and it accounts for the position "
        "of the selected weight on the shoulder immediately preceding the drop: the "
        "contribution of the raw model is admitted at a small weight but is not "
        "eliminated."
    )
    b.p(
        "Two independent checks confirm that both models really run. Structurally, "
        "every rule of the reference method maps onto the deployed decision path. "
        "Operationally, the scoring method of each ONNX session was wrapped and "
        "counted over a replay of 6 runs and 5 scenarios: 13,160 decisions produced "
        "13,160 residual-model calls and 13,160 raw-model calls, so neither model is "
        "cached or skipped."
    )

    b.h2("4.4. Evaluation On Windows Not Seen In Training")
    b.p(
        "The evaluation protocol inherited from the reference study builds the test "
        "set by injecting each of the four scenarios into every window of the "
        "recording, which is how 41,688 test windows arise from 10,422 windows. The "
        "autoencoders, however, were trained on the first 8,294 of those windows. "
        "Consequently 79.6 % of the evaluation windows are windows the models had "
        "already seen, in their clean form, during training. The protocol is kept "
        "here so that the comparison with the reference study remains valid, but "
        "the split is reported rather than left implicit, because the two halves "
        "behave very differently."
    )
    b.figure(
        "fig5_unseen.png",
        "Effect Of The Train/Test Overlap | PR-AUC on all evaluation windows, on "
        "those the autoencoders were trained on, and on the 8,512 windows they "
        "never saw. On unseen data the two models exchange roles and the fusion "
        "gains nothing over the residual model alone.",
        wide=False)
    b.p(
        "On the 8,512 windows outside the training range the ranking quality "
        "inverts completely. The residual autoencoder rises to PR-AUC 0.995 and "
        "Best F1 0.959, while the raw autoencoder falls to PR-AUC 0.222 and Best F1 "
        "0.336; the fusion reaches 0.995 and 0.961, that is, it matches the "
        "residual model and gains nothing from the raw one. Table 7 gives the "
        "corresponding areas under the ROC curve."
    )
    b.table(
        "Ranking Quality On The Two Halves Of The Evaluation Set (AUC).",
        ["Subset", "Windows", "Residual", "Raw", "Fusion"],
        [["All (as reported)", "41,688", "0.812", "0.837", "0.939"],
         ["Used in training", "33,176", "0.829", "0.866", "0.947"],
         ["**Never seen**", "**8,512**", "**0.999**", "**0.687**", "**0.999**"]],
        widths=[2.35, 1.55, 1.6, 1.25, 1.6], align_right=[1, 2, 3, 4],
        note="The unseen windows are the clean validation set, from which the "
             "normalisation bounds and the 97th-percentile thresholds are also "
             "taken, so they are not an untouched test set either. AUC, PR-AUC and "
             "Best F1 are invariant to the monotone per-model scaling, so the "
             "residual and raw columns are unaffected by that dependence; the "
             "fusion column is not, because the relative scale of the two scores "
             "is fitted on these windows.")
    b.p(
        "Two conclusions follow. First, the margin of +0.178 reported in Table 6 is "
        "measured under the reference protocol and is inflated by the overlap: the "
        "raw autoencoder is strong on the windows it was trained on (PR-AUC 0.705) "
        "and weak outside them (0.222), which is the signature of memorisation "
        "rather than generalisation. The claim that the fusion beats the better "
        "single model therefore holds under the reference protocol but not on the "
        "unseen segment, where the residual model alone is equally good. Second, "
        "the complementarity claim itself survives, and in a sharper form: each "
        "model dominates in a different regime, and the regimes are separated here "
        "not by fault type but by position in the recording."
    )
    b.p(
        "The second conclusion matters beyond the offline evaluation. A shift large "
        "enough to move the raw model's PR-AUC from 0.705 to 0.222 occurs inside a "
        "single recording of one robot performing one task family. It is the same "
        "out-of-distribution effect that dominates the real-cell results in "
        "Section 4.7, observed here two hours of robot operation earlier and on "
        "simulated faults. A run-disjoint retraining, in which no window of an "
        "evaluation run contributes to training, is the proper remedy and is left "
        "as the immediate next step."
    )

    b.h2("4.5. Origin Of The Previously Published Figures")
    b.p(
        "Rebuilt in the mixed-unit configuration of Section 3.8, the pipeline "
        "reproduces the reference study point by point. Table 8 lists the "
        "comparison."
    )
    b.table(
        "Reproduction Of The Reference Pipeline.",
        ["Quantity", "Rebuild", "Reference", "Δ"],
        [["Test windows", "179,888", "179,896", "8"],
         ["Fault windows", "14,401 (8.01 %)", "14,402 (8.0 %)", "1"],
         ["Residual-only share", "23.9 %", "24.1 %", "0.2 pt"],
         ["Sensor noise — residual", "0.267", "0.272", "0.005"],
         ["OR recall", "0.841", "0.854", "0.013"],
         ["Recall — raw", "0.601", "0.613", "0.012"],
         ["Encoder glitch — residual", "0.975", "0.986", "0.011"],
         ["Raw model AUC", "0.943", "0.952", "0.009"],
         ["Fusion MAX — AUC", "0.974", "0.976", "0.002"]],
        widths=[2.95, 1.95, 1.85, 1.60], align_right=[1, 2, 3],
        note="The decisive row is sensor noise. A residual model scoring 0.267 on "
             "that scenario can only arise on a pipeline where the residual has lost "
             "its physical meaning, that is where amperes and newton metres are "
             "subtracted directly. Agreement of the window counts to 8 and 1 windows "
             "in 179,896 confirms that the same raw dataset is being split the same "
             "way.")
    b.p(
        "The conclusion is not that the reference study's claim is wrong. The "
        "complementarity of the two representation spaces is measured on both "
        "pipelines — 23.9 % residual-only on the reproduction and 24.3 % on the "
        "corrected pipeline — and the fusion margin survives on both. What was wrong "
        "is the physical scale of the reported numbers. The corrected pipeline is "
        "therefore the main result of this paper, and the reproduction is an "
        "appendix that documents where the earlier figures came from."
    )

    b.h2("4.6. Latency And Compute Budget")
    b.p(
        "Per sample, the inverse dynamics evaluation costs 22 µs and the Jacobian "
        "about 30 µs, together 1.5 % of the 500 Hz budget. Per decision, the two "
        "ONNX forward passes cost 5.9 ms, that is 12 % of the 50 ms budget, measured "
        "on CPU in offline replay, and 8.5 ms (17 % of the budget) on the cell, "
        "where the ROS 2 executor and the logging layer share the same cores; on "
        "the deployment workstation with the CUDA provider both fall further. The "
        "detection latency is the sum of the 50 ms filter delay and the "
        "50 ms decision period, plus one more decision period when the two-consecutive "
        "rule is active, giving 120–150 ms end to end. At a tool speed of 0.25 m/s "
        "that corresponds to about 3.75 cm of additional travel."
    )
    b.p(
        "An end-to-end replay of the prepared data through the deployed class, over "
        "6 runs and 113,722 samples, gives a false-alarm rate of 0.0 % on clean data "
        "and detects 6 of 6 runs for collision and sensor noise, 5 of 6 for the "
        "encoder glitch and 4 of 6 for motor drift. Median latency is about 70 ms "
        "for the abrupt faults and 1,677 ms for the slow ramp, a split that is "
        "structural: within a 200 ms window a slow ramp is almost a constant offset, "
        "and an autoencoder reconstructs a constant offset without difficulty."
    )

    b.h2("4.7. Commissioning On The Real Robot")
    b.p(
        "All results up to this point were obtained on the recorded dataset. The "
        "system was then commissioned on the physical UR10e cell in a session that "
        "ran from 10:14 to 12:13 on 21 August 2026, just under two hours. The "
        "session covered the two production use cases of the platform — the "
        "autonomous visual inspection cycle and a pick-and-place scenario — "
        "together with dedicated fault-trial runs in which anomalies were provoked "
        "by hand so that the detector could be exercised against events whose "
        "ground truth is known to the operator. Across eight runs the detector "
        "logged 50.4 minutes of decisions, roughly sixty thousand decisions in "
        "total, with every decision written to disk and every alarm block written "
        "to an event log. Table 9 summarises how the online system had to deviate "
        "from the reference method, and how much of that was forced."
    )
    b.table(
        "Deviations Of The Online System From The Reference Method.",
        ["Deviation", "Status", "Reason"],
        [["D1  Normalisation bounds from clean validation instead of the test set",
          "forced", "the published bounds are not causal and compress the live "
          "range to ~10⁻⁴"],
         ["D2  Operating threshold for the fused score",
          "forced", "the reference method reports Best F1 only, which needs the "
          "labels it predicts"],
         ["D3  Adaptive median + k·MAD rule", "optional, **off**",
          "helps offline, catches no verified event on hardware"],
         ["D4  Two consecutive decisions before alarm", "optional, on",
          "suppresses single-decision noise triggers"]],
        widths=[6.3, 2.4, 5.6], wide=True,
        note="D3 and D4 are switched off by a single parameter each, which restores "
             "the reference behaviour exactly. D1 keeps the published min–max "
             "formula and changes only the set the bounds are taken from.")
    b.p(
        "The first measurement invalidated the operating threshold. On a clean run "
        "the median fused score is 4.27 while the threshold derived from the offline "
        "validation set is 0.6436; 97 % of all decisions are therefore declared "
        "alarms. The threshold is seven times too low."
    )
    b.p(
        "The cause was measured, and it is not the threshold. The residual is pose "
        "dependent. Figure 6(b) shows the fused score in four operating regimes of "
        "the same robot with the same models: 0.0037 in the folded park pose, 0.32 "
        "during a protective stop, 1.57 in normal motion and 4.41 in a "
        "gravity-loaded static pose — three orders of magnitude. The current-to-torque "
        "coefficient could be measured on only two of six joints and the calibration "
        "offset is a constant, which cannot compensate a pose-dependent error. A "
        "single global threshold is therefore inherently fragile. The limitation "
        "originates in model fidelity rather than in the choice of threshold."
    )
    b.figure(
        "fig6_real_cell.png",
        "Behaviour On The Physical Cell | (a) fused score over a fault-trial run "
        "with the three operator-confirmed events marked. (b) score distribution by "
        "operating regime; the threshold derived from offline data passes through "
        "the middle of three of the four regimes.")
    b.p(
        "A second offline conclusion inverted. In the first run the node was started "
        "while the robot stood still; the adaptive baseline was learned from motionless "
        "noise and settled at 0.0103, the score jumped to 0.34 as soon as the robot "
        "moved, the alarm latched, and because the baseline freezes while an alarm "
        "lasts it could never update again. The result was a single alarm block of "
        "564 s in a 620 s run — 91 % of the recording — which also hid a genuine "
        "event: a hand caught between links drove the robot into a protective stop "
        "and the fused score to the highest value of the run, 10.27, but no new event "
        "record was produced because the system was already alarming. The 3 s freeze "
        "timeout was added for this reason. On the labelled set the adaptive rule was "
        "then re-swept over k ∈ [8, 30]; no value caught a verified event and every "
        "value added false blocks to clean data, because the rule's baseline absorbs "
        "the peaks of normal motion, the MAD inflates and the adaptive threshold "
        "climbs above the real events. On the physical robot the component that "
        "catches events is the absolute threshold — the exact opposite of the offline "
        "finding."
    )
    b.p(
        "The threshold was therefore re-measured by an exhaustive recount rather "
        "than selected. Every alarm block the detector would have raised was "
        "recounted "
        "directly from the logged decisions of all eight runs, for each candidate "
        "threshold and under the same two-consecutive-decision rule the node "
        "applies; each block was then classified against the five events the "
        "operator had confirmed while standing at the cell. The recount reproduces "
        "the event logs exactly where they overlap — at the threshold of 8.0 that "
        "was active during the inspection run it returns the same sixteen blocks "
        "with the same peaks — so the sweep of Table 10 covers the whole 50.4 "
        "minutes rather than only the runs that happened to be labelled."
    )
    b.table(
        "Threshold Sweep Recounted From All 50.4 Minutes Of Logged Decisions.",
        ["Threshold", "Real caught", "FA", "FA / hour", "Alarm duty"],
        [["0.6436  (offline)", "5 / 5", "335", "398.7", "74.4 %"],
         ["8.0  (first deployed)", "5 / 5", "29", "34.5", "7.1 %"],
         ["12.0", "4 / 5", "25", "29.8", "4.7 %"],
         ["**18.0  (selected)**", "**4 / 5**", "**12**", "**14.3**", "**2.9 %**"],
         ["26.0", "4 / 5", "5", "6.0", "1.8 %"],
         ["33.0", "3 / 5", "1", "1.2", "1.0 %"],
         ["91.0", "2 / 5", "0", "0.0", "0.6 %"]],
        widths=[2.70, 1.55, 1.00, 1.50, 1.60], align_right=[1, 2, 3, 4],
        note="Alarm duty is the share of logged time spent in an alarm state. The "
             "value first deployed, 8.0, was carried over from an initial estimate "
             "and is shown for reference.")
    b.p(
        "The selection is an interval, not a point. Its lower bound is the ceiling "
        "of the autonomous inspection cycle, measured at 17.43: any threshold above "
        "that value silences all sixteen of the cycle's periodic excursions at "
        "once, and any threshold below it lets all sixteen through. Its upper bound "
        "is the weakest operator-confirmed event, 31.98. Within the resulting "
        "interval [17.43, 31.98] the value 18.0 is the conservative end, chosen to "
        "keep the largest possible margin for events weaker than the ones observed; "
        "a purely false-alarm-minimising choice inside the same interval would be "
        "about 26, at the cost of that margin. The single missed event at either "
        "value is the first jam, peak 10.27, which the robot's own protective stop "
        "had already interrupted."
    )
    b.p(
        "The distribution of the residual false alarms over the runs is as "
        "informative as their number. At θ = 18 the six runs of ordinary "
        "operation — 37.4 minutes, covering five complete cycles of the "
        "autonomous inspection task — produce no false alarm; by the rule of "
        "three this bounds the false-alarm rate of that configuration at 4.8 per "
        "hour with 95 % confidence. All twelve alarms occur in the two runs in "
        "which a person interacted physically with the robot or the workpiece: "
        "ten in pick-and-place and two in a fault-trial run. The detector is "
        "therefore silent while the cell operates autonomously and active while a "
        "human is inside the workspace, which is the inverse of the intended "
        "behaviour of a safety function and is attributable to the same cause as "
        "the remaining findings of this section."
    )
    b.p(
        "The remaining false alarms are not stochastic; they are structured and "
        "locked to the trajectory. In the inspection cycle the intervals between "
        "alarm onsets are 60, 38, 59, 32, 60, 32, 59, 32 and 60 seconds, and each "
        "92-second cycle repeats the same triplet: 6.2 s with a peak near 17, then "
        "3.0 s with a peak near 10.3, then 0.15 s near 8.4. The pattern was "
        "identical in all five cycles observed. This determinism has a "
        "methodological consequence: the natural unit of observation for this "
        "false-alarm process is the task cycle rather than elapsed time, since "
        "repeating the same trajectory reproduces the same excursions. It is also "
        "direct evidence that the models do not recognise the real trajectories: "
        "for autoencoders trained on the recorded dataset, the actual inspection "
        "scan is out of distribution."
    )
    b.p(
        "A discriminating feature other than amplitude was sought and not found. "
        "Block sharpness, defined as peak over within-block median, is 1.08–1.99 for "
        "the real events and 1.00–4.83 for the false alarms, so the sharpest block "
        "in the campaign is a false alarm; rise time does not separate the classes "
        "either. The peak distributions overlap genuinely: the confirmed real events "
        "peak at 10.27, 31.98, 79.86, 146.44 and 195.83, while the confirmed false "
        "alarms range from 8.28 to 90.55 with a median of 10.42. One false alarm "
        "therefore peaks higher than three of the five real events. That case cannot "
        "be removed by any choice of threshold."
    )
    b.p(
        "Detection latency on the physical cell is the sum of the 50 ms filter delay, "
        "the 50 ms decision period at 495 Hz and one further decision period for the "
        "two-consecutive rule, about 150 ms in total, with an inference time of "
        "8.5 ms as reported in Section 4.6. The clearest verified event — the collision at "
        "the end of the pick-and-place scenario, peak 146.44 — lasted only 0.25 s, "
        "which forces every consumer of the alarm to treat it as an edge rather than "
        "as a level."
    )

    b.h2("4.8. Operator Interface")
    b.p(
        "Figure 7 shows the interface connected to the running robot. The status "
        "banner reads NORMAL, the decision rate is 20.0 Hz and the operating "
        "threshold is 18.00. The peak of about 14 visible in the live chart stayed "
        "below the threshold and therefore produced no alarm. The breakdown panel "
        "shows that the raw model was driving the decision at that moment (raw "
        "5.60 / 3.45 = 1.62 times its own threshold, residual 1.57 / 1.60 = 0.98 "
        "times). The event table below lists the pick-and-place run, with the "
        "collision of peak 146.44 in the top row."
    )
    b.figure(
        "fig7_interface.png",
        "Operator Interface Connected To The Physical UR10e | the labels recorded "
        "in the event table form the retraining set identified in Section 5.")

    # ────────────────────────────────────────── 5. Discussion ────────
    b.h1("5. Discussion")

    b.h2("5.1. Why The Absolute Values Fell")
    b.p(
        "The corrected pipeline scores below the reference study on every absolute "
        "metric, for three measurable reasons. First, fault amplitudes are now on "
        "the true physical scale. On the mixed-unit pipeline a ramp specified as "
        "15 Nm was added as a bare number to a channel expressed in amperes, which "
        "is a perturbation roughly ten times larger than its physical counterpart; "
        "the corrected pipeline injects a genuine 15 Nm, so the problem is harder. "
        "Second, the dataset was cleared of discontinuities, and those "
        "discontinuities did not merely add noise — they created artificial jumps "
        "that a reconstruction model reads as anomalous, so removing them shrinks "
        "the apparent distance between normal and anomalous to its true value. "
        "Third, the evaluation is now leakage-free, with the split aligned to run "
        "boundaries and windows respecting them."
    )
    b.p(
        "What should be compared between the two pipelines is therefore not the "
        "absolute number but the structure, and the structure is preserved in every "
        "respect: the two models complement each other in the same proportions, the "
        "weight sweep is broad and stable in the same region, the fusion margin over "
        "the better single model is of the same order (+0.178 against +0.161), and "
        "the fault-type asymmetry that motivates the whole design is reproduced "
        "almost exactly. Both pipelines, however, share the evaluation protocol "
        "audited in Section 4.4, so both margins are measured largely on windows "
        "the models were trained on, and neither should be read as a "
        "generalisation estimate."
    )

    b.h2("5.2. Why Motor Drift Stays Hard")
    b.p(
        "Motor drift is the weakest scenario for both single models and for the "
        "fusion (AUC 0.443; the reference study reports 0.588 and also records a "
        "marginal loss from fusion in this scenario). This is a structural limit of "
        "a windowed autoencoder rather than an implementation defect: inside a "
        "100-sample window a slow linear ramp is almost a constant offset, and a "
        "constant offset is reconstructed without difficulty. Online, the adaptive "
        "rule partly covers this gap because its baseline follows the last 30 "
        "seconds, so a slowly growing score can raise an alarm without ever crossing "
        "the absolute threshold; in replay, motor drift was caught in 4 of 6 runs "
        "with a median delay of 1,677 ms. The correct long-term answer is not to "
        "force a windowed autoencoder but to add a second, long-horizon indicator "
        "such as the slope of a moving-average residual."
    )

    b.h2("5.3. What The Transfer To Hardware Changed")
    b.p(
        "Two conclusions supported by the offline data were inverted by the physical "
        "cell, and this is the central finding of the paper. The operating threshold "
        "had to be raised by a factor of seven, because the residual is pose "
        "dependent while the threshold is a single global constant. The adaptive "
        "alarm rule — the component that rescued the single models offline, and "
        "whose justification came from the reference method's own blind spot "
        "argument — caught no verified event on hardware and only added false blocks; "
        "it is disabled by default."
    )
    b.p(
        "Both inversions have the same root. The autoencoders were trained on a "
        "recorded dataset that does not contain the real inspection trajectories, so "
        "on hardware those trajectories are out of distribution. The evidence is "
        "direct rather than inferential: the false alarms repeat with the cycle of "
        "the trajectory, to the second, and the same motion produces the same "
        "excursion every time. Under that condition no threshold rule can succeed, "
        "because the score is not measuring abnormality but unfamiliarity. This is "
        "the anomaly-detection counterpart of the reality gap described for control "
        "policies by Zhao et al. (2020), and it suggests that reporting a detection "
        "threshold without stating the operating regime it was measured in is of "
        "limited value."
    )
    b.p(
        "The offline result of Section 4.4 is the same phenomenon at a smaller "
        "scale, and it is what makes the hardware outcome predictable in "
        "hindsight. Moving from the first four fifths of one recording to the last "
        "fifth already moves the raw model's PR-AUC from 0.705 to 0.222 while the "
        "residual model rises from 0.486 to 0.995. If a segment boundary inside a "
        "single recording can do that, a change of task profile on real hardware "
        "certainly can. The two models are not equally exposed: the residual model "
        "sees a physically normalised quantity and degrades gracefully, whereas the "
        "raw model sees the signals themselves and degrades sharply. That "
        "asymmetry, rather than the size of the fusion margin, is the transferable "
        "finding of this study."
    )
    b.p(
        "The practical consequence is a design rule. An operating threshold must be "
        "measured on the deployment hardware, over at least one full task cycle, and "
        "it must be revisited whenever the task profile changes. The system stores "
        "the offline value alongside the deployed one and records the justification "
        "for the deployed one, so that the difference between the two remains "
        "visible rather than being quietly overwritten."
    )

    b.h2("5.4. Effect Of The Measured Solver Limitations")
    b.p(
        "The solver contains no friction model and its inertia matrix fails the "
        "symmetry and positive-definiteness tests, so the theoretical advantage of "
        "the residual space is not realised at the wrist joints, where the model "
        "torque is a small fraction of the measured torque. The lower single-model "
        "performance of the residual autoencoder relative to the reference study "
        "(AUC 0.812 against 0.908) is consistent with this effect. The survival of "
        "the fusion margin under this limitation is itself diagnostic: the raw "
        "model carries the information directly in exactly those channels, which is "
        "the intended function of the fusion."
    )

    b.h2("5.5. Limitations")
    for item in (
        "A single robot (UR10e) and a single task profile; no validation on other "
        "robot types has been performed.",
        "Synthetic faults may not reflect the full dynamics of real faults; in "
        "particular the collision pulse is distributed equally over the wrench "
        "channels instead of being propagated from a real contact point through the "
        "Jacobian.",
        "The current-to-torque coefficient could be measured directly on only two "
        "of six joints; the other four are derived by assumption, which affects the "
        "physical interpretation rather than the detection.",
        "Four fifths of the offline evaluation windows are also training windows, "
        "because the evaluation protocol of the reference study is retained for "
        "comparability. Section 4.4 reports the split separately, but a "
        "run-disjoint retraining is required before the offline margins can be "
        "read as generalisation.",
        "The operating threshold is selected on the same commissioning campaign on "
        "which its false-alarm rate is reported, and the campaign yields only five "
        "operator-confirmed events. The threshold should be validated on a "
        "campaign it was not fitted to.",
        "The observation covers five complete cycles of the inspection task. "
        "Because the false-alarm process is deterministic and cycle-locked, "
        "additional repetitions of the same trajectory would add little "
        "information about it; what a longer campaign would add is the variation "
        "the two-hour window cannot contain, namely thermal drift of the drives, "
        "part-to-part and payload variation, mechanical wear and operator "
        "differences. The results reported here are therefore a commissioning-scale "
        "proof of concept with respect to those factors.",
        "Online measurements were taken on CPU; they should be repeated with the "
        "GPU provider on the deployment workstation.",
    ):
        p = b.para("", style="Paragraf", align=WD_ALIGN_PARAGRAPH.JUSTIFY,
                   space_before=3, space_after=0)
        p.paragraph_format.left_indent = Cm(0.4)
        b.rich(p, "•  " + item)

    # ──────────────────────────────────────── 6. Conclusions ─────────
    b.h1("6. Conclusions")
    b.p(
        "This paper followed a score-level fusion framework for cobot anomaly "
        "detection from a published offline result all the way to a detector running "
        "on a physical UR10e cell, and reported every measurement taken on the way, "
        "including the ones that were unfavourable."
    )
    b.p(
        "The method of the reference study was rebuilt independently and its "
        "architectural and procedural components were shown to match exactly. Seven "
        "findings were measured in the data path; five were corrected, one — the "
        "measured limitations of the inverse dynamics solver — was deliberately "
        "accepted under a fixed constraint, and one is a typesetting inconsistency. "
        "The previously published figures were traced to a mixed-unit pipeline, and "
        "that pipeline was reproduced explicitly so that the origin of the numbers "
        "is documented rather than merely asserted."
    )
    b.p(
        "On the corrected pipeline, and under the reference study's own evaluation "
        "protocol, the value of the fusion is preserved: F1 0.792 against 0.613 for "
        "the raw model and 0.579 for the residual model, a gain of +0.178 over the "
        "better single model, with complementarity ratios matching the reference "
        "study. Removing the five per cent raw weight collapses PR-AUC from 0.780 "
        "to 0.553, which shows that the fusion is functional rather than cosmetic."
    )
    b.p(
        "That protocol was also audited. Four fifths of its evaluation windows are "
        "windows the autoencoders were trained on, and on the remaining fifth the "
        "two models exchange roles: the residual model reaches PR-AUC 0.995 while "
        "the raw model falls to 0.222, and the fusion gains nothing over the "
        "residual model alone. The complementarity claim survives this test in a "
        "sharper form — each model dominates a different regime — but the size of "
        "the fusion margin does not, and is reported here as protocol-dependent "
        "rather than as a generalisation estimate."
    )
    b.p(
        "The system was implemented as a ROS 2 node running at 500 Hz whose feature "
        "engine is numerically identical to the offline pipeline to floating-point "
        "rounding, in which both models run on every decision, and whose inference "
        "consumes 12 % of the decision budget."
    )
    b.p(
        "The measurements on the physical robot inverted two offline conclusions: "
        "the operating threshold had to be raised sevenfold, and the adaptive alarm "
        "rule had to be disabled. Both follow from the same root cause, which was "
        "measured and not assumed — the real trajectories are out of distribution "
        "for models trained on the recorded dataset. The replacement threshold is "
        "not a free parameter: recounting every alarm block over the full 50.4 "
        "minutes of logged decisions from a two-hour commissioning session bounds "
        "it from below by the ceiling of the autonomous inspection cycle (17.43) "
        "and from above by the weakest confirmed event (31.98), and at the selected "
        "value of 18.0 the detector is silent through 37.4 minutes of autonomous "
        "operation. The remaining limit cannot be removed by any threshold, since "
        "one confirmed false alarm peaks higher than three of the five confirmed "
        "real events."
    )
    b.p(
        "The next step follows directly from that limit and is already instrumented: "
        "the operator interface accumulates verified labels, and those labels form "
        "the set on which the two autoencoders should be retrained on real robot "
        "data, under a run-disjoint split in which no window of an evaluation run "
        "contributes to training. Two further directions are a long-horizon "
        "indicator to cover the slow drift scenario, and a pose-conditioned rather "
        "than global threshold, since the score was measured to vary over three "
        "orders of magnitude with the pose of the arm."
    )

    # ─────────────────────────────────────────── back matter ─────────
    b.h1("Acknowledgement")
    b.p(
        "The project is supported by the KDT Joint Undertaking (101140216) and its "
        "members, including additional funding from Vinnova (Sweden), "
        "Österreichische Forschungsförderungsgesellschaft mbH – FFG (Austria), "
        "Business Finland (Finland), Ministry of Universities and Research (Italy), "
        "FCT (Portugal) and TÜBİTAK (124N448) (Türkiye). The measurements were "
        "carried out at the Autonomous Systems and Reliability Laboratory of the "
        "ESOGÜ Intelligent Systems Application and Research Centre."
    )

    b.h1("Contribution Of Researchers")
    b.p(
        "Author 1: design and implementation of the offline pipeline, the audit and "
        "the ROS 2 detector node, commissioning measurements, preparation of the "
        "manuscript. Author 2: … . Author 3: … . (To be completed by the authors.)"
    )

    b.h1("Conflict Of Interest")
    b.p("No conflict of interest has been declared by the authors.")

    b.h1("References")
    for ref in REFERENCES:
        p = b.para("", style="Kaynaklar", align=WD_ALIGN_PARAGRAPH.JUSTIFY,
                   space_before=6, space_after=0)
        p.paragraph_format.left_indent = Cm(0.5)
        p.paragraph_format.first_line_indent = Cm(-0.5)
        b.rich(p, ref, size=BODY_PT)


REFERENCES = [
    "Blochwitz, T., Otter, M., Arnold, M., Bausch, C., Clauß, C., Elmqvist, H., "
    "… Wolf, S. (2011). The Functional Mockup Interface for tool independent "
    "exchange of simulation models. *Proceedings of the 8th International Modelica "
    "Conference*, 105–114, Dresden, Germany.",

    "Correia, L., Goos, J. C., Klein, P., Bäck, T. & Kononova, A. V. (2024). "
    "Online model-based anomaly detection in multivariate time series: Taxonomy, "
    "survey, research challenges and future directions. *Engineering Applications "
    "of Artificial Intelligence, 138*, 109323.",

    "Darban, Z. Z., Webb, G. I., Pan, S., Aggarwal, C. C. & Salehi, M. (2024). "
    "Deep learning for time series anomaly detection: A survey. *ACM Computing "
    "Surveys, 57*(1), 1–42.",

    "Haddadin, S., De Luca, A. & Albu-Schäffer, A. (2017). Robot collisions: A "
    "survey on detection, isolation, and identification. *IEEE Transactions on "
    "Robotics, 33*(6), 1292–1312.",

    "Huang, X., Chen, N., Deng, Z. & Huang, S. (2024). Multivariate time series "
    "anomaly detection via dynamic graph attention network and Informer. *Applied "
    "Intelligence, 54*, 7636–7658.",

    "Katsampiris-Salgado, K., Dimitropoulos, N., Gkrizis, C., Michalos, G. & "
    "Makris, S. (2024). Collision detection for collaborative assembly operations "
    "on high-payload robots. *Robotics and Computer-Integrated Manufacturing, 87*, "
    "102708.",

    "Leys, C., Ley, C., Klein, O., Bernard, P. & Licata, L. (2013). Detecting "
    "outliers: Do not use standard deviation around the mean, use absolute "
    "deviation around the median. *Journal of Experimental Social Psychology, "
    "49*(4), 764–766.",

    "Li, W., Han, Y. & Xiong, Z. (2020). Collision detection of robots based on a "
    "force/torque sensor at the bedplate. *IEEE Transactions on Industrial "
    "Electronics, 67*(12), 12440–12449.",

    "Liu, K., Wang, L., Zhang, X., Sun, Y. & Li, J. (2025). Anomaly detection in "
    "multidimensional time series for water injection pump operations based on "
    "LSTMA-AE and mechanism constraints. *Scientific Reports, 15*.",

    "Macenski, S., Foote, T., Gerkey, B., Lalancette, C. & Woodall, W. (2022). "
    "Robot Operating System 2: Design, architecture, and uses in the wild. "
    "*Science Robotics, 7*(66), eabm6074.",

    "Malhotra, P., Ramakrishnan, A., Anand, G., Vig, L., Agarwal, P. & Shroff, G. "
    "(2016). LSTM-based encoder-decoder for multi-sensor anomaly detection. "
    "*arXiv preprint arXiv:1607.00148*.",

    "Malhotra, P., Vig, L., Shroff, G. & Agarwal, P. (2015). Long short term "
    "memory networks for anomaly detection in time series. *Proceedings of the "
    "European Symposium on Artificial Neural Networks (ESANN)*, 89–94, Bruges, "
    "Belgium.",

    "Park, D., Hoshi, Y. & Kemp, C. C. (2018). A multimodal anomaly detector for "
    "robot-assisted feeding using an LSTM-based variational autoencoder. *IEEE "
    "Robotics and Automation Letters, 3*(3), 1544–1551.",

    "Savitzky, A. & Golay, M. J. E. (1964). Smoothing and differentiation of data "
    "by simplified least squares procedures. *Analytical Chemistry, 36*(8), "
    "1627–1639.",

    "Yılmaz, C. S., Kahraman, S., Yılmaz, M., Yavuz, H. S. & Yayan, U. (2026). "
    "FMU tabanlı kalıntı ayrıştırma ve ikili LSTM özkodlayıcı birleşimi ile "
    "işbirlikçi robotlarda anomali tespiti [FMU-based residual decomposition and "
    "dual LSTM autoencoder fusion for anomaly detection in collaborative robots]. "
    "⟨KONFERANS ADI VE YERİ — yazarlar tarafından tamamlanacaktır⟩ kurultayında "
    "sunulmuş bildiri, Türkiye.",

    "Zhang, T., Chen, Y. & Zou, Y. (2024). Robot collision detection based on "
    "external torque observer. *Journal of South China University of Technology, "
    "52*(3), 84–92.",

    "Zhao, W., Queralta, J. P. & Westerlund, T. (2020). Sim-to-real transfer in "
    "deep reinforcement learning for robotics: A survey. *2020 IEEE Symposium "
    "Series on Computational Intelligence (SSCI)*, 737–744, Canberra, Australia.",
]


def main():
    doc, anchor = open_template()
    b = Builder(doc, anchor)
    front_matter(b)
    body(b)
    b._leave_wide()
    doc.save(str(OUT))
    print(f"written: {OUT}")
    print(f"  figures: {b.fig_no}   tables: {b.tab_no}   equations: {b.eq_no}")


if __name__ == "__main__":
    main()
