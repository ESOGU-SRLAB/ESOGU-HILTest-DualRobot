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
import os
from pathlib import Path

from docx import Document
from docx.enum.table import WD_TABLE_ALIGNMENT
from docx.enum.text import WD_ALIGN_PARAGRAPH, WD_TAB_ALIGNMENT
from docx.oxml import OxmlElement
from docx.oxml.ns import qn
from docx.shared import Cm, Pt, RGBColor

HERE = Path(__file__).resolve().parent
PKG = HERE.parent


def find_backup():
    """`backup_anomaly_detection` dizinini bul.

    Yedek 28.08.2026'da paketin dışına alındı; paketin çalışma zamanı ona
    bağlı DEĞİL, ama makale üreteçleri bağlı (dergi şablonu, çevrimdışı
    skorlar, bildiri PDF'i). Konumu sabitlemek yerine aranıyor; taşınırsa
    `AD_BACKUP` ile gösterilebilir.
    """
    env = os.environ.get("AD_BACKUP")
    adaylar = ([Path(env).expanduser()] if env else []) + [
        PKG / "backup_anomaly_detection",              # eski yer (paket içi)
        Path.home() / "Desktop" / "backup_anomaly_detection",
        Path.home() / "backup_anomaly_detection",
        PKG.parent / "backup_anomaly_detection",       # colcon_ws/src/ altında
    ]
    for c in adaylar:
        if (c / "Dosya.docx").is_file():
            return c
    raise SystemExit(
        "backup_anomaly_detection bulunamadı. Bakılan yerler:\n  "
        + "\n  ".join(str(c) for c in adaylar)
        + "\nDoğru yeri AD_BACKUP ile verin, ör.:\n"
          "  AD_BACKUP=~/Desktop/backup_anomaly_detection python3 makale_uret.py")


BACKUP = find_backup()
TEMPLATE = BACKUP / "Dosya.docx"
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
    # Şekil ve tablo başlıklarının dil etiketi. Türkçe sürüm bunları
    # "Şekil"/"Tablo" olarak geçersiz kılar; böylece yerleşim kodu tek yerde
    # kalır ve iki sürüm arasında biçim farkı oluşmaz.
    FIG_LABEL = "Figure"
    TAB_LABEL = "Table"

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
    def equation(self, text, label=None):
        """
        `label` verilirse numara sayaçtan ALINMAZ ve sayaç ilerlemez.

        Ana denklemler arasına ek bir denklem sokmak gerektiğinde (ör. kalıntı
        tanımının sürtünmeli hâli) "2a"/"2b" gibi bir etiket kullanılır; aksi
        hâlde sonraki bütün denklem numaraları kayar ve metindeki her atıf
        sessizce yanlışa döner.
        """
        if label is None:
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
        style_run(p.add_run(f"({label or self.eq_no})"), BODY_PT)
        self.para("", style="Normal", space_before=0, space_after=0)
        return label or self.eq_no

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
        self.rich(cap, f"**{self.FIG_LABEL} {self.fig_no}.** {name.strip()}",
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
        self.rich(cap, f"**{self.TAB_LABEL} {self.tab_no}.** {caption}",
                  size=SMALL_PT)

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
    "fusion of a physics-informed residual autoencoder and a data-driven raw "
    "signal autoencoder for a UR10e cobot and, in a first hardware extension, "
    "showed that the fusion margin measured offline is a property of how faults "
    "are injected rather than of the two representation spaces: it collapses "
    "from +0.189 to -0.003 PR-AUC once faults are injected physically instead "
    "of by hand. This paper re-examines the residual definition itself and "
    "finds that the force/torque channel both prior studies relied on is not a "
    "physical measurement: the ROS 2 driver populates it from the controller's "
    "own payload-compensated force estimate, not from a transducer, and the raw "
    "strain-gauge field it should come from reads a constant, implausible "
    "offset. Removing this channel collapses the twelve-channel intrinsic/"
    "extrinsic split to a single six-channel total residual and removes six "
    "channels from the raw model. In its place, the missing payload term is "
    "fitted outside the validated solver, per production task, alongside the "
    "friction term; on four production tasks and 5.16 million prepared samples "
    "it lowers the held-out residual spread by 42-48 % where a single global "
    "offset generalised backwards (a positive bias sign-flipped between "
    "training and unseen sessions of the same task). Under physically "
    "consistent fault injection, the fusion weight is now measured rather than "
    "assumed a priori and converges to a narrow high-residual optimum "
    "(0.90-1.00 across three normalisations), an order narrower than the broad "
    "plateau reported before; in aggregate the raw model no longer helps "
    "(Best F1 0.609 fused against 0.611 for the residual model alone), and its "
    "one measured advantage is confined to a fault type the residual model is "
    "physically blind to: a 0.3 rad encoder step moves the gravity torque by "
    "under 0.1 Nm, below the residual noise floor. The detector was "
    "re-implemented as a 500 Hz ROS 2 node without the removed channel; its "
    "measurement-level equivalence with the offline pipeline was re-verified "
    "and, on a replay of the physical cell, it drew its first alarm from both "
    "models simultaneously. Hardware commissioning and threshold "
    "recalibration on the physical cell, the step that closed the previous "
    "extension, is in progress and is reported separately."
)

ABSTRACT_TR = (
    "İşbirlikçi robotlarda anomalilerin erken tespiti operatör güvenliği ve "
    "üretim sürekliliği açısından kritiktir. Önceki çalışmada, bir UR10e kobot "
    "için fizik tabanlı kalıntı özkodlayıcısı ile veri güdümlü ham sinyal "
    "özkodlayıcısının skor düzeyinde birleşimi sunulmuş ve yalnızca çevrimdışı "
    "değerlendirilmişti. Bu makale o çerçeveyi başlangıç noktası alarak dört "
    "yönde genişletmekte ve gerçek hücrede çalışan bir dedektör olarak teslim "
    "etmektedir. Kalıntı tanımı, doğrulanmış ters dinamik çözücünün içermediği "
    "sürtünme terimiyle tamamlanmış, bilek eklemlerinde kalıntı yayılımı %87 ve "
    "%92 azalmıştır. Değerlendirme koşu-ayrık bölme ve beş eğitim tohumu "
    "üzerine oturtulmuş, bu koşulda birleşim iki tekil modeli de geçmiştir (F1 "
    "0,791'e karşılık 0,627 ve 0,614). Arızaların yalnız ölçülen kanallara "
    "uygulandığı ve kalıntının yeniden hesaplandığı fiziksel olarak tutarlı bir "
    "enjeksiyon protokolü ise birleşim marjını her tohumda +0,189'dan -0,003 "
    "PR-AUC'ye indirmektedir; tamamlayıcılığın kaynağı, fiziksel yayılımından "
    "elli bir kat küçük seçilmiş tek bir enjeksiyon genliğidir. Dedektör, "
    "öznitelik motoru çevrimdışı hatla kayan nokta düzeyinde örtüşen 500 Hz'lik "
    "bir ROS 2 düğümü olarak gerçeklenmiştir. Donanımda çevrimdışı türetilen "
    "eşik taşınmamış; sebebi her iki modelin de eğitim dağılımının çok dışında "
    "çalışması ve %5 ağırlıklı modelin birleşik skorun %29'unu sürüklemesidir. "
    "Hücrede yeniden kalibrasyondan sonra dedektör, operatörce doğrulanmış üç "
    "çarpışmayı yanlış alarm üretmeden yakalamış, düşük genlikli bir temas ise "
    "normal hareketin altında kalmıştır."
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
        "An intermediate stage of this study closed that item on the recorded "
        "dataset's own terms: it added the friction term the validated solver "
        "lacks, placed the evaluation on a run-disjoint footing, and, in its own "
        "principal finding, showed that the fusion advantage the offline study "
        "reported (+0.189 PR-AUC) was an artefact of how faults were injected "
        "and collapses (−0.003) once they are injected physically instead. "
        "Sections below refer to that intermediate stage, unpublished in its "
        "own right, as *the extension this study builds on*. Extending it to "
        "hardware raised the question this paper answers: the intermediate "
        "stage's residual, like the original study's, depended on an "
        "end-effector force/torque channel, and that channel turns out not to "
        "be a physical measurement at all."
    )
    b.p(
        "The gap this paper fills is therefore a specific instance of a general "
        "one. Physics-informed anomaly detection is only as trustworthy as the "
        "measurements its physics model is compared against, and a channel "
        "sourced from a driver rather than verified against its own "
        "documentation can silently be something other than what its name "
        "claims — here, a controller's payload-compensated force estimate "
        "standing in for a transducer reading. What is rarely reported is "
        "whether such a dependency was checked at all, and what a fusion "
        "framework's measured advantage is worth once it is removed. This "
        "paper reports both for one complete system, together with the "
        "offline consequences of removing it and, as an addendum once "
        "available, its hardware behaviour."
    )
    b.p("The contributions are:")
    for item in (
        "a finding that the end-effector force/torque channel two prior "
        "stages of this framework depended on is a controller estimate, not a "
        "measurement — traced to the ROS 2 driver source and quantified "
        "against the channel it should be uncorrelated with (Section 3.4);",
        "a residual redefinition from a twelve-channel intrinsic/extrinsic "
        "split to a single six-channel total residual, and a raw-model "
        "channel reduction from twenty-four to sixteen, with the negative "
        "result that a wrap-around-safe (sin q, cos q) encoding, tried as an "
        "alternative to dropping two position channels outright, makes "
        "generalisation worse rather than better (Section 3.5);",
        "a task-conditioned payload term fitted outside the validated solver "
        "alongside the friction term, replacing a single global offset shown "
        "to generalise backwards on at least one held-out task, evaluated on "
        "four production tasks and 5.16 million prepared samples (Section 3.4);",
        "an independent, RTDE-based re-derivation of the current-to-torque "
        "calibration across the full operating envelope rather than only "
        "near rest, which agrees with the earlier quasi-static values to "
        "within 2 % on the two joints either method can measure (Section "
        "3.3);",
        "a fusion weight measured rather than assumed a priori, converging to "
        "a narrow high-residual optimum under physically consistent injection "
        "once the unreliable channel is removed, and a log-domain "
        "normalisation adopted specifically to pre-empt the threshold-"
        "transfer failure mechanism the intermediate stage found on hardware "
        "(Sections 3.6, 4.3);",
        "a re-implementation as a ROS 2 node without the removed channel, "
        "numerically re-verified against the offline pipeline (Section 3.8); "
        "hardware commissioning is reported as an addendum once the trial "
        "reported pending in Section 4.6 is complete.",
    ):
        p = b.para("", style="Paragraf", align=WD_ALIGN_PARAGRAPH.JUSTIFY,
                   space_before=3, space_after=0)
        p.paragraph_format.left_indent = Cm(0.4)
        b.rich(p, "•  " + item)
    b.p(
        "The remainder of the paper is organised as follows. Section 2 reviews the "
        "related literature. Section 3 describes the platform and the data, the "
        "revised residual definition and the online implementation. Section 4 "
        "presents the offline findings. Section 5 discusses them and states the "
        "limitations, and Section 6 concludes."
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
        "inspection chassis frame, shown in Figure 1. The cell performs four "
        "production tasks on the same hardware: an autonomous visual inspection "
        "cycle of automotive body panels running alone, the same inspection cycle "
        "coordinated with a second, independently moving robot on the shared "
        "deck, a pick-and-place transfer task, and a fastening task in which the "
        "end effector is exchanged for a driving tool. The cell runs ROS 2 Humble "
        "with MoveIt 2; the recorded motions of the inspection tasks were produced "
        "by eleven different motion planning algorithms so that the normal "
        "operating distribution is not dominated by a single planner, and the "
        "other two tasks contribute their own, structurally different motion "
        "and load profiles. No force/torque sensor is fitted; Section 3.4 "
        "establishes why the channel a force/torque sensor would occupy cannot be "
        "used as one."
    )
    b.figure(
        "fig1_platform.png",
        "Robotic Quality Inspection Platform | UR10e robot on a Festo linear "
        "axis, with the inspection chassis frame on the right.",
        wide=False, width_cm=6.2)
    b.p(
        "The source dataset is a joint-state export covering all four tasks, "
        "8,294,076 samples nominally at 500 Hz, together with a separate "
        "148.6-minute Real-Time Data Exchange (RTDE) capture used only for the "
        "calibration of Section 3.3. Model training was performed on a laptop "
        "with an Intel Core i7-13650HX processor and an NVIDIA RTX 4060 GPU."
    )
    b.p(
        "The study uses no human subjects and required no ethics committee "
        "approval. All measurements were collected on laboratory equipment of the "
        "ESOGÜ Intelligent Systems Application and Research Centre, and the "
        "principles of research and publication ethics were observed throughout."
    )

    b.h2("3.2. Data Investigation And Preparation")
    b.p(
        "The first stage of the audit was the data itself, and, as in the "
        "extension this study itself builds on, it produced the largest single "
        "correction — this time a different and more severe one. The export is "
        "not merely locally gapped; its row order does not track wall-clock time "
        "at all. Sorting the 8,294,076 samples by their header timestamp exposes "
        "jumps of hundreds of hours in either direction between consecutive "
        "rows as exported — a sequence advancing from January 2026 to June 2026 "
        "is, a few rows later, immediately followed by a return to August 2025 — "
        "consistent with an unordered concatenation of many separate recording "
        "sessions spanning roughly thirteen months rather than one contiguous "
        "export. No notion of a run, a gap or an acceleration is meaningful "
        "before this ordering is restored."
    )
    b.p(
        "Once sorted, three further conditions were applied, each removing a "
        "specific and named contamination rather than a fixed fraction of the "
        "data: rows outside the controller's own safety_mode = NORMAL state "
        "(protective stop, fault and emergency stop) are excluded; rows carrying "
        "no production-task label (the controller's IDLE state between tasks) are "
        "excluded; and sessions whose median inter-sample interval exceeds 4 ms "
        "are excluded outright, which isolates a set of early sessions recorded "
        "before the current 500 Hz driver configuration, at an effective rate "
        "close to 10 Hz — an order of magnitude too slow for the fixed-step "
        "Savitzky–Golay derivative (Savitzky and Golay, 1964, window 51, order "
        "3, detailed in Section 3.8) to remain physical."
    )
    b.p(
        "The remaining stream is then segmented into physically continuous "
        "sessions and, within each session, into runs short enough to fail the "
        "hundred-sample autoencoder window plus its fifty-sample derivative "
        "margin. The session-boundary threshold — the inter-sample gap beyond "
        "which two consecutive samples are no longer treated as continuous — was "
        "chosen from the data rather than assumed: four candidate thresholds were "
        "scored by the fraction of each task's samples retained in sessions of at "
        "least 150 samples (Table 1). Ten and twenty milliseconds differ sharply "
        "for the least sampled task (54 % against 96 %), while fifty milliseconds "
        "adds almost nothing further; twenty milliseconds was adopted."
    )
    b.table(
        "Session-Boundary Threshold Selection (Share Of Task Samples Retained In "
        "Sessions Of At Least 150 Samples).",
        ["Task", "6 ms", "10 ms", "20 ms", "50 ms"],
        [["Fastening (HRC)", "93.8 %", "98.6 %", "99.8 %", "99.9 %"],
         ["Cooperative inspection", "77.9 %", "93.5 %", "99.8 %", "100.0 %"],
         ["Pick-and-place", "44.8 %", "77.1 %", "99.6 %", "100.0 %"],
         ["Solo inspection", "24.5 %", "54.2 %", "95.5 %", "98.7 %"]],
        widths=[3.6, 1.9, 1.9, 1.9, 1.9], align_right=[1, 2, 3, 4])
    b.p(
        "Table 2 gives the outcome of the full cascade. The reduction is not a "
        "loss in the sense of the earlier study: every removed sample belongs to "
        "an explicitly named condition (out of safety, unlabelled, duplicated, "
        "too slow, or too short to host a single window), and each count is "
        "reported rather than absorbed into a single interpolation figure."
    )
    b.table(
        "Outcome Of Data Preparation.",
        ["Quantity", "Value"],
        [["Raw exported samples", "8,294,076"],
         ["Removed — safety_mode ≠ NORMAL", "93,249  (1.1 %)"],
         ["Removed — unlabelled (IDLE)", "2,313,148  (27.9 %)"],
         ["Removed — duplicate timestamp", "103,272  (1.2 %)"],
         ["Removed — session shorter than 150 samples", "187,868  (2.3 %)"],
         ["Removed — unlabelled legacy, stationary", "282,416  (3.4 %)"],
         ["Removed — session rate below ≈250 Hz", "31,181  (0.4 %)"],
         ["Session-bounded samples", "5,282,942"],
         ["Prepared samples (after derivative margin)", "5,160,792"],
         ["Physical sessions retained", "2,443"]],
        widths=[6.1, 2.25], align_right=[1],
        note="The derivative margin removes 25 samples from each end of each of "
             "the 2,443 retained sessions (2 × 25 × 2,443 = 122,150), matching the "
             "Stage-A/Stage-B sample difference exactly.")

    b.h2("3.3. Current-To-Torque Calibration")
    b.p(
        "The UR ROS 2 driver writes the motor current, in amperes, into the effort "
        "field of the joint state message; the field is filled directly from the "
        "actual current reported by the controller. The inverse dynamics model, in "
        "contrast, produces newton metres. A conversion coefficient is therefore "
        "required before the residual of Equation (1) can be formed at all — the "
        "same requirement as in the extension this study builds on, but re-derived "
        "here from a different and independent source."
    )
    b.p(
        "The earlier method regressed measured current on model torque restricted "
        "to quasi-static samples (|q̇| < 0.005 rad/s), where the joint torque is "
        "dominated by gravity; four of the six joints could not be measured this "
        "way because their quasi-static gravity torque is negligible. RTDE exposes "
        "a second, independent quantity: target_moment, the controller's own "
        "torque estimate for the commanded trajectory, populated at every control "
        "cycle rather than only near rest. Regressing measured current on "
        "target_moment over the full RTDE capture — 1,016,438 samples after the "
        "same safety-mode filter, 148.6 minutes of controller time, every task "
        "represented — extends the excitation from a near-zero-velocity subset to "
        "the whole operating envelope. Table 3 lists the result."
    )
    b.table(
        "Current-To-Torque Coefficients From The RTDE Regression.",
        ["Joint", "Nm/A", "R²", "target_moment σ [Nm]", "Trusted"],
        [["shoulder_pan", "14.569", "0.007", "1.02", "no"],
         ["shoulder_lift", "10.723", "0.971", "63.83", "yes"],
         ["elbow", "9.055", "0.960", "31.80", "yes"],
         ["wrist_1", "6.365", "0.540", "2.48", "no (weak)"],
         ["wrist_2", "7.691", "0.042", "0.55", "no"],
         ["wrist_3", "13.583", "0.001", "0.12", "no"]],
        widths=[2.7, 1.7, 1.6, 3.3, 2.0], wide=True, align_right=[1, 2, 3],
        note="A coefficient is trusted at R² ≥ 0.70 and a physically plausible "
             "value; only shoulder_lift and elbow pass, the same two joints "
             "trusted by the quasi-static method — and the two independent "
             "estimates agree to within 2 % (10.723 against 10.522 Nm/A; 9.055 "
             "against 9.130 Nm/A), obtained from unrelated samples and an "
             "unrelated regressor. target_moment σ is the standard deviation of "
             "the regressor on that joint: the four untrusted joints all carry "
             "under 2.5 Nm of exciting signal against 32-64 Nm for the trusted "
             "two, which is why no regression against any single-valued torque "
             "proxy identifies them, not a defect of this particular method. "
             "wrist_1 is the one joint where the wider excitation helps: R² rises "
             "from 0.000 under the quasi-static method to a still-untrusted but "
             "non-zero 0.540.")
    b.p(
        "Untrusted coefficients are filled by copying the nearest trusted joint "
        "within the same actuator family — shoulder_pan from shoulder_lift, and "
        "wrist_2 and wrist_3 from wrist_1, the best-conditioned of the three wrist "
        "joints — rather than by the torque-limit-ratio scaling used previously. "
        "As before, this choice affects the physical interpretation of a fault "
        "amplitude in newton metres on four of six joints and does not affect "
        "detection, because the autoencoders standardise every input channel; the "
        "node warns about the affected joints at start-up."
    )

    b.h2("3.4. Residual Definition And The Removal Of The Force/Torque Channel")
    b.p(
        "The inverse dynamics model packages the Newton–Euler formulation of the "
        "UR10e in the Functional Mock-up Interface format (Blochwitz et al., 2011):"
    )
    b.equation("τ̂_model  =  M(q)·q̈ + C(q, q̇)·q̇ + g(q)")
    b.p(
        "The extension this study builds on defined the residual as the "
        "measurement-model difference split into an intrinsic and an extrinsic "
        "part, r_ext = J(q)ᵀ·F_FTS, using an end-effector wrench topic the ROS 2 "
        "driver advertises. That split is withdrawn here, and this subsection "
        "reports why, because the reason is a finding in its own right rather "
        "than a simplification of convenience."
    )
    b.p(
        "**The wrench channel is not a physical measurement.** The UR ROS 2 "
        "driver's hardware interface populates it by reading the RTDE field "
        "actual_TCP_force, which is the controller's own force estimate, computed "
        "from joint currents under whichever payload mass is currently configured "
        "— not a transducer reading. The RTDE field that would carry a genuine "
        "strain-gauge signal, ft_raw_wrench, was sampled independently over 2,000 "
        "points and found to sit at an approximately constant offset of order "
        "10⁴ in raw sensor units on every channel, with no response distinguishable "
        "from noise; no force/torque sensor is fitted to this cell, and the driver "
        "does not report the absence."
    )
    b.p(
        "The consequence is quantitative, not merely definitional. The RTDE "
        "capture used in Section 3.3 shows the controller's active payload "
        "setting taking at least four distinct values (0.001, 0.20, 0.44 and "
        "1.86 kg) across sessions, presumably one per tool actually mounted; "
        "when the configured value does not match the true mounted mass — which "
        "it need not, since nothing in the pipeline enforces it — the reported "
        "actual_TCP_force carries a static bias for the whole session, static "
        "pose included. Measured across the four production tasks, the "
        "between-task standard deviation of a wrench-derived extrinsic residual "
        "is 20.9 Nm, against 3.1 Nm for the total residual defined below without "
        "any wrench term; reintroducing the split would add roughly seven times "
        "more task-dependent noise than it removes."
    )
    b.p(
        "The residual is therefore a single quantity, the total residual of the "
        "withdrawn decomposition without a channel to subtract from it:"
    )
    b.equation("r_tot  =  τ_meas − τ̂_model − τ̂_c(q, q̇)")
    b.p(
        "where τ̂_c is a correction term, estimated outside the validated solver "
        "exactly as the friction term of the extension this study builds on, "
        "which is why it is retained and extended rather than discarded. Two "
        "properties of the solver motivate correcting outside rather than inside "
        "it, both unchanged from the earlier finding: the friction vector is "
        "identically zero over 500 random joint poses, and no payload model is "
        "present, so a workpiece or tool produces a sustained bias the solver "
        "cannot explain. The correction now has three parts, "
    )
    b.equation("τ̂_c  =  F_c·tanh(q̇ / ε) + F_v·q̇  +  b_task  +  m_task·A(q) + u_task·B(q)",
               "2a")
    b.p(
        "a Coulomb-plus-viscous friction term identical in form to the earlier "
        "one (tanh in place of sign(q̇) for the same reason: this robot spends "
        "most of its time at low speed, where a discontinuous sign function "
        "injects a step at every reversal, precisely the shape a detector is "
        "meant to flag), and a task-conditioned offset b_task plus a "
        "task-conditioned payload term. The payload term follows the standard "
        "result for a point mass rigidly attached at the flange,"
    )
    b.equation("A(q)  =  g · Jᵥ(q)ᵀ·ẑ,     B(q)  =  g · (ẑ × Jᵥ(q))ᵀ·R(q)", "2b")
    b.p(
        "where Jᵥ(q) is the linear-velocity block of the geometric Jacobian at "
        "the flange, R(q) the flange orientation, ẑ the vertical unit vector and "
        "g gravitational acceleration; m_task (kg) is then the extra mass the "
        "task's tooling represents relative to whatever nominal load the solver "
        "assumes, and u_task = m_task·c_task (kg·m) its first moment about the "
        "flange. Both A(q) and B(q) are functions of the already-available pose "
        "only, so the term costs one Jacobian evaluation, already computed for "
        "the equivalence check of Section 3.8, and no additional model."
    )
    b.p(
        "F_c, F_v and, per task, b_task, m_task, u_task are fitted jointly by "
        "weighted least squares — each joint's equation weighted by the inverse "
        "of its own residual spread, so that shoulder_lift and shoulder_pan "
        "cannot dominate the fit of the wrist joints — exclusively on the "
        "training sessions of the split defined in Section 3.7. A "
        "held-out comparison decided whether the payload term earns its "
        "complexity: fitted with b_task alone against fitted with the full term "
        "of Equation (2a), scored on validation sessions the fit never saw, the "
        "mean ratio of corrected to raw residual spread is 0.537 for the offset "
        "alone against 0.477 with the payload term, and the offset-alone model "
        "fails in a way a single scalar cannot fix: on the pick-and-place task it "
        "leaves a validation-set bias of +23.9 Nm on the shoulder-lift channel, "
        "opposite in practical effect to the −5.1 Nm fitted on the training "
        "sessions of the same task, because the true extra-mass torque is "
        "pose-dependent and a constant cannot track it. The full term was "
        "adopted; Table 4 lists its coefficients and Table 5 the residual "
        "reduction."
    )
    b.table(
        "Friction And Per-Task Payload Coefficients, Fitted On Training Sessions Only.",
        ["Joint", "F_c [Nm]", "F_v [Nm·s/rad]"],
        [["shoulder_pan", "9.44", "24.44"],
         ["shoulder_lift", "11.78", "11.91"],
         ["elbow", "6.09", "17.92"],
         ["wrist_1", "2.30", "3.42"],
         ["wrist_2", "2.57", "3.20"],
         ["wrist_3", "2.42", "4.12"]],
        widths=[3.4, 2.9, 3.9],
        note="Per-task terms (b_task, m_task, u_task) are not tabulated per "
             "joint for space; the fitted extra mass m_task ranks the four tasks "
             "in the expected order — fastening tool +1.43 kg, cooperative "
             "inspection +0.48 kg, pick-and-place +0.22 kg, solo inspection "
             "−0.30 kg (lighter than the solver's nominal load) — but is reliable "
             "only as a ranking: at these small magnitudes the fitted centre-of-"
             "gravity offset u_task/m_task is not separately identifiable and is "
             "not reported as a measured quantity.")
    b.table(
        "Total Residual Standard Deviation Before And After Correction [Nm], By Split.",
        ["Joint", "Train", "Validation", "Test"],
        [["shoulder_pan", "8.42 → 4.88", "8.26 → 5.46", "9.61 → 5.37"],
         ["shoulder_lift", "12.00 → 7.16", "11.58 → 6.62", "12.93 → 6.69"],
         ["elbow", "7.07 → 3.05", "7.88 → 2.96", "8.00 → 2.76"],
         ["wrist_1", "2.03 → 0.89", "2.16 → 0.79", "2.29 → 0.94"],
         ["wrist_2", "2.06 → 0.85", "2.20 → 1.02", "2.36 → 0.97"],
         ["wrist_3", "1.99 → 0.73", "2.00 → 0.84", "2.18 → 0.90"]],
        widths=[2.7, 2.6, 2.6, 2.6], wide=True,
        note="Every joint's spread falls on validation and test as well as on "
             "train, which the offset-only comparison above did not achieve; the "
             "reduction is a genuine generalisation, not a fitting artefact.")
    b.p(
        "The online feature engine applies the identical expression from the "
        "same coefficient file, selected by the task the cell is currently "
        "running, and the equivalence test of Section 3.8 is run with the full "
        "correction active."
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
        "The residual model's six channels follow directly from Section 3.4. The "
        "raw model's channels do not follow as directly as the twenty-four of the "
        "extension this study builds on: removing the wrench leaves eighteen "
        "(six positions, six velocities, six torques), and two of the six "
        "positions were removed further, on evidence rather than by design. An "
        "eighteen-channel model was trained first; two of the thirty sessions in "
        "the test and validation sets were reconstructed almost uniformly badly "
        "— 28.8 % and 19.2 % of their windows above threshold — traced to the "
        "shoulder_pan and, overwhelmingly, the wrist_3 channel visiting joint-"
        "angle ranges the training sessions never covered (up to 43.9 % of one "
        "session's samples outside the training range on wrist_3 alone). Neither "
        "joint's angle enters the gravity term the residual model already "
        "captures — shoulder_pan is the vertical axis, wrist_3 the terminal, "
        "unbounded-rotation joint — so both were dropped, leaving sixteen. A "
        "further attempt encoded the four remaining position channels as "
        "(sin q, cos q) pairs instead of dropping wrist_3 and shoulder_pan, to "
        "remove the wrap-around discontinuity without discarding the position "
        "information entirely; it made the same two sessions worse, not better "
        "(62.0 % and 18.8 % of windows above threshold), indicating that the "
        "affected sessions visit genuinely novel task poses rather than an "
        "angle-wrapping artefact, and that added position sensitivity amplifies "
        "the effect. The sixteen-channel model — positions of joints 2 to 5, all "
        "six velocities, all six torques — was retained."
    )
    b.p(
        "Windowing uses a stride of 25 samples, that is 75 % overlap. Two rules are "
        "enforced jointly: no window may span two sessions, and windows containing "
        "samples invalidated by the derivative edge margin are dropped. Training and "
        "validation windows come from disjoint, task-balanced sets of physical "
        "sessions (Section 3.7), so no sample is shared between them. Table 6 "
        "reports the architectures and the training outcome of the deployed "
        "single-seed models; a five-seed sensitivity study, run for the extension "
        "this study builds on, was not repeated here and is listed among the "
        "items still outstanding in Section 5.6."
    )
    b.table(
        "Model Architectures And Training Outcome.",
        ["", "Residual AE", "Raw AE"],
        [["Input channels", "6", "16"],
         ["Hidden / latent", "128 / 32", "256 / 64"],
         ["Parameters", "421,670", "1,683,536"],
         ["Train / val. windows", "125,662 / 39,956", "125,662 / 39,956"],
         ["Best epoch", "300", "5"],
         ["Best validation loss", "0.0744", "0.0798"],
         ["Threshold θ (P97)", "0.565", "0.739"]],
        widths=[2.95, 2.7, 2.7], align_right=[1, 2],
        note="Optimiser Adam (lr = 10⁻³, β = 0.9/0.999), batch size 256, gradient "
             "clipping at 1.0, ReduceLROnPlateau (factor 0.5, patience 8), early "
             "stopping with patience 25 over at most 300 epochs. The raw model's "
             "best epoch is five: with eighteen times more training windows than "
             "the extension this study builds on had, the validation loss plateau "
             "is reached almost immediately and patience 25 then exhausts itself "
             "without further gain, unlike the residual model, which keeps "
             "improving to the full 300-epoch budget. Both channel counts and "
             "both thresholds differ from the twelve/twenty-four-channel, "
             "0.887/0.412 architecture of the extension this study builds on for "
             "the reasons of Sections 3.4 and above, not as a refinement of the "
             "same quantity.")

    b.h2("3.6. Score-Level Fusion And Log-Domain Normalisation")
    b.p(
        "The normalised scores of the two models are combined by a weighted average,"
    )
    b.equation("S_fused  =  w_res · z_res  +  w_raw · z_raw")
    b.p(
        "with w_res + w_raw = 1. Two changes are made to the fusion of the "
        "extension this study builds on, one to how the weight is chosen and one "
        "to how a score is normalised, and both follow from the same "
        "observation: the reconstruction-error distribution of either model is "
        "heavy-tailed by three orders of magnitude between its median and its "
        "99.9th percentile, which an affine min–max transform does not compress."
    )
    b.p(
        "The weight is measured rather than adopted a priori. w_res is chosen on "
        "validation windows only, by a sweep over [0, 1] in steps of 0.05 under "
        "the physically consistent fault injection of Section 3.7, and the choice "
        "is cross-checked across three normalisations and three raw-channel "
        "variants of Section 3.5 (eighteen, sixteen and the sin/cos-encoded "
        "twenty-four) so that the outcome is not an artefact of one arbitrary "
        "scaling. Every combination selects a weight at or above 0.90; the value "
        "adopted, w_res = 0.90, is the sixteen-channel, log-normalised optimum "
        "and is discussed against the alternatives in Section 4.3."
    )
    b.p(
        "The normalisation itself moves from an affine min–max transform to a "
        "standardised log transform,"
    )
    b.equation("z  =  (log₁₀(S + ε) − μ) / σ")
    b.p(
        "with ε = 10⁻⁹ guarding the logarithm and μ, σ the mean and standard "
        "deviation of log₁₀(S + ε) over clean validation windows — estimated once "
        "per model, from the same causal set the reference formula already used, "
        "so the change is in the shape of the transform rather than in which "
        "windows inform it. The reason is the tail: a min–max bound taken from "
        "clean validation windows is set by whichever window happened to "
        "reconstruct worst, and one atypical window can set a bound the rest of "
        "the distribution never approaches, compressing ordinary variation to "
        "invisibility exactly as the fault-injected min–max bound of the "
        "reference formula did for a different reason (Section 5.3 returns to "
        "this in the context of the operating threshold). The log transform "
        "instead maps the bulk of the distribution to an approximately symmetric "
        "range and lets the tail extend rather than dominate it."
    )

    b.h2("3.7. Data Split, Fault Injection And Evaluation Protocol")
    b.p(
        "The 2,443 physical sessions are partitioned once, and every consumer "
        "reads the same partition: the friction and payload fit of Section 3.4, "
        "the autoencoder training of Section 3.5, and the evaluation below. The "
        "partition is by session, not by row index, for the reason established in "
        "the extension this study builds on: a partition that lets a test window "
        "share a run, and therefore near-identical dynamics, with a training "
        "window overstates every downstream metric. A second requirement is added "
        "here, because it did not previously apply — the source data carries no "
        "task label. Sessions are allocated per task, largest first, to whichever "
        "of train, validation and test is furthest below its 70/15/15 sample-share "
        "target, so that every task is represented in every split rather than one "
        "task dominating a split by chance; two tasks with as few as five and six "
        "sessions each make this allocation, not simple random sampling, "
        "necessary."
    )
    b.table(
        "Task-Balanced Run-Disjoint Split (Samples).",
        ["Task", "Train", "Validation", "Test"],
        [["Fastening (HRC)", "980,613", "355,939", "264,937"],
         ["Cooperative inspection", "1,481,403", "396,909", "360,616"],
         ["Pick-and-place", "293,166", "161,180", "93,480"],
         ["Solo inspection", "504,429", "140,080", "128,040"],
         ["**Total**", "**3,259,611**", "**1,054,108**", "**847,073**"]],
        widths=[3.0, 2.5, 2.5, 2.5], wide=True,
        align_right=[1, 2, 3],
        note="63.2 / 20.4 / 16.4 % overall, short of the 70/15/15 target because "
             "the two smallest tasks (five and six sessions) cannot be split "
             "more finely without breaking the no-shared-session rule; every "
             "task is nonetheless present in every split. Windowing (Section "
             "3.5) yields 125,662 / 39,956 / 32,305 windows.")
    b.p(
        "Table 8 lists the four synthetic fault scenarios injected for evaluation, "
        "unchanged in kind from the extension this study builds on but revised in "
        "two respects. First, only the physically consistent injection protocol "
        "is used: faults perturb the measured channels — joint torque or joint "
        "position — and the residual is recomputed through the pipeline itself, "
        "rather than being added by hand to each model's own representation "
        "space; the predecessor's central finding, that the two protocols disagree "
        "and only the physical one is meaningful (its own Section 5.2), is taken "
        "as established rather than re-derived. Second, each amplitude is swept "
        "at 0.5×, 1× and 2× severity rather than injected once, so that a fault "
        "type's detectability is reported as a curve, not a single point. A "
        "window is labelled anomalous when at least 30 % of its samples overlap "
        "the fault mask, drawn to give an overall anomalous-window prevalence of "
        "8 %, matching the design of the original offline study this line of "
        "work traces back to."
    )
    b.table(
        "Synthetic Fault Scenarios (Base Amplitude, Swept At 0.5×/1×/2×).",
        ["Scenario", "Injected into", "Base amplitude"],
        [["Motor drift", "Elbow torque, linear ramp", "15 Nm"],
         ["Collision", "All joint torques, Gaussian pulse", "12 % of each joint's torque limit"],
         ["Encoder step", "Wrist-2 position, step", "0.3 rad"],
         ["Measurement noise", "All joint torques, Gaussian noise", "σ = 3/3/2/1/1/1 Nm"]],
        widths=[2.4, 3.9, 3.7], wide=True,
        note="Without a wrench channel, collision is injected directly as a "
             "joint-torque pulse rather than as a wrench propagated through the "
             "Jacobian from an assumed contact point; Section 5.6 lists this as a "
             "limitation. The encoder step is reduced from the 1.5 rad of the "
             "extension this study builds on to 0.3 rad, judged a more plausible "
             "single-glitch magnitude; Section 4.2 reports what this costs the "
             "residual model's sensitivity to it.")

    b.h2("3.8. Online Implementation")
    b.p(
        "The online system differs structurally from the offline pipeline in three "
        "ways, unchanged from the extension this study builds on: causality "
        "(nothing computed over a whole dataset may be used live), a time budget "
        "(2 ms per sample and, with a stride of 25, 50 ms per decision), and "
        "continuity (the live system notices an interruption only after it has "
        "happened and must reset its state). A fourth is new — the correction of "
        "Section 3.4 is task-conditioned, so the running task must be known to the "
        "node rather than inferred, and is taken as a required startup parameter."
    )
    b.figure(
        "fig2_architecture.png",
        "Online Detection Pipeline | joint-state features feed a six-channel "
        "residual autoencoder and a sixteen-channel raw autoencoder in "
        "parallel; scores are combined under log-domain normalisation "
        "(Section 3.6) at the measured weight w_res = 0.90.",
        wide=True)
    b.p(
        "The feature engine is the sample-by-sample equivalent of the offline "
        "computation, unchanged in structure and reduced in scope: it keeps a "
        "51-sample ring buffer and, for every new sample, emits the features of "
        "the sample at the centre of the buffer — the Savitzky–Golay derivative "
        "applied as an inner product with precomputed coefficients, the "
        "current-to-torque conversion, the inverse dynamics evaluation and the "
        "task-conditioned correction of Equation (2a) — without the base-frame "
        "rotation and Jacobian transfer the withdrawn wrench term required. The "
        "centred derivative costs the same structural delay of 25 samples (50 ms) "
        "as before."
    )
    b.p(
        "Equivalence was verified rather than assumed, as before. Recorded "
        "sessions from four tasks were pushed through the online engine sample by "
        "sample and compared with the offline feature file: over four sessions "
        "the largest absolute difference in the total residual is 1.5·10⁻⁶ Nm and "
        "in the fused score 5.2·10⁻⁴ relative, with the moving/static regime flag "
        "agreeing on every decision. These are far tighter than the tolerance "
        "that matters — the gap between clean and anomalous scores — so the "
        "deployed node sees the features the models were trained on, not merely "
        "similar ones."
    )
    b.p(
        "The two optional rules of the extension this study builds on are kept "
        "unchanged in mechanism: an adaptive rule that fires when the fused score "
        "exceeds median + k·1.4826·MAD of the last 600 decisions (Leys, Ley, "
        "Klein, Bernard and Licata, 2013), with its baseline frozen during an "
        "alarm and the freeze released after 3 s so a long alarm is treated as a "
        "regime change; and a two-consecutive-decision rule before an alarm is "
        "raised. The predecessor's own commissioning found the adaptive rule's "
        "premise broken on hardware — the residual score is pose-dependent, not "
        "low during ordinary operation — and disabled it by default; because the "
        "task-conditioned payload term of Section 3.4 is aimed at exactly that "
        "pose dependence, whether it is disabled by default here is left to the "
        "commissioning trial of Section 4.6 rather than decided in advance."
    )
    b.p(
        "The detector is packaged as a ROS 2 node (Macenski, Foote, Gerkey, "
        "Lalancette and Woodall, 2022) that keeps the whole computation in a "
        "ROS-independent core; the node is a thin wrapper around it, so replay "
        "tests exercise the class the node actually runs. The models are exported "
        "to ONNX and executed with ONNX Runtime. Every decision is written to a "
        "comma-separated log and every alarm to a line-buffered JSON-lines event "
        "log, each session's log tagged with the checksum of the models and "
        "configuration it ran — the predecessor's commissioning was set back once "
        "by a session that silently ran superseded models, a failure this record "
        "is designed to make legible after the fact rather than to prevent."
    )
    b.p(
        "Two connection-layer measurements repeat from before and one is new. "
        "The joint state topic still carries three different name sets from "
        "three publishers, of which the UR data arrives in a seven-element set "
        "in scrambled order; the mapping is still resolved per message and "
        "cached per name set. The measured sample rate is still close to "
        "500 Hz rather than exactly on it. New this round: because no wrench "
        "topic is subscribed to at all, the single largest source of silent "
        "failure in the predecessor's deployment — a wrench topic name mismatch "
        "that produced no score with no error — cannot recur; removing the "
        "channel removed the failure mode along with it."
    )

    b.h2("3.9. Operator Interface")
    b.p(
        "The detector was integrated into the laboratory's existing web "
        "dashboard as a third tab, unchanged in design from the extension this "
        "study builds on: a collector subscribes to the node's decision, alarm "
        "and score topics, keeps a rolling buffer and pushes it to the browser. "
        "Alarms are latched on the rising edge and held rather than sampled at a "
        "fixed rate, because a short alarm can otherwise fall entirely between "
        "two samples of a periodic display. The vertical axis is framed on the "
        "live threshold rather than on a fixed range, because the score scale "
        "moves when the task changes; a logarithmic axis was rejected because it "
        "visually flattens the region above the threshold, the only region that "
        "matters operationally. Both the axis and the displayed per-model "
        "thresholds are read from the live decision stream rather than compiled "
        "in, so a recalibration cannot leave the display describing a "
        "configuration that is no longer running. The event table it produces "
        "is the source of the commissioning record in Section 4.6."
    )

    # ──────────────────────────────────────────── 4. Findings ────────
    b.h1("4. Findings")

    b.h2("4.1. Offline Performance Of The Extended Pipeline")
    b.p(
        "The extended pipeline is evaluated on 32,305 test windows and 39,956 "
        "validation windows, drawn from sessions that contributed nothing to "
        "training, the friction/payload fit, or the choice of any threshold; "
        "windows are labelled by the physically consistent injection of Section "
        "3.7 at 8 % prevalence, averaged over the three severities and four "
        "fault types of Table 8. A single seed is reported (Section 3.5); Table "
        "9 gives the result."
    )
    b.figure(
        "fig3_pr_roc.png",
        "Precision-Recall And ROC, Severity 1× | combined test-set windows "
        "(8 % prevalence, all four fault types pooled at severity 1×, not "
        "averaged over severities as Table 9 is — the small numeric offset "
        "from Table 9 follows from that difference, not from a second "
        "measurement).",
        wide=True)
    b.table(
        "Overall Performance On The Run-Disjoint Test Set.",
        ["Model", "AUC", "PR-AUC", "Best F1"],
        [["Residual LSTM AE", "0.825", "0.506", "0.611"],
         ["Raw LSTM AE (16 ch.)", "0.735", "0.238", "0.397"],
         ["**Fusion (w_res = 0.90, log)**", "0.824", "0.504", "0.609"]],
        widths=[3.6, 1.9, 1.9, 1.9],
        align_right=[1, 2, 3],
        note="Classical baselines (Isolation Forest, One-Class SVM, a residual-"
             "norm threshold) were part of the comparison in the extension this "
             "study builds on and were not re-fitted for this revision; their "
             "omission here is a gap, not a finding, and is listed in Section "
             "5.6. The headline number is the one the rest of this section "
             "explains rather than states plainly: in aggregate, across four "
             "fault types of very different physical character, the fused "
             "detector does not exceed the residual model alone (Best F1 0.609 "
             "against 0.611). Section 4.2 shows why an aggregate is the wrong "
             "level to read this result at.")
    b.p(
        "The aggregate is a mean over fault types the residual model handles very "
        "differently, not a single operating regime, and Section 4.2 is the "
        "reason a one-line summary understates what is actually happening."
    )

    b.h2("4.2. Fault-Type Asymmetry And Complementarity")
    b.p(
        "Table 10 breaks the result down by fault type and severity. The "
        "asymmetry the fusion is meant to exploit is present, but concentrated "
        "in one fault type rather than spread evenly across two, as it was under "
        "the earlier, hand-chosen injection: on the encoder step the residual "
        "model is close to chance (AUC 0.505-0.550 across severities) while the "
        "raw model, though weak in absolute terms, is consistently better "
        "(0.514-0.636). On the other three fault types the residual model is "
        "equal to or clearly ahead of the raw model at every severity."
    )
    b.table(
        "Detection By Fault Type And Severity (AUC).",
        ["Scenario", "Severity", "Residual", "Raw (16 ch.)", "Fusion"],
        [["Motor drift", "0.5×", "0.581", "0.531", "0.578"],
         ["", "1×", "0.838", "0.606", "0.820"],
         ["", "2×", "0.983", "0.718", "0.981"],
         ["Collision", "0.5×", "0.988", "0.862", "0.989"],
         ["", "1×", "0.999", "0.915", "0.999"],
         ["", "2×", "1.000", "0.996", "1.000"],
         ["Encoder step", "0.5×", "0.505", "0.514", "0.507"],
         ["", "1×", "0.517", "0.549", "0.520"],
         ["", "2×", "0.550", "0.636", "0.558"],
         ["Measurement noise", "0.5×", "0.943", "0.706", "0.943"],
         ["", "1×", "0.991", "0.821", "0.990"],
         ["", "2×", "0.999", "0.874", "0.999"]],
        widths=[3.4, 1.8, 2.1, 2.4, 2.0], wide=True, align_right=[2, 3, 4],
        note="Fusion is w_res = 0.90 under the log normalisation of Section 3.6. "
             "Encoder step is the only fault type where the raw model is ahead "
             "of the residual model at every severity, and the reason is "
             "quantitative rather than architectural: a 0.3 rad step on wrist_2 "
             "changes the modelled gravity torque of the six joints by at most "
             "0.13 Nm (computed from the same inverse dynamics solver, holding "
             "velocity and acceleration at zero), against a residual noise "
             "floor of 0.9-6.7 Nm measured on clean test windows — the fault is "
             "below the residual model's noise floor by construction, not by a "
             "limitation of the fit.")
    b.p(
        "This is the physical account of why the aggregate of Section 4.1 shows "
        "no fusion gain: three of four fault types leave the residual model "
        "already close to its ceiling, where a 10 % raw-model weight can only "
        "subtract, and the fourth is exactly where the raw model earns its "
        "keep but is itself weak (AUC at best 0.636), so a small aggregate loss "
        "on the majority of fault types is not clearly repaid by a small "
        "aggregate gain on the minority. Section 4.4 returns to whether this "
        "means the ten per cent weight is misplaced."
    )

    b.h2("4.3. Sensitivity To The Fusion Weight")
    b.p(
        "The weight is measured, not assumed (Section 3.6): three "
        "normalisations were swept over w_res ∈ [0, 1] in steps of 0.05 on "
        "validation windows, each paired with the sixteen-channel raw model of "
        "Section 3.5. Table 11 gives the outcome."
    )
    b.table(
        "Fusion Weight Selected By Three Normalisations (Test Set).",
        ["Normalisation", "w_res selected", "Test AUC", "Test PR-AUC", "Test Best F1"],
        [["Ratio-to-threshold", "1.00", "0.825", "0.506", "0.611"],
         ["**Log (adopted)**", "0.90", "0.824", "0.504", "0.609"],
         ["Empirical rank", "1.00", "0.824", "0.507", "0.611"],
         ["Residual alone", "—", "0.825", "0.506", "0.611"],
         ["Raw alone", "—", "0.735", "0.238", "0.397"]],
        widths=[3.1, 2.7, 1.9, 2.1, 2.2], wide=True, align_right=[1, 2, 3, 4])
    b.figure(
        "fig4_fusion_value.png",
        "Fusion Weight Sweep And Per-Fault Detection | (a) ROC-AUC and PR-AUC "
        "of the log-normalised fusion across w_res ∈ [0, 1], with the adopted "
        "w_res = 0.90 marked against w_res = 1.00 (residual alone) in the "
        "inset; (b) the same 1× severity row as Table 10, by fault type.",
        wide=True)
    b.p(
        "Two of three normalisations select w_res = 1.00 — no raw contribution "
        "at all — and the log normalisation, adopted because it is the only one "
        "that keeps the raw model in the fused score, selects 0.90. This is a "
        "narrow high-residual optimum, not the broad [0.25, 0.95] plateau "
        "reported by the extension this study builds on under its hand-chosen "
        "injection; under physically consistent injection the stable region has "
        "contracted to a small neighbourhood of w_res = 1. Two independent "
        "checks confirm both models genuinely run in the deployed detector "
        "regardless of weight: structurally, every fusion rule maps onto the "
        "code path exercised by the end-to-end replay of Section 3.8, and "
        "operationally, a provoked-fault replay produced an alarm attributed to "
        "both models jointly (Section 4.4)."
    )

    b.h2("4.4. What Sustains The Margin, KTS Removed")
    b.p(
        "The extension this study builds on reported that the offline fusion "
        "margin is a property of the injection protocol: +0.189 PR-AUC by hand, "
        "−0.003 once faults were injected physically, both measured on the "
        "twelve/twenty-four-channel, wrench-carrying decomposition of that "
        "study. This paper adopts the physical protocol throughout (Section "
        "3.7) and does not re-derive that comparison; what it can report that "
        "the earlier study could not is what the same physical protocol gives "
        "once the wrench-derived channels are removed rather than kept."
    )
    b.p(
        "The result is small and normalisation-dependent rather than a return "
        "to the earlier positive margin. Under log normalisation the fused "
        "detector is within 0.002 AUC and 0.002 PR-AUC of the residual model "
        "alone (Table 9) — neither a gain nor the earlier study's clearly "
        "negative −0.003, but a value close enough to zero that two of three "
        "normalisations round it to exactly zero by selecting w_res = 1. The "
        "one place a positive contribution is unambiguous is the encoder "
        "fault (Table 10), where it follows from physics rather than from a "
        "chosen injection amplitude: the residual channel this fault would "
        "have to move through carries under 0.13 Nm of signal against several "
        "newton-metres of noise, a relationship fixed by the robot's own "
        "gravity term and not by any modelling choice made in this pipeline. "
        "Removing the wrench channel therefore does not resurrect the "
        "predecessor's fusion advantage; it replaces a margin manufactured by "
        "an unphysical injection amplitude with a smaller, fault-type-specific "
        "one that has a stated physical cause."
    )
    b.p(
        "This is not evidence that the raw model is dispensable at the level of "
        "an individual decision. During the end-to-end replay of Section 3.8, "
        "one provoked torque-pulse fault was flagged by both models exceeding "
        "their own thresholds simultaneously; a single such observation is "
        "reported as what it is, one qualitative data point from a live class, "
        "not as a statistic, and the systematic version of this question is "
        "left to the hardware commissioning of Section 4.6."
    )

    b.h2("4.5. Latency And Compute Budget")
    b.p(
        "Per sample, the inverse dynamics evaluation costs 26 µs and the "
        "Jacobian-based payload term about 96 µs, together under 0.03 % of the "
        "500 Hz budget, measured in an offline batch benchmark of the same "
        "feature code the online engine runs. Per decision, the deployed ROS 2 "
        "node — both feature extraction and both ONNX forward passes together, "
        "measured on the physical cell during the replay of Section 3.8 — costs "
        "6.05 ms, 12 % of the 50 ms budget, on CPU; a single ONNX pass of the "
        "residual model alone, benchmarked separately, is 1.82 ms. The overall "
        "figure is close in relative terms to the 12 % reported by the "
        "extension this study builds on, despite the raw model's input channels "
        "falling by a third, because the payload term of Section 3.4 adds a "
        "Jacobian evaluation the earlier feature engine did not need."
    )
    b.p(
        "The detection latency is, as before, the sum of the 50 ms filter delay "
        "and the 50 ms decision period, plus one more decision period when the "
        "two-consecutive rule is active, giving 100-150 ms end to end. A "
        "per-fault-type latency distribution, measured by the extension this "
        "study builds on from an end-to-end replay of live data through the "
        "deployed class, was not repeated for this revision and is deferred to "
        "the commissioning trial of Section 4.6, where it can be measured "
        "together with the threshold it depends on rather than separately from "
        "it."
    )

    b.h2("4.6. Commissioning On The Real Robot")
    b.p(
        "**Pending.** All results in Sections 4.1-4.5 were obtained on the "
        "recorded dataset. The extension this study builds on closed with "
        "exactly this step — commissioning on the physical cell, re-measuring "
        "the fused threshold there, and reporting what the deployed detector "
        "did against provoked, operator-confirmed events — and Section 3.8 "
        "reports that the online feature engine has already been re-verified "
        "against the offline pipeline in preparation for it. The trial itself, "
        "across the four production tasks of Section 3.1, had not yet been run "
        "at the time of writing and is reported as a self-contained addendum "
        "once it has. Two questions the offline sections above could not settle "
        "are deferred to it specifically: whether the offline threshold "
        "transfers any better than the predecessor's did, now that the "
        "correction of Section 3.4 is conditioned on the running task, and "
        "whether the small, encoder-specific fusion contribution of Section 4.4 "
        "is visible on a genuine fault rather than only on an injected one."
    )

    # ────────────────────────────────────────── 5. Discussion ────────
    b.h1("5. Discussion")

    b.h2("5.1. What Removing The Force/Torque Channel Changed")
    b.p(
        "The extension this study builds on found that its measured fusion "
        "advantage was manufactured by a hand-chosen injection amplitude rather "
        "than by genuine complementarity, and traced the mechanism to the "
        "extrinsic residual's linear sensitivity to the wrench. This study asked "
        "the next question the finding raises — is the wrench channel itself "
        "trustworthy — and found that it is not: it is a controller estimate, "
        "not a measurement, and its influence on the residual was seven times "
        "larger between tasks than the influence of every other source of "
        "disturbance combined. Removing it is therefore not offered as a "
        "simplification but as the correction the predecessor's own finding "
        "points to."
    )
    b.p(
        "What replaces the removed channel is not nothing. The payload term of "
        "Section 3.4 restores, outside the validated solver and per task rather "
        "than globally, the one physical effect a wrench channel could "
        "legitimately have carried — a sustained load at the flange — while "
        "leaving out its transient, contact-detecting role entirely; Section 5.6 "
        "returns to what that trade costs. On the data available, the "
        "replacement generalises where a simpler one did not: a single global "
        "offset, tried first, reduced training-session residual spread but "
        "reversed sign on at least one task's held-out sessions (Section 3.4), "
        "while the task-conditioned payload term reduces spread on every split "
        "for every joint (Table 5)."
    )

    b.h2("5.2. The Fusion Margin Under A KTS-Free Residual")
    b.p(
        "The central offline finding of the extension this study builds on was "
        "negative and concerned its own evaluation: a fusion margin of "
        "+0.189 PR-AUC under hand-chosen injection collapsed to −0.003 once the "
        "same faults were injected physically. This study does not re-derive "
        "that comparison — it adopts the physical protocol throughout, as the "
        "predecessor's finding recommends — and asks instead what the physical "
        "protocol gives once the decomposition it operates on no longer includes "
        "a wrench term. The answer is a second, smaller negative result rather "
        "than a reversal: in aggregate the fused detector is statistically "
        "indistinguishable from the residual model alone (Table 9), and two of "
        "three normalisation schemes select a fusion weight of exactly 1.0 — no "
        "raw contribution — when left to choose freely (Table 11)."
    )
    b.p(
        "Where a contribution is measurable, it has a stated physical cause "
        "rather than a chosen injection amplitude. The encoder-step fault moves "
        "the residual channels by under 0.13 Nm, a figure computed from the same "
        "inverse dynamics solver rather than assumed, against a multi-newton-"
        "metre residual noise floor; the residual model's blindness to this "
        "fault type is therefore a property of the robot's own gravity term at "
        "that joint, not of the fit. The raw model's advantage there is real "
        "(Table 10) but itself modest, which is the honest reading of why the "
        "measured weight settles at 0.90 rather than lower: enough to keep a "
        "small, physically grounded contribution, not enough to let it cost "
        "much on the three fault types it does not help with."
    )

    b.h2("5.3. Why A Log-Domain Normalisation Was Adopted Before Testing On Hardware")
    b.p(
        "The predecessor's commissioning traced its threshold-transfer failure "
        "to a specific mechanism: a min–max bound fitted on clean validation "
        "windows can be set by whichever window reconstructed worst, and one "
        "atypical window then compresses the entire ordinary range to "
        "invisibility once the deployment distribution shifts even slightly — "
        "exactly what an unmodelled payload plateau did to that study's "
        "residual span. This mechanism is a property of an affine transform "
        "applied to a heavy-tailed distribution, not of any one dataset, and it "
        "does not depend on the wrench channel that study's payload plateau "
        "happened to involve. The log-domain normalisation of Section 3.6 was "
        "adopted for exactly this reason, before rather than after a "
        "commissioning failure repeated it: standardising log₁₀(score) maps the "
        "bulk of a heavy-tailed distribution to a bounded range and lets the "
        "tail extend rather than dominate it. Whether this is sufficient — "
        "a payload swing an order of magnitude larger than any single training "
        "session, or a task the calibration run did not cover, could still "
        "defeat it — is exactly what Section 4.6 will measure, and this section "
        "records the reasoning in advance so that a favourable or unfavourable "
        "result on hardware can be attributed to a stated hypothesis rather "
        "than assessed after the fact."
    )

    b.h2("5.4. What The Task-Conditioned Correction Bought")
    b.p(
        "Adding friction and a task-conditioned payload term outside the solver "
        "reduces total residual spread by 42-48 % on validation and test "
        "sessions the fit never saw, on every one of six joints (Table 5). That "
        "is measured generalisation, not a training-set fit: the same "
        "correction, restricted to a single global offset, reduced spread on "
        "training sessions comparably well but reversed sign on held-out "
        "sessions of at least one task (Section 3.4), which the task-conditioned "
        "version does not."
    )
    b.p(
        "Whether the correction also changes detection of the four synthetic "
        "faults of Section 3.7, as opposed to the shape of the residual it is "
        "computed from, was not re-tested against a no-correction baseline this "
        "round — the extension this study builds on ran exactly that ablation "
        "for its own friction term and found a physical-fidelity gain with no "
        "significant detection gain under physical injection. Repeating it for "
        "the combined friction-and-payload term is listed among the items "
        "outstanding in Section 5.6 rather than assumed to repeat."
    )

    b.h2("5.5. Limitations Carried Over From The Force/Torque Removal")
    b.p(
        "Removing the wrench channel removes a real capability along with the "
        "unreliable one: whatever transient, contact-shaped signal a working "
        "force/torque sensor would have supplied is now not available to either "
        "model at any weight, at exactly the fault type — collision — where the "
        "predecessor's hardware commissioning found both models saturating "
        "identically and therefore uninformative about which channel actually "
        "carried the detection. The collision scenario of Section 3.7 is "
        "injected directly into joint torque for the same reason, and is "
        "correspondingly a weaker stand-in for a real contact event, propagated "
        "through no Jacobian and referenced to no contact point, than the "
        "wrench-based injection it replaces. This is a genuine reduction in "
        "scope, accepted because the channel it removes was not doing what it "
        "was assumed to do, not because the capability was unneeded."
    )

    b.h2("5.6. Limitations")
    for t in [
        "A single robot type (UR10e), now across four task profiles rather than "
        "one; no validation on other robot types has been performed.",
        "Synthetic faults, even injected physically, remain analytic "
        "perturbations, and the collision scenario is a weaker stand-in for a "
        "real contact event than before, now that it is injected directly into "
        "joint torque rather than propagated from an assumed contact point "
        "through the Jacobian (Section 5.5).",
        "The current-to-torque coefficient could be measured directly on only "
        "two of six joints by the RTDE method, the same two as by the earlier "
        "quasi-static method; the other four are derived by assumption, which "
        "affects the physical interpretation rather than the detection.",
        "The friction and payload coefficients are fitted jointly by a "
        "regression that absorbs any velocity- and pose-correlated model error, "
        "not friction or payload alone; the fitted centre-of-gravity offset is "
        "not separately identifiable at the small extra-mass magnitudes found "
        "here (Table 4) and is not reported as measured. No independent "
        "tribological or mass validation was performed.",
        "Whether the task-conditioned correction changes detection of the four "
        "synthetic faults, as opposed to the residual's held-out spread, was not "
        "re-tested this round (Section 5.4).",
        "A single training seed is reported; the five-seed sensitivity study of "
        "the extension this study builds on, which showed the ranking metrics "
        "stable to within 0.01 across seeds and the operating threshold varying "
        "by up to 46 % of its mean, was not repeated.",
        "Classical baselines (Isolation Forest, One-Class SVM, a residual-norm "
        "threshold) and the architecture and fusion-weight-sweep figures of the "
        "extension this study builds on were not regenerated for this revision.",
        "Hardware commissioning, and everything Sections 4.6, 5.2 and 5.3 state "
        "as expectation rather than measurement, is outstanding at the time of "
        "writing.",
    ]:
        pp = b.para("", style="Paragraf", align=WD_ALIGN_PARAGRAPH.JUSTIFY,
                    space_before=3, space_after=0)
        pp.paragraph_format.left_indent = Cm(0.4)
        b.rich(pp, "•  " + t)
    b.p("")

    b.h1("6. Conclusions")
    b.p(
        "This paper re-examined the residual definition of a score-level fusion "
        "framework for cobot anomaly detection, previously extended to a "
        "physical UR10e cell across a single task, and found that the channel "
        "the extrinsic half of its residual depended on is not a physical "
        "measurement. Every measurement taken on the way is reported, including "
        "the ones unfavourable to the resulting design."
    )
    b.p(
        "The finding is traceable to the driver source and confirmed "
        "quantitatively: the ROS 2 hardware interface populates the wrench "
        "topic from the controller's own payload-compensated force estimate, "
        "and the field that would carry a genuine strain-gauge signal reads a "
        "constant, implausible offset. Across four production tasks now "
        "recorded on the same cell, the resulting task-dependent noise in a "
        "wrench-derived residual (20.9 Nm between tasks) is roughly seven "
        "times larger than the total residual's own spread once the channel is "
        "removed (3.1 Nm). The twelve-channel intrinsic/extrinsic split was "
        "withdrawn on this evidence, replaced by a single six-channel total "
        "residual, and the raw model's channels were reduced from twenty-four "
        "to sixteen — removing the same wrench channels and, on separate "
        "empirical evidence, two joint-position channels a wrap-around "
        "encoding was shown not to fix."
    )
    b.p(
        "In place of the withdrawn split, the missing payload term was added "
        "outside the validated solver, conditioned on which of four production "
        "tasks is running, alongside the friction term of the predecessor. "
        "Fitted on 3.26 million training samples and evaluated on sessions it "
        "never saw, it reduces total residual spread by 42-48 % on every joint, "
        "where a single global offset tried first reduced training-session "
        "spread comparably but reversed sign on held-out sessions of at least "
        "one task."
    )
    b.p(
        "Under fault injection restricted to the measured channels — the "
        "protocol the predecessor's own principal finding established as the "
        "only meaningful one — the fusion weight is now measured rather than "
        "assumed and converges to a narrow high-residual optimum (0.90-1.00 "
        "across three normalisations), an order narrower than the broad "
        "plateau reported before. In aggregate the fused detector no longer "
        "exceeds the residual model alone (Best F1 0.609 against 0.611); the "
        "one measurable, physically grounded exception is an encoder fault "
        "the residual model is blind to by construction, where a 0.3 rad step "
        "moves the modelled gravity torque by under 0.13 Nm against a "
        "multi-newton-metre noise floor. Removing the force/torque channel "
        "therefore does not restore the predecessor's offline fusion "
        "advantage; it replaces a margin manufactured by an unphysical "
        "injection amplitude with a smaller one that has a stated physical "
        "cause."
    )
    b.p(
        "The system was re-implemented as a ROS 2 node without the removed "
        "channel, and its feature engine was re-verified against the offline "
        "pipeline (largest observed deviation 1.5·10⁻⁶ Nm in the residual). On "
        "a replay of the physical cell it drew an alarm from both models "
        "simultaneously on a provoked fault, one qualitative data point rather "
        "than a statistic. Commissioning the detector on hardware — "
        "re-measuring the threshold there, as the predecessor found necessary, "
        "and reporting what it catches and what it misses across four "
        "production tasks rather than one — is the step this paper's own "
        "predecessor closed with and this revision reopens; it is reported "
        "separately once complete, together with the per-fault-type latency "
        "distribution and detection-ablation measurements listed as "
        "outstanding in Section 5.6."
    )
    b.p(
        "Two directions follow, one carried over unchanged and one specific to "
        "this revision. Real-time detection of low-amplitude contact remains an "
        "open problem the predecessor identified and this study did not "
        "address, and closing it plausibly needs either a model trained on the "
        "cell's own trajectories or a genuine force/torque channel rather than "
        "the withdrawn one — a fitted correction outside the solver, however "
        "well it generalises, is not a substitute for a transducer where a "
        "transducer is actually required. Specific to this revision: whether a "
        "detector whose residual definition changed in kind, not only in "
        "coefficients, reproduces the predecessor's hardware finding that the "
        "offline threshold does not transfer, or whether the task-conditioned "
        "correction narrows that gap, is a question this paper has posed and "
        "not yet answered."
    )

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
