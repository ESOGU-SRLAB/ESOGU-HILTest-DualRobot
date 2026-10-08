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
        Numaralar 1'den başlayarak sırayla verilir (Dosya.pdf yazım kuralı:
        "Denklemler baştan itibaren 1'den başlayarak numaralandırılmalıdır").
        06.10.2026'da ara denklemler "2a"/"2b" gibi alt harfli etiketlerle
        eklenmişti; bu kurala aykırıydı ve 08.10.2026'da düz sıraya (1,2,3,...)
        çevrildi. `label` yalnız sayaçtan bağımsız, elle bir numara basmak
        gerekirse kullanılsın (normalde kullanılmamalı) - verilirse sayaç
        ilerlemez, metindeki her "Equation (N)" atfını elle kontrol et.
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
TITLE_EN = "RESIDUAL AND RAW LSTM AUTOENCODER FUSION FOR ANOMALY DETECTION ON A UR10E COBOT"
TITLE_TR = "UR10E KOBOTUNDA ANOMALİ TESPİTİ İÇİN KALINTI VE HAM LSTM ÖZKODLAYICI BİRLEŞİMİ"

KEYWORDS_EN = ["Anomaly detection", "Collaborative robots", "LSTM autoencoder",
               "Channel audit", "Real-time deployment"]
KEYWORDS_TR = ["Anomali tespiti", "İşbirlikçi robotlar", "LSTM özkodlayıcı",
               "Kanal denetimi", "Gerçek zamanlı sistem"]

ABSTRACT_EN = (
    "Early detection of anomalies in collaborative robots matters for operator "
    "safety and production continuity. We report the commissioning of a "
    "physics-based residual autoencoder and a raw-signal autoencoder, fused at "
    "score level, on a UR10e cobot executing four production tasks. A channel "
    "audit showed that the force/torque signal used by the predecessor study is "
    "a controller estimate computed from joint currents under the configured "
    "payload, not a transducer reading; it was removed. The residual is "
    "redefined as a six-channel total residual. A payload correction is fitted "
    "per production task outside the validated inverse-dynamics solver; on "
    "held-out sessions it reduces total residual standard deviation by 34-63 % "
    "on validation and 44-66 % on test sessions (Table 5). Under physically "
    "consistent fault injection, fusion does not exceed the residual model alone "
    "(Best F1 0.609 against 0.611). On the physical cell, the first deployment "
    "raised 159 alarms across nine runs, of which five were confirmed and 154 "
    "were false. Most of these traced to a deployment defect: the launch "
    "argument selecting the task-specific correction defaulted to one task, so "
    "the wrong physical model was applied. With the argument mandatory and a "
    "per-task operating threshold fitted on site, the affected tasks produced "
    "no false alarms in backtest; because the thresholds were fitted on the same "
    "sessions, this is an in-sample result. All seven confirmed events remained "
    "above their thresholds, by factors of 1.1-1.6."
)

ABSTRACT_TR = (
    "İşbirlikçi robotlarda anomalilerin erken tespiti operatör güvenliği ve "
    "üretim sürekliliği açısından önemlidir. Bu çalışmada, fizik tabanlı bir "
    "kalıntı özkodlayıcısı ile ham sinyal özkodlayıcısının skor düzeyinde "
    "birleşimini, dört üretim görevi yürüten bir UR10e kobotunda devreye aldık. "
    "Kanal denetimi, önceki çalışmada kullanılan kuvvet/tork sinyalinin bir "
    "dönüştürücü ölçümü değil, yapılandırılmış yük altında eklem akımlarından "
    "hesaplanan bir kontrolcü tahmini olduğunu gösterdi; bu kanal kaldırıldı. "
    "Kalıntı altı kanallı toplam kalıntı olarak yeniden tanımlandı. Her üretim "
    "görevi için yük düzeltmesi, doğrulanmış ters dinamik çözücünün dışında "
    "ayrıca kestirildi; ayrılmış oturumlarda toplam kalıntı standart sapmasını "
    "doğrulama kümesinde %34-63, test kümesinde %44-66 azalttı (Tablo 5). "
    "Fiziksel olarak tutarlı arıza enjeksiyonu altında birleşim, tek başına "
    "kalıntı modelini geçmedi (En İyi F1 0,609'a karşılık 0,611). Fiziksel "
    "hücrede ilk devreye almada dokuz koşuda 159 alarm üretildi; bunların beşi "
    "doğrulandı, 154'ü yanlış çıktı. Bunların çoğu bir dağıtım hatasına "
    "dayanıyordu: görev düzeltmesini seçen argüman varsayılan olarak tek bir "
    "göreve ayarlıydı, bu yüzden yanlış fiziksel model uygulandı. Argüman "
    "zorunlu hâle getirilip görev başına işletme eşiği sahada kalibre edildikten "
    "sonra, etkilenen görevlerde geriye dönük testte yanlış alarm kalmadı; eşikler "
    "aynı oturumlardan kalibre edildiği için bu sonuç örneklem içidir. Doğrulanan "
    "yedi olayın tümü eşiklerinin üzerinde kaldı; marjları 1,1-1,6 kat arasındaydı."
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
    AUTHORS = [("Cem Süha Yılmaz", "1,4"), ("Serhat Kahraman", "2,4"),
               ("Metin Yılmaz", "2,4"), ("Hasan Serhan Yavuz", "1"),
               ("Uğur Yayan", "3,4")]
    p = b.para("", style="Normal", align=WD_ALIGN_PARAGRAPH.CENTER, front=True)
    for k, (name, sup) in enumerate(AUTHORS):
        style_run(p.add_run(("" if k == 0 else ", ") + name), BODY_PT)
        style_run(p.add_run(sup), BODY_PT - 3.5).font.superscript = True
    AFFIL = ["Eskişehir Osmangazi Üniversitesi, Elektrik-Elektronik Mühendisliği, Eskişehir, Türkiye",
             "Eskişehir Osmangazi Üniversitesi, Bilgisayar Mühendisliği, Eskişehir, Türkiye",
             "Eskişehir Osmangazi Üniversitesi, Yazılım Mühendisliği, Eskişehir, Türkiye",
             "ESOGÜ Akıllı Sistemler Uygulama ve Araştırma Merkezi, Otonom Sistemler ve Güvenilirlik Laboratuvarı (ESOGÜ-ASRLab), Eskişehir, Türkiye"]
    for i, text in enumerate(AFFIL, start=1):
        p = b.para("", style="Normal", align=WD_ALIGN_PARAGRAPH.CENTER, front=True)
        style_run(p.add_run(f"{i} "), SMALL_PT - 1)
        style_run(p.add_run(f"{text}. ORCID: https://orcid.org/[to be completed]"), SMALL_PT - 1)
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
        "Short-Term Memory (LSTM) autoencoder operating on residual channels "
        "derived from a Functional Mock-up Unit (FMU) inverse dynamics model was "
        "combined, at score level, with a raw LSTM autoencoder. That offline study is "
        "the first stage of this work. The present paper takes the same fusion framework "
        "online: it revises the residual definition, the score normalisation and the "
        "fault-injection protocol, and reports the online deployment on the physical cell."
    )
    b.p(
        "The present study starts from that offline framework and takes it to the "
        "physical robot. Three things are new here. First, the end-effector "
        "force/torque channel on which the earlier residual depends is not a "
        "physical measurement on this platform, and we document why (Section 3.4). "
        "Second, the residual is redefined and the payload correction is "
        "conditioned on the task. Third, the detector was run on the cell, where a "
        "deployment defect and a threshold problem were found and corrected "
        "(Section 4.6)."
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
        "framework's measured advantage is worth once it is removed. This paper "
        "reports both for one complete system: the offline consequences of "
        "removing the channel (Section 4.1) and the behaviour of the resulting "
        "detector on the physical cell (Section 4.6)."
    )
    b.p("The contributions are:")
    for item in (
        "a channel audit that identifies the force/torque signal of the offline "
        "framework as a controller estimate rather than a measurement (Section 3.4);",
        "a six-channel total residual with a task-conditioned payload correction, "
        "and a raw model reduced to sixteen channels (Sections 3.4, 3.5);",
        "a 500 Hz ROS 2 implementation checked for equivalence with the offline "
        "pipeline (Section 3.8);",
        "commissioning on the physical cell across three of four production tasks, "
        "including a launch-argument defect that dominated the first deployment, "
        "and a second, early-warning threshold (Sections 4.6, 4.7).",
    ):
        p = b.para("", style="Paragraf", align=WD_ALIGN_PARAGRAPH.JUSTIFY,
                   space_before=3, space_after=0)
        p.paragraph_format.left_indent = Cm(0.4)
        b.rich(p, "•  " + item)
    b.p(
        "The remainder of the paper is organised as follows. Section 2 reviews the "
        "related literature. Section 3 describes the platform and the data, the "
        "revised residual definition and the online implementation. Section 4 presents the findings, first offline and then on the physical cell. Section 5 discusses them and states the "
        "limitations, and Section 6 concludes."
    )

    # ──────────────────────────────────── 2. Literature review ───────
    b.h1("2. Literature Review")
    b.p(
        "Physics-based detection for manipulators is a mature field. Haddadin, De "
        "Luca and Albu-Schäffer (2017) survey collision detection, isolation and "
        "identification and establish the residual observer as the canonical tool. De Luca, Albu-Schäffer, Haddadin and Hirzinger (2006) implemented a momentum observer of the same kind on a lightweight arm. "
        "Li, Han and Wu (2020) detect collisions from a force/torque sensor at the bedplate, while Zhang, Chen and Ge (2023) predict external torque with an LSTM for collision detection on a six-joint robot. "
        "Katsampiris-Salgado et al. (2024) address collision detection for collaborative assembly on high-payload robots. The "
        "common weakness of this family is that whatever the model does not "
        "represent — friction, payload, joint elasticity — is indistinguishable from "
        "a fault, and whatever the model filters well is also filtered away when it "
        "is the fault."
    )
    b.p(
        "Data-driven detection for multivariate time series is surveyed by Darban, "
        "Webb, Pan, Aggarwal and Salehi (2024). Learning-based collision detection without an explicit observer has also been studied (Golluccio, Di Vito, Antonelli and Marino, 2025). Reconstruction-based autoencoders are the dominant design: Malhotra, Ramakrishnan, Anand, Vig, Agarwal and Shroff (2016) applied an LSTM encoder–decoder to multi-sensor time-series anomaly detection. Park, Hoshi and Kemp (2018) applied an LSTM-based variational autoencoder to anomaly detection in robot-assisted feeding. These methods need no fault "
        "labels, but they inherit whatever bias the training distribution carries."
    )
    b.p(
        "Hybrid approaches attempt to combine the two. Yang et al. (2023) learn the dynamics of collaborative-robot joints with a physics-informed network, and Križić, Musić and Kamnik (2021) estimate end-effector force and joint torque with deep learning, a learned alternative to the force/torque channel of Section 3.4. Wang et al. (2025) constrain an LSTM autoencoder with mechanism knowledge for pump operations. Huang, Chen, Deng and Huang (2024) apply graph attention and an Informer model to multivariate anomaly detection. Correia, "
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
        "with MoveIt 2; the four tasks contribute structurally different motion "
        "and load profiles to the recorded data. No force/torque sensor is fitted; Section 3.4 "
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
        "approval. All robot measurements were collected at the Intelligent "
        "Factory and Robotics Laboratory (IFARLAB), and all data analysis was "
        "carried out at the Autonomous Systems and Reliability Laboratory "
        "(ASRLab); both laboratories are part of the ESOGÜ Intelligent Systems "
        "Application and Research Centre. The principles of research and "
        "publication ethics were observed throughout."
    )

    b.h2("3.2. Data Investigation And Preparation")
    b.p(
        "The first stage of the audit was the data itself, and it produced the largest single "
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
        "Table 2 gives the outcome of the full cascade. The reduction is not a loss of information: every removed sample belongs to "
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
        "required before the residual of Equation (1) can be formed at all — the same requirement as in earlier work, but re-derived here from a different and independent source."
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
             "value; only shoulder_lift and elbow pass; no independent cross-check is reported here. target_moment σ is the standard deviation of "
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
        "The inverse dynamics model packages the Newton–Euler formulation of the UR10e "
        "in the Functional Mock-up Interface format (Blochwitz et al., 2011):"
    )
    b.equation("τ̂_model  =  M(q)·q̈ + C(q, q̇)·q̇ + g(q)")
    b.p(
        "The earlier work defined the residual through an extrinsic term, "
        "r_ext = J(q)ᵀ·F_FTS, with F_FTS taken from the end-effector wrench topic of the "
        "ROS 2 driver. The predecessor paper describes F_FTS as a force/torque sensor "
        "measurement. The analysis below shows that this does not hold on this platform, "
        "and the split is withdrawn. The residual is the total residual"
    )
    b.equation("r_tot  =  τ_meas − τ̂_model − τ̂_c(q, q̇)")
    b.p(
        "The wrench is a controller estimate. The driver fills it from the RTDE field "
        "actual_TCP_force, which the controller computes from joint currents under the "
        "configured payload. The field that would carry a strain-gauge signal, "
        "ft_raw_wrench, reads a constant offset of order 10⁴ raw units with no response "
        "distinguishable from noise, because no transducer is fitted. The between-task "
        "standard deviation of the extrinsic residual is 20.9 Nm, against 3.1 Nm for the "
        "total residual; these are different quantities, compared here only by magnitude. "
        "A configured payload that differs from the mounted mass produces a static bias "
        "for the whole session. During the campaign the configured payload was 0 kg throughout."
    )
    b.p(
        "The correction term τ̂_c is a Coulomb-plus-viscous friction term (tanh in place of "
        "sign for smoothness) plus a task-conditioned offset and payload term. The payload "
        "term follows the point-mass result, with u_task a three-vector in the flange frame. "
        "Coefficients are fitted jointly by weighted least squares on training sessions only "
        "(Table 4)."
    )
    b.equation("τ̂_c  =  F_c·tanh(q̇ / ε) + F_v·q̇  +  b_task  +  m_task·A(q) + u_task·B(q)")
    b.equation("A(q)  =  g · Jᵥ(q)ᵀ·ẑ,     B(q)  =  g · J_ω(q)ᵀ·((R(q)·u_task) × ẑ)")
    b.p(
        "A held-out comparison decided whether the payload term earns its complexity. The "
        "offset alone gives a mean corrected-to-raw spread ratio of 0.537, against 0.477 "
        "with the payload term; on pick-and-place the offset alone leaves a validation bias "
        "of +23.9 Nm on shoulder_lift, opposite in sign to its −5.1 Nm training bias. The "
        "full term was adopted (Table 5)."
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
        note="m_task is a fitted nuisance coefficient, not a mass: the two inspection tasks "
             "share identical hardware but are fitted at +0.48 kg and −0.30 kg. The "
             "configured payload was 0 kg throughout, while the mounted hardware differed by "
             "task (flange and camera; screwdriver in fastening; vacuum gripper in pick-and-place).")
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
        note="Every joint's spread falls on validation and test as well as on train, so the "
             "reduction is a generalisation, not a fitting artefact.")

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
        "The residual model uses the six channels of Section 3.4. The raw model uses "
        "sixteen of its eighteen candidate channels: the shoulder_pan and wrist_3 joint "
        "angles, which enter no gravity term, were dropped after two sessions were "
        "reconstructed poorly on out-of-range joint angles. A sin/cos encoding of the position "
        "channels, tried as an alternative, made those sessions worse and was not used."
    )
    b.p(
        "Windows have 75 % overlap (stride 25) and never span two sessions; windows "
        "touching the derivative edge margin are dropped. Table 6 reports the deployed, "
        "single-seed models; a multi-seed study was not performed (Section 5.6)."
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
             "best epoch is five: the validation loss plateau "
             "is reached almost immediately and patience 25 then exhausts itself "
             "without further gain, unlike the residual model, which keeps "
             "improving to the full 300-epoch budget and had not converged when training stopped, so its reported threshold is a budget-limited value. The thresholds are 0.565 and 0.739, set at the 97th percentile of validation errors; the values follow from the revised channel sets of Sections 3.4 and 3.5 and the data of Section 3.2, not from a refinement of the same quantity.")

    b.h2("3.6. Score-Level Fusion And Log-Domain Normalisation")
    b.p(
        "The normalised scores of the two models are combined by a weighted average,"
    )
    b.equation("S_fused  =  w_res · z_res  +  w_raw · z_raw")
    b.p(
        "with w_res + w_raw = 1. Two changes are made to the fusion of earlier work, one to how the weight is chosen and one "
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
        "reference formula did for a different reason. The log transform "
        "instead maps the bulk of the distribution to an approximately symmetric "
        "range and lets the tail extend rather than dominate it."
    )

    b.h2("3.7. Data Split, Fault Injection And Evaluation Protocol")
    b.p(
        "The 2,443 physical sessions are partitioned once, and every consumer "
        "reads the same partition: the friction and payload fit of Section 3.4, "
        "the autoencoder training of Section 3.5, and the evaluation below. The "
        "partition is by session, not by row index, for a general reason: a partition that lets a test window "
        "share a run, and therefore near-identical dynamics, with a training "
        "window overstates every downstream metric. A second requirement is added "
        "here, because it did not previously apply: task membership is recorded per row, but sessions are very unevenly distributed across tasks. Sessions are allocated per task, largest first, to whichever "
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
        "revised in two respects. First, only the physically consistent injection protocol "
        "is used: faults perturb the measured channels — joint torque or joint "
        "position — and the residual is recomputed through the pipeline itself, "
        "rather than being added by hand to each model's own representation "
        "space; the central concern, that the two protocols disagree "
        "and only the physical one is meaningful, is taken as established rather than re-derived. Second, each amplitude is swept "
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
             "limitation. The encoder step is 0.3 rad. The smaller step is a judgement without joint-specific field data; Section 4.2 reports what this choice means for the residual model's sensitivity to it.")

    b.h2("3.8. Online Implementation")
    b.p(
        "The online system differs structurally from the offline pipeline in three ways (Figure 2 shows the resulting pipeline): causality "
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
        "task-conditioned correction of Equation (3) — without the base-frame "
        "rotation and Jacobian transfer the withdrawn wrench term required. The "
        "centred derivative costs the same structural delay of 25 samples (50 ms)."
    )
    b.p(
        "Equivalence was verified rather than assumed. Recorded "
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
        "Two optional rules carry over unchanged from the offline pipeline. The "
        "first is a two-consecutive-decision rule before any alarm is raised. "
        "The second is an adaptive rule, which continuously re-estimates what "
        "counts as normal from the last 600 decisions — specifically their "
        "median plus k times the robust scale 1.4826·MAD (Leys, Ley, Klein, "
        "Bernard and Licata, 2013) — and fires only when the current score "
        "departs from that moving baseline, rather than from a fixed threshold. "
        "Its baseline is frozen while an alarm is active and released 3 s after "
        "the alarm ends, so that a long alarm is treated as a change of regime "
        "instead of being absorbed into its own baseline. The adaptive rule was "
        "disabled by default in the deployment reported here, because its "
        "premise — that the score stays low during ordinary operation "
        "regardless of where the arm is posed — may not hold for a residual "
        "that still varies with pose. Whether the task-conditioned payload term "
        "of Section 3.4 removes enough of that pose dependence to re-enable the "
        "rule safely has not been tested; the rule therefore stays off."
    )
    b.p(
        "The detector is packaged as a ROS 2 node (Macenski, Foote, Gerkey, "
        "Lalancette and Woodall, 2022) that keeps the whole computation in a "
        "ROS-independent core; the node is a thin wrapper around it, so replay "
        "tests exercise the class the node actually runs. The models are exported "
        "to ONNX and executed with ONNX Runtime. Every decision is written to a "
        "comma-separated log and every alarm to a line-buffered JSON-lines event "
        "log, each session's log tagged with the checksum of the models and "
        "configuration it ran. A session that silently ran superseded models "
        "would otherwise be indistinguishable from a correct one after the fact, "
        "so the record is designed to make the software and model versions of "
        "every run legible rather than to prevent such a mismatch."
    )
    b.p(
        "Two connection-layer measurements repeat from before and one is new. "
        "The joint state topic still carries three different name sets from "
        "three publishers, of which the UR data arrives in a seven-element set "
        "in scrambled order; the mapping is still resolved per message and "
        "cached per name set. The measured sample rate is still close to "
        "500 Hz rather than exactly on it. New in this study: because no wrench topic is subscribed to at all, the single largest source of silent failure in a wrench-based deployment — a topic name mismatch that produced no score and no error — cannot recur; removing the "
        "channel removed the failure mode along with it."
    )

    b.h2("3.9. Operator Interface")
    b.p(
        "The detector was integrated into the laboratory's existing web "
        "dashboard as a third tab, using a collector subscribes to the node's decision, alarm "
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
        "fault types of Table 8. A single seed is reported (Section 3.5); Table 9 gives the result and Figure 3 shows the curves."
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
             "norm threshold) were not part of this study; their omission is a gap, not a finding, and is listed in Section "
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
        "Table 10 breaks the result down by fault type. The residual model is close to "
        "chance on the encoder step (AUC 0.505-0.550), where the raw model is consistently "
        "better (0.514-0.636); on the other three fault types the residual model is equal to "
        "or ahead of the raw model at every severity."
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
             "0.13 Nm (computed from the same inverse dynamics solver, holding velocity and acceleration at zero; this is a quasi-static bound, and dynamic contributions, including the derivative response of the Savitzky-Golay filter to a step, are not bounded here), against a residual noise "
             "floor of 0.9-6.7 Nm measured on clean test windows — on the quasi-static bound, the fault is below the residual model's noise floor, and this is a property of the robot's gravity term rather than of the fit.")
    b.p(
        "The encoder step is the one fault type where the raw model is ahead at every "
        "severity. A 0.3 rad step on wrist_2 changes the quasi-static gravity torque by at most "
        "0.13 Nm, below the residual noise floor of 0.9-6.7 Nm measured on clean windows. The "
        "aggregate of Section 4.1 therefore hides three fault types where the residual model "
        "is near its ceiling and one where the raw model is weak (AUC at best 0.636)."
    )
    b.p(
        "Window-level complementarity is measured at the per-model operating "
        "thresholds (0.565 for the residual and 0.739 for the raw model, both at the "
        "97th percentile of validation errors) over the 35,113 test windows, of which "
        "2,808 are anomalous under the injection of Section 3.7. The residual model "
        "alone detects 48.5 % of the anomalous windows, the raw model alone 1.2 %, and "
        "both 3.0 %; the union reaches 52.7 % recall against 51.5 % for the residual "
        "model alone, at a false-positive rate of 4.5 % against 2.1 % for the residual "
        "model. Under this protocol the two representations are therefore far from "
        "complementary at the operating point: the raw model adds almost no detections "
        "that the residual model misses. The encoder-fault exception of Table 10 is a "
        "separate, threshold-free effect."
    )
    b.h2("4.3. Sensitivity To The Fusion Weight")
    b.p(
        "The weight is measured, not assumed (Section 3.6): three "
        "normalisations were swept over w_res ∈ [0, 1] in steps of 0.05 on "
        "validation windows, each paired with the sixteen-channel raw model of "
        "Section 3.5. Table 11 gives the outcome and Figure 4 shows the sweep."
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
        "narrow high-residual optimum, not a broad plateau; under physically consistent injection the stable region has "
        "contracted to a small neighbourhood of w_res = 1. Two independent "
        "checks confirm both models genuinely run in the deployed detector "
        "regardless of weight: structurally, every fusion rule maps onto the "
        "code path exercised by the end-to-end replay of Section 3.8, and "
        "operationally, a provoked-fault replay produced an alarm attributed to "
        "both models jointly (Section 4.4)."
    )

    b.h2("4.4. What Sustains The Margin, F/T Removed")
    b.p(
        "The offline fusion margin of this kind depends on the injection "
        "protocol, which is why the physical protocol of Section 3.7 is used "
        "throughout. What this study can report is what that protocol gives "
        "once the wrench-derived channels are removed rather than kept."
    )
    b.p(
        "The result is small and normalisation-dependent rather than a positive margin. Under log normalisation the fused "
        "detector is within 0.002 AUC and 0.002 PR-AUC of the residual model "
        "alone (Table 9) — neither a gain nor a loss at the level of its own uncertainty, but a value close enough to zero that two of three "
        "normalisations round it to exactly zero by selecting w_res = 1. The "
        "one place a positive contribution is unambiguous is the encoder "
        "fault (Table 10), where it follows from physics rather than from a "
        "chosen injection amplitude: the residual channel this fault would "
        "have to move through carries under 0.13 Nm of signal against several "
        "newton-metres of noise, a relationship fixed by the robot's own "
        "gravity term and not by any modelling choice made in this pipeline. "
        "Removing the wrench channel therefore does not resurrect the offline fusion advantage; it replaces a margin manufactured by "
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
        "Per sample the inverse dynamics and the payload term cost about 120 µs, "
        "under 0.03 % of the 500 Hz budget. Per decision the deployed node, including "
        "both ONNX passes, costs 6.05 ms on CPU, 12 % of the 50 ms budget, measured "
        "on the physical cell. Detection latency is the 50 ms filter delay plus one "
        "decision period, or 100-150 ms with the two-consecutive rule. A per-fault-type "
        "latency distribution was not measured and is listed in Section 5.6."
    )

    b.h2("4.6. Commissioning On The Real Robot")
    b.p(
        "The trial covered three of the four production tasks (UR10E_INSPECTION, HRC and "
        "PICKPLACE) over two sessions, on 30 September and 1 October 2026. MULTIROBOT_INSPECTION "
        "was not run."
    )
    b.p(
        "The first session raised 159 alarms across nine runs; an operator confirmed five and "
        "labelled 154 false. The cause was a deployment defect, not the threshold: every run was "
        "tagged UR10E_INSPECTION, although the campaign also covered screw-driving and "
        "pick-and-place. On seven of the nine runs the static median of the fused score lay "
        "between +1.55 and +2.41, against −0.49 on the zero-alarm run, so the body of the "
        "distribution had moved. Recalibrating the threshold on the zero-alarm run and "
        "backtesting it against all nine runs raised the alarm count from 159 to 177, which "
        "confirms that no cut can separate the two populations."
    )
    b.p(
        "The argument selecting the task correction was made mandatory, and a live check "
        "compares it with the task the dashboard broadcasts. Motor current shows the stake: "
        "during HRC screw-driving contact, shoulder and elbow current reached 4-11 A and up "
        "to 6.5 A, against under 2 A at the non-contact events of the same session. With the "
        "task corrected, the HRC regime medians matched the baseline. A residual, task-specific "
        "contact effect remained; a per-task operating threshold, fitted at the 99.99th "
        "percentile of each task's own clean on-site decisions, removed it (Table 12). The "
        "99.9th percentile was tried first and left 2 of HRC's 9 false alarms and 2 of "
        "PICKPLACE's 16 uncorrected, while separating every confirmed event."
    )
    b.table(
        "Real-Robot Commissioning, Before And After Task-Specific Thresholds.",
        ["Task", "Decisions", "Confirmed events", "Alarms, shared offline θ",
         "Alarms, task-specific θ"],
        [["First session, 30 Sep (mis-tagged, nine runs)", "43,273", "5", "159", "not applicable"],
         ["UR10E_INSPECTION", "6,819", "0", "0", "— (unchanged)"],
         ["HRC", "11,654", "0", "9", "0"],
         ["PICKPLACE", "15,232", "2", "18", "2 (both confirmed)"],
         ["MULTIROBOT_INSPECTION", "—", "—", "—", "not yet run"]],
        widths=[4.7, 2.1, 2.7, 3.3, 3.3], wide=True, align_right=[1, 2, 3],
        note="θ_task is the 99.99th percentile of the task's own clean on-site "
             "decisions (Section 3.6 quantile family), fitted separately per "
             "regime; UR10E_INSPECTION was not refitted because its shared "
             "offline threshold already produced zero alarms on site (static "
             "/ moving p99.9 on-site: 1.66 / 3.52, against 2.14 / 3.90 "
             "offline — looser, not tighter, so no correction was needed). "
             "MULTIROBOT_INSPECTION keeps its own row even though the UR's own "
             "trajectory and end effector are identical to UR10E_INSPECTION's, "
             "because the correction is conditioned on the broadcast task "
             "label, not on the arm's kinematics, and only an on-site run — "
             "not run here — can confirm that the second robot's presence on "
             "the shared deck leaves the residual distribution as it is.")
    b.p(
        "Seven alarms were confirmed across the two sessions. Six exceeded the fused threshold "
        "by a factor of 1.4-1.6 (the residual sub-model by 64-128×); the seventh, the only "
        "confirmed static-regime event, exceeded its task threshold by 11 % (4.43 against 4.0). "
        "One PICKPLACE event stayed latched for 205 s because the arm halted in the faulted "
        "configuration (Figure 5, a shorter replay of a different event, shows the same "
        "plateau). The adaptive rule stayed disabled; the per-fault-type latency distribution "
        "and the encoder-fault contribution on genuine faults were not measured."
    )
    b.figure(
        "fig5_interface.png",
        "Operator Dashboard During A Confirmed Real Event | screen capture "
        "taken from a live replay (Section 3.9) of a recorded PICKPLACE "
        "session, not from the robot directly — robot and detector are both "
        "idle during capture, and the dashboard cannot distinguish the two. "
        "The fused score (blue) crosses the moving-regime threshold (red, "
        "dashed) nine seconds before the capture and remains above it for "
        "the rest of the window because the arm halts in the faulted pose "
        "rather than recovering from it.",
        wide=True)

    b.h2("4.7. A Second Threshold: Early Warning And Automatic Pause")
    b.p(
        "The gap between the two percentiles compared in Section 4.6 — wide "
        "enough that the lower one (p99.9) still let through two of HRC's "
        "nine false alarms and two of PICKPLACE's sixteen, the higher one "
        "(p99.99) none — suggested a use for the rejected threshold rather "
        "than discarding it: a second, lower-stakes tier. Added to the deployed node after the campaign above. The warning tier has been observed live on the dashboard whenever the score crossed the p99.9 threshold; the pause tier has not yet been exercised, because no genuine anomaly state occurred during the trial. It is reported here as an implementation addition, not as a further finding."
    )
    b.p(
        "Every decision is now checked against both percentiles of Table 12's "
        "task-specific table independently. Crossing p99.9 changes nothing "
        "about the robot; it latches a separate, debounced signal that the "
        "operator dashboard surfaces as a dismissible, auto-clearing notice, "
        "distinct from the alarm banner Section 3.9 describes. Crossing "
        "p99.99 — the operating threshold Sections 3.6 and 4.6 already act "
        "on — now additionally calls the ROS 2 driver's dashboard-server "
        "pause service on the alarm's rising edge, pausing whichever program "
        "is currently loaded and running on the controller, the External "
        "Control URCap node in every task reported here."
    )
    b.p(
        "This is explicitly not the safety-rated protective stop the "
        "robot's own monitoring or a Configurable Safety Input can trigger: "
        "a UR controller does not expose that function to software, by "
        "design, so pausing the loaded program is the available substitute. "
        "It is not cosmetic — pausing also drops the reverse communication "
        "interface the ROS driver depends on, documented upstream as "
        "\"Connection to reverse interface dropped\" — but recovery is a "
        "single deliberate action (teach-pendant Play, or the equivalent "
        "dashboard call under remote control) rather than the fuller "
        "recovery an emergency stop requires. The operator who deployed "
        "this addition accepted manual resumption as the operating "
        "assumption; the feature defaults to enabled on that basis and can "
        "be disabled per launch. It has not been assessed against ISO 10218-2 or ISO/TS 15066, which would be required before any use with people in the workspace. Whether it fires at the intended rate on "
        "genuine hardware operation, rather than only in the offline "
        "backtest of Section 4.6, is accordingly listed with the other "
        "untested additions in Section 5.6."
    )

    # ────────────────────────────────────────── 5. Discussion ────────
    b.h1("5. Discussion")

    b.h2("5.1. What Removing The Force/Torque Channel Changed")
    b.p(
        "A fusion advantage measured under a hand-chosen injection amplitude can be "
        "manufactured rather than genuine. This study asked the next question that raises "
        "for the extrinsic residual: is the wrench channel itself trustworthy? It is not: "
        "it is a controller estimate, not a measurement, and its influence on the residual "
        "varied between tasks by more than the spread of the total residual itself. Removing "
        "it is therefore not offered as a simplification but as the correction that the "
        "channel analysis of Section 3.4 points to."
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

    b.h2("5.2. Why A Log-Domain Normalisation")
    b.p(
        "The log-domain normalisation was chosen from the validation-window score "
        "distribution before any run on the hardware. A min–max bound taken from clean "
        "validation windows is set by the single worst reconstruction; an atypical window "
        "can then set a bound that ordinary variation never approaches, compressing normal "
        "scores as soon as the deployment distribution shifts slightly. Standardising "
        "log₁₀(score) keeps the bulk of the heavy-tailed distribution in a bounded range and "
        "lets the tail extend rather than dominate (Section 3.6). The commissioning failure "
        "of Section 4.6 was not of this kind: a wrong task label fed a correction built for "
        "another task, shifting the body of the distribution and producing false alarms. No "
        "choice of score normalisation addresses that."
    )

    b.h2("5.6. Limitations")
    for t in [
        "A single robot type (UR10e), now across four task profiles rather than "
        "one; no validation on other robot types has been performed.",
        "Synthetic faults, even injected physically, remain analytic "
        "perturbations, and the collision scenario is a weaker stand-in for a "
        "real contact event than before, now that it is injected directly into "
        "joint torque rather than propagated from an assumed contact point "
        "through the Jacobian.",
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
        "synthetic faults, as opposed to the residual's held-out spread, was not re-tested in this study.",
        "A single training seed is reported. Seed variation of the operating threshold was not measured, and the precision of the margins reported in Section 4.6 should be read with that in mind.",
        "Classical baselines (Isolation Forest, One-Class SVM, a residual-norm threshold) were not evaluated in this study.",
        "Hardware commissioning (Section 4.6) covered three of the cell's four "
        "production tasks; MULTIROBOT_INSPECTION was not exercised. Within the "
        "three, each task's operating threshold was fitted from a single "
        "on-site session — the PICKPLACE static-regime margin it produced "
        "(11 %) is the calibration's thinnest point and a larger clean sample "
        "would plausibly widen it. The per-fault-type latency distribution "
        "and the encoder-specific fusion contribution on a genuine fault, "
        "both deferred to the trial in Sections 4.5 and 4.4, were not settled "
        "by it: no provoked fault with a known onset was run, and none of the "
        "confirmed events was an encoder-type fault. Whether the adaptive "
        "rule's pose-dependence (Section 3.8) was reduced by the "
        "task-conditioned correction was not re-evaluated; it was left "
        "disabled throughout.",
        "The early-warning notice of Section 4.7 has been observed live above p99.9; the automatic pause has not been exercised on a genuine anomaly, and its only evidence is the offline backtest of Section 4.6.",
    ]:
        pp = b.para("", style="Paragraf", align=WD_ALIGN_PARAGRAPH.JUSTIFY,
                    space_before=3, space_after=0)
        pp.paragraph_format.left_indent = Cm(0.4)
        b.rich(pp, "•  " + t)
    b.p("")

    b.h1("6. Conclusions")
    b.p(
        "This paper takes an offline residual-and-raw autoencoder fusion framework for "
        "UR10e anomaly detection to online operation, and audits the sensor channel "
        "the offline framework depended on. The force/torque signal is a controller "
        "estimate computed from joint currents under the configured payload, not a "
        "transducer reading; it was removed. Under physically consistent fault "
        "injection, fusion does not exceed the residual model alone, and the one "
        "physically explained contribution is an encoder fault that the residual model "
        "cannot resolve."
    )
    b.p(
        "On the physical cell the first deployment raised 159 alarms, 154 of them false, "
        "mostly because a launch argument applied the wrong task's correction. After "
        "that defect was removed and per-task thresholds were fitted on site, the "
        "affected tasks produced no false alarms in backtest, an in-sample result. "
        "MULTIROBOT_INSPECTION, per-fault-type latency and the encoder-fault contribution "
        "on genuine faults remain open."
    )

    b.h1("Acknowledgement")
    b.p(
        "The project is supported by the KDT Joint Undertaking (101140216) and its "
        "members, including additional funding from Vinnova (Sweden), "
        "Österreichische Forschungsförderungsgesellschaft mbH – FFG (Austria), "
        "Business Finland (Finland), Ministry of Universities and Research (Italy), "
        "FCT (Portugal) and TÜBİTAK (124N448) (Türkiye). The measurements were "
        "carried out at the Intelligent Factory and Robotics Laboratory (IFARLAB), "
        "and the data analysis at the Autonomous Systems and Reliability Laboratory "
        "(ASRLab), both within the ESOGÜ Intelligent Systems Application and "
        "Research Centre."
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
    "Blochwitz, T., Otter, M., Arnold, M., Bausch, C., Clauss, C., Elmqvist, H., "
    "… Wolf, S. (2011). The Functional Mockup Interface for tool independent "
    "exchange of simulation models. *Proceedings of the 8th International Modelica "
    "Conference*, 105–114, Dresden, Germany. https://doi.org/10.3384/ecp11063105",

    "Correia, L., Goos, J. C., Klein, P., Bäck, T. & Kononova, A. V. (2024). "
    "Online model-based anomaly detection in multivariate time series: Taxonomy, "
    "survey, research challenges and future directions. *Engineering Applications "
    "of Artificial Intelligence, 138*, 109323.",

    "Darban, Z. Z., Webb, G. I., Pan, S., Aggarwal, C. C. & Salehi, M. (2024). "
    "Deep learning for time series anomaly detection: A survey. *ACM Computing "
    "Surveys, 57*(1), 1–42.",

    "De Luca, A., Albu-Schäffer, A., Haddadin, S. & Hirzinger, G. (2006). Collision "
    "detection and safe reaction with the DLR-III lightweight manipulator arm. IEEE/RSJ "
    "International Conference on Intelligent Robots and Systems, 1623–1630.",

    "Golluccio, G., Di Vito, D., Antonelli, G. & Marino, A. (2025). Deep learning-based collision detection framework for robot tasks in clutter. *Robotica, 43*(5), 1807–1826. https://doi.org/10.1017/s0263574725000517",

    "Haddadin, S., De Luca, A. & Albu-Schäffer, A. (2017). Robot collisions: A "
    "survey on detection, isolation, and identification. *IEEE Transactions on "
    "Robotics, 33*(6), 1292–1312.",

    "Huang, X., Chen, N., Deng, Z. & Huang, S. (2024). Multivariate time series "
    "anomaly detection via dynamic graph attention network and Informer. *Applied "
    "Intelligence, 54*, 7636–7658.",

    "Katsampiris-Salgado, K., Haninger, K., Gkrizis, C., Dimitropoulos, N., Krüger, J., "
    "Michalos, G. & Makris, S. (2024). Collision detection for collaborative assembly operations "
    "on high-payload robots. *Robotics and Computer-Integrated Manufacturing, 87*, "
    "102708.",

    "Križić, S., Musić, J. & Kamnik, R. (2021). End-effector force and joint torque estimation of a 7-DoF robotic manipulator using deep learning. *Electronics, 10*(23), 2963. https://doi.org/10.3390/electronics10232963",

    "Leys, C., Ley, C., Klein, O., Bernard, P. & Licata, L. (2013). Detecting "
    "outliers: Do not use standard deviation around the mean, use absolute "
    "deviation around the median. *Journal of Experimental Social Psychology, "
    "49*(4), 764–766.",

    "Li, W., Han, Y. & Wu, J. (2020). Collision detection of robots based on a "
    "force/torque sensor at the bedplate. *IEEE/ASME Transactions on Mechatronics, 25*(5), "
    "2565–2573. https://doi.org/10.1109/tmech.2020.2995904",


    "Macenski, S., Foote, T., Gerkey, B., Lalancette, C. & Woodall, W. (2022). "
    "Robot Operating System 2: Design, architecture, and uses in the wild. "
    "*Science Robotics, 7*(66), eabm6074.",

    "Malhotra, P., Ramakrishnan, A., Anand, G., Vig, L., Agarwal, P. & Shroff, G. "
    "(2016). LSTM-based encoder-decoder for multi-sensor anomaly detection. "
    "*arXiv preprint arXiv:1607.00148*.",

    "Park, D., Hoshi, Y. & Kemp, C. C. (2018). A multimodal anomaly detector for "
    "robot-assisted feeding using an LSTM-based variational autoencoder. *IEEE "
    "Robotics and Automation Letters, 3*(3), 1544–1551.",

    "Savitzky, A. & Golay, M. J. E. (1964). Smoothing and differentiation of data "
    "by simplified least squares procedures. *Analytical Chemistry, 36*(8), "
    "1627–1639.",

    "Wang, M., Zhu, X., Zhou, G., Li, K., Wu, Q. & Fan, W. (2025). Anomaly detection in "
    "multidimensional time series for water injection pump operations based on "
    "LSTMA-AE and mechanism constraints. *Scientific Reports, 15*, article 2020. "
    "https://doi.org/10.1038/s41598-025-85436-x",

    "Yang, X., Du, Y., Li, L., Zhou, Z. & Zhang, X. (2023). Physics-informed neural network for model prediction and dynamics parameter identification of collaborative robot joints. *IEEE Robotics and Automation Letters, 8*(12), 8462–8469. https://doi.org/10.1109/lra.2023.3329620",

    "Yılmaz, C. S., Kahraman, S., Yılmaz, M., Yavuz, H. S. & Yayan, U. (2026). "
    "FMU tabanlı kalıntı ayrıştırma ve ikili LSTM özkodlayıcı birleşimi ile "
    "işbirlikçi robotlarda anomali tespiti [FMU-based residual decomposition and dual LSTM autoencoder fusion for anomaly detection in collaborative robots, in Turkish]. "
    "In 2026 34th Signal Processing and Communications Applications Conference (SIU), "
    "pp. 1–4, published 7 July 2026. Available: https://ieeexplore.ieee.org/abstract/document/11636980.",

    "Zhang, T., Chen, Y. & Ge, P. (2023). LSTM-based external torque prediction for "
    "6-DOF robot collision detection. *Journal of Mechanical Science and Technology, 37*(9), "
    "4847–4855. https://doi.org/10.1007/s12206-023-0837-3",

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
