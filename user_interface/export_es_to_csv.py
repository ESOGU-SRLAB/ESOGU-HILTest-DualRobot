#!/usr/bin/env python3
"""Dump every Elasticsearch index to its own CSV file. READ-ONLY.

Her indeks (topic) ayri bir CSV dosyasina yazilir:

    es_export_20261007_141530/
        ros-joint-states.csv
        ros-sim-joint-states.csv
        ur-rtde-data.csv
        ...
        manifest.json          <- indeks basina satir sayisi, sutunlar, sure

VERI GUVENLIGI
--------------
Bu script Elasticsearch'e HICBIR SEY yazmaz ve HICBIR SEYI SILMEZ. Gonderdigi
her istek `_request()` icindeki beyaz listeden gecer; listede yalnizca okuma
uclari var:

    GET  _cat/indices          indeks listesi
    GET  <index>/_count        belge sayisi (dogrulama icin)
    GET  <index>/_mapping      sutun adlari
    POST <index>/_search       ilk scroll sayfasi
    POST _search/scroll        sonraki sayfalar
    DELETE _search/scroll      <- ASAGIYA BAKIN

Tek DELETE, `_search/scroll` ucuna gider. Bu cagri bir scroll'un ACIK ARAMA
BAGLAMINI kapatir, yani sunucudaki gecici okuma imlecini birakir; belge, alan
veya indeks silmez. Beyaz liste baska hicbir DELETE'e izin vermez: bir indekse
ya da `_delete_by_query`ye gonderilen bir DELETE, istek gonderilmeden exception
ile durur. Cagri basarisiz olsa bile zarari yoktur, scroll kendi `keep_alive`
suresi dolunca kendiliginden dusurulur.

Dosya tarafinda da uzerine yazmaz: her CSV once `.part` olarak yazilir ve ancak
tamamlandiginda asil adina tasinir. Yarida kesilen bir calisma `.part` olarak
kalir, tamamlanmis bir dosya gibi gorunmez. Zaten var olan bir CSV atlanir
(`--overwrite` verilmedikce).

KULLANIM
--------
    python3 export_es_to_csv.py                      # hepsi, ./es_export_<tarih>/ altina
    python3 export_es_to_csv.py --out /mnt/disk2/es_export
    python3 export_es_to_csv.py --gzip               # .csv.gz olarak yaz (disk icin)
    python3 export_es_to_csv.py --only ur-rtde-data ros-joint-states
    python3 export_es_to_csv.py --exclude ros_sim_image
    python3 export_es_to_csv.py --expand-arrays      # dizileri ayri sutunlara ac
    python3 export_es_to_csv.py --yes                # onay sormadan basla

DIZILER
-------
Varsayilan olarak dizi degerler tek bir sutuna JSON olarak yazilir:

    data.actual_q = "[1.28, -1.61, -1.67, 0.53, 0.76, 1.17]"

`--expand-arrays` ile her eleman kendi sutununa acilir (data.actual_q.0 ...).
Sutun sayisi indeksin ilk `--sample` belgesine bakilarak belirlenir; daha uzun
bir dizi cikarsa tasan kisim `_overflow_json` sutununa yazilir ve sonunda uyari
verilir, yani bu modda da veri kaybolmaz.

ZAMAN FILTRESI YOK
------------------
Sorgu `match_all`: indeksteki her belge yazilir. Arayuzun aksine burada
`@timestamp` var mi diye bakilmaz, dolayisiyla eski belgeler de gelir.
"""

import argparse
import csv
import datetime
import gzip
import json
import os
import re
import sys
import time

try:
    import requests
except ImportError:                                   # pragma: no cover
    sys.exit("requests gerekli:  pip install requests")


# ------------------------------------------------------------------------------
# HTTP katmani -- okuma disindaki her sey burada durur
# ------------------------------------------------------------------------------

# (method, path deseni). Path'ler ES koku altinda, bas taraftaki / olmadan.
# Bir indeks adi / icermez, bu yuzden [^/]+ bir indeksi tam olarak karsilar.
_ALLOWED = (
    ("GET", re.compile(r"^_cat/indices$")),
    ("GET", re.compile(r"^[^/?]+/_count$")),
    ("GET", re.compile(r"^[^/?]+/_mapping$")),
    ("POST", re.compile(r"^[^/?]+/_search$")),
    ("POST", re.compile(r"^_search/scroll$")),
    # Yalnizca scroll baglamini kapatir. Belge silmez. Aciklama icin dosya
    # basligindaki VERI GUVENLIGI bolumune bakin.
    ("DELETE", re.compile(r"^_search/scroll$")),
)


class NotAllowed(RuntimeError):
    """Beyaz listede olmayan bir istek denendi -- istek gonderilmez."""


def _check(method, path):
    for m, rx in _ALLOWED:
        if m == method and rx.match(path):
            return
    raise NotAllowed(f"bu script yalnizca okuma yapar; reddedildi: {method} /{path}")


def _request(session, base, method, path, params=None, body=None, retries=3):
    _check(method, path)
    url = f"{base.rstrip('/')}/{path}"
    last = None
    for attempt in range(1, retries + 1):
        try:
            r = session.request(method, url, params=params, json=body, timeout=300)
            if r.status_code >= 400:
                raise RuntimeError(f"{method} /{path} -> HTTP {r.status_code}: {r.text[:300]}")
            return r.json()
        except (requests.ConnectionError, requests.Timeout) as e:
            last = e
            if attempt == retries:
                break
            wait = 2 ** attempt
            print(f"  ! baglanti hatasi ({e.__class__.__name__}), {wait} sn sonra tekrar "
                  f"({attempt}/{retries})", flush=True)
            time.sleep(wait)
    raise RuntimeError(f"{method} /{path} basarisiz: {last}")


# ------------------------------------------------------------------------------
# Mapping -> sutun adlari
# ------------------------------------------------------------------------------

def mapping_leaves(props, prefix=""):
    """Mapping agacindaki yaprak alanlarin noktali yollari.

    `.keyword` gibi alt alanlar (multi-field) atlanir: ayni degerin ikinci bir
    kopyasi, CSV'de ayri bir sutunu hak etmiyor.
    """
    out = []
    for name, spec in (props or {}).items():
        path = f"{prefix}{name}"
        sub = spec.get("properties")
        if isinstance(sub, dict):
            out.extend(mapping_leaves(sub, path + "."))
        else:
            out.append(path)
    return out


def flatten(src, prefix=""):
    """_source'u noktali anahtarlara duzlestirir. Diziler oldugu gibi kalir."""
    out = {}
    for key, val in src.items():
        path = f"{prefix}{key}"
        if isinstance(val, dict):
            out.update(flatten(val, path + "."))
        else:
            out[path] = val
    return out


def cell(val):
    """Tek bir hucrenin metin hali."""
    if val is None:
        return ""
    if isinstance(val, bool):
        return "true" if val else "false"
    if isinstance(val, (list, dict)):
        return json.dumps(val, ensure_ascii=False)
    return str(val)


# ------------------------------------------------------------------------------
# Indeks disa aktarimi
# ------------------------------------------------------------------------------

def sample_array_lengths(session, base, index, sample):
    """--expand-arrays icin: alan basina gorulen en uzun dizi uzunlugu."""
    res = _request(session, base, "POST", f"{index}/_search",
                   body={"size": sample, "query": {"match_all": {}}})
    lengths = {}
    for hit in res.get("hits", {}).get("hits", []):
        for path, val in flatten(hit.get("_source", {})).items():
            if isinstance(val, list) and val and not isinstance(val[0], (dict, list)):
                lengths[path] = max(lengths.get(path, 0), len(val))
    return lengths


def export_index(session, base, index, out_dir, args):
    """Tek bir indeksi CSV'ye yazar. Dondurdugu sozluk manifest'e girer."""
    suffix = ".csv.gz" if args.gzip else ".csv"
    final = os.path.join(out_dir, index + suffix)
    part = final + ".part"

    if os.path.exists(final) and not args.overwrite:
        print(f"= {index}: zaten var, atlaniyor ({os.path.basename(final)})")
        return {"index": index, "status": "skipped-existing", "file": final}

    expected = _request(session, base, "GET", f"{index}/_count").get("count", 0)
    mapping = _request(session, base, "GET", f"{index}/_mapping")
    props = list(mapping.values())[0].get("mappings", {}).get("properties", {})
    leaves = sorted(mapping_leaves(props))

    array_lengths = {}
    if args.expand_arrays:
        array_lengths = sample_array_lengths(session, base, index, args.sample)

    # Sutunlar: once kimlik, sonra mapping'den gelen alanlar, sonra guvenlik agi
    # olarak iki JSON sutunu. Mapping'de olmayan bir alan (_unmapped_json) ya da
    # ornekten uzun bir dizi (_overflow_json) sessizce dusmez, oraya yazilir.
    columns = ["_index", "_id"]
    for leaf in leaves:
        n = array_lengths.get(leaf)
        if n:
            columns.extend(f"{leaf}.{i}" for i in range(n))
        else:
            columns.append(leaf)
    columns += ["_unmapped_json", "_overflow_json"]
    known = set(leaves)

    print(f"→ {index}: {expected:,} belge, {len(columns)} sutun -> "
          f"{os.path.basename(part)}", flush=True)

    opener = (lambda p: gzip.open(p, "wt", newline="", encoding="utf-8")) if args.gzip \
        else (lambda p: open(p, "w", newline="", encoding="utf-8"))

    rows = 0
    unmapped_seen = set()
    overflow_seen = set()
    started = time.time()
    scroll_id = None

    try:
        with opener(part) as fh:
            writer = csv.DictWriter(fh, fieldnames=columns, extrasaction="ignore")
            writer.writeheader()

            res = _request(session, base, "POST", f"{index}/_search",
                           params={"scroll": args.keep_alive},
                           body={"size": args.batch, "query": {"match_all": {}},
                                 "sort": ["_doc"]})
            while True:
                scroll_id = res.get("_scroll_id")
                hits = res.get("hits", {}).get("hits", [])
                if not hits:
                    break

                for hit in hits:
                    flat = flatten(hit.get("_source", {}))
                    row = {"_index": hit.get("_index"), "_id": hit.get("_id")}
                    unmapped, overflow = {}, {}

                    for path, val in flat.items():
                        n = array_lengths.get(path)
                        if n and isinstance(val, list):
                            for i, item in enumerate(val):
                                if i < n:
                                    row[f"{path}.{i}"] = cell(item)
                                else:
                                    overflow.setdefault(path, []).append(item)
                                    overflow_seen.add(path)
                            continue
                        if path in known:
                            row[path] = cell(val)
                        else:
                            unmapped[path] = val
                            unmapped_seen.add(path)

                    row["_unmapped_json"] = json.dumps(unmapped, ensure_ascii=False) if unmapped else ""
                    row["_overflow_json"] = json.dumps(overflow, ensure_ascii=False) if overflow else ""
                    writer.writerow(row)
                    rows += 1

                if rows % args.progress < args.batch:
                    el = time.time() - started
                    rate = rows / el if el else 0
                    left = (expected - rows) / rate if rate and expected > rows else 0
                    print(f"   {rows:,}/{expected:,} ({rate:,.0f} satir/sn, "
                          f"~{left/60:.1f} dk kaldi)", flush=True)

                res = _request(session, base, "POST", "_search/scroll",
                               body={"scroll": args.keep_alive, "scroll_id": scroll_id})
    finally:
        # Scroll baglamini birak. Belge silmez; ayrintili aciklama dosya
        # basligindaki VERI GUVENLIGI bolumunde.
        if scroll_id:
            try:
                _request(session, base, "DELETE", "_search/scroll",
                         body={"scroll_id": [scroll_id]}, retries=1)
            except Exception as e:
                print(f"  ! scroll baglami kapatilamadi ({e}); kendi suresinde dusecek")

    os.replace(part, final)
    took = time.time() - started
    size = os.path.getsize(final)
    status = "ok" if rows == expected else "count-mismatch"
    print(f"✓ {index}: {rows:,} satir, {size/1e6:,.1f} MB, {took/60:.1f} dk -> "
          f"{os.path.basename(final)}")
    if status == "count-mismatch":
        print(f"  ! beklenen {expected:,}, yazilan {rows:,}. Scroll bir anlik "
              f"goruntu uzerinde calisir: export sirasinda yeni veri yazildiysa "
              f"fark normaldir (yazilan < beklenen).")
    if unmapped_seen:
        print(f"  ! mapping'de olmayan {len(unmapped_seen)} alan _unmapped_json "
              f"sutununa yazildi: {sorted(unmapped_seen)[:5]}")
    if overflow_seen:
        print(f"  ! ornekten uzun diziler _overflow_json sutununa yazildi: "
              f"{sorted(overflow_seen)[:5]}")

    return {"index": index, "status": status, "file": final, "rows": rows,
            "expected": expected, "columns": len(columns), "bytes": size,
            "seconds": round(took, 1),
            "unmapped_fields": sorted(unmapped_seen),
            "overflow_fields": sorted(overflow_seen)}


# ------------------------------------------------------------------------------

def main():
    ap = argparse.ArgumentParser(
        description="Elasticsearch'teki tum veriyi indeks basina CSV'ye yazar (salt okuma).")
    ap.add_argument("--es", default=os.environ.get("ES_URL", "http://localhost:9200"))
    ap.add_argument("--out", default=None, help="cikti klasoru (varsayilan: ./es_export_<tarih>)")
    ap.add_argument("--only", nargs="+", metavar="INDEX", help="yalnizca bu indeksler")
    ap.add_argument("--exclude", nargs="+", metavar="INDEX", default=[], help="bu indeksleri atla")
    ap.add_argument("--gzip", action="store_true", help=".csv.gz olarak yaz")
    ap.add_argument("--expand-arrays", action="store_true",
                    help="dizi alanlarini ayri sutunlara ac (varsayilan: tek sutunda JSON)")
    ap.add_argument("--sample", type=int, default=500,
                    help="--expand-arrays icin ornek belge sayisi (varsayilan 500)")
    ap.add_argument("--batch", type=int, default=1000, help="scroll sayfa boyutu (varsayilan 1000)")
    ap.add_argument("--keep-alive", default="5m", help="scroll omru (varsayilan 5m)")
    ap.add_argument("--progress", type=int, default=100000, help="kac satirda bir ilerleme yazsin")
    ap.add_argument("--overwrite", action="store_true", help="var olan CSV'lerin uzerine yaz")
    ap.add_argument("--yes", action="store_true", help="onay sorma")
    args = ap.parse_args()

    print("Bu script Elasticsearch'ten yalnizca OKUR. Belge silmez, yazmaz.\n")

    session = requests.Session()
    rows = _request(session, args.es, "GET", "_cat/indices",
                    params={"format": "json", "h": "index,docs.count,store.size",
                            "bytes": "b"})

    indices = []
    for r in rows:
        name = r.get("index") or ""
        if name.startswith("."):            # Kibana'nin kendi defterleri
            continue
        if args.only and name not in args.only:
            continue
        if name in args.exclude:
            continue
        indices.append((name, int(r.get("docs.count") or 0), int(r.get("store.size") or 0)))
    indices.sort()

    if args.only:
        missing = sorted(set(args.only) - {n for n, _, _ in indices})
        if missing:
            sys.exit(f"bulunamayan indeks: {', '.join(missing)}")
    if not indices:
        sys.exit("disa aktarilacak indeks yok.")

    total_docs = sum(d for _, d, _ in indices)
    total_bytes = sum(b for _, _, b in indices)
    print(f"{len(indices)} indeks, {total_docs:,} belge, kaynak boyut {total_bytes/1e9:,.1f} GB")
    for name, docs, size in indices:
        print(f"   {name:<52} {docs:>12,} belge  {size/1e9:>7,.2f} GB")
    print("\nCSV, kaynak boyutun birkac kati yer kaplayabilir (--gzip bunu kucultur).")

    if not args.yes:
        try:
            if input("Devam edilsin mi? [e/H] ").strip().lower() not in ("e", "y", "evet", "yes"):
                sys.exit("iptal edildi.")
        except (EOFError, KeyboardInterrupt):
            sys.exit("\niptal edildi.")

    out_dir = args.out or os.path.join(
        os.getcwd(), "es_export_" + datetime.datetime.now().strftime("%Y%m%d_%H%M%S"))
    os.makedirs(out_dir, exist_ok=True)
    print(f"\ncikti klasoru: {out_dir}\n")

    results, failed = [], []
    started = time.time()
    for name, _, _ in indices:
        try:
            results.append(export_index(session, args.es, name, out_dir, args))
        except KeyboardInterrupt:
            print("\nkullanici durdurdu. Tamamlanan CSV'ler yerinde, yarim kalan .part olarak duruyor.")
            break
        except Exception as e:
            print(f"✗ {name}: {e}")
            failed.append({"index": name, "error": str(e)})

    manifest = {
        "created": datetime.datetime.now().isoformat(timespec="seconds"),
        "elasticsearch": args.es,
        "expand_arrays": bool(args.expand_arrays),
        "gzip": bool(args.gzip),
        "indices": results,
        "failed": failed,
        "seconds": round(time.time() - started, 1),
    }
    with open(os.path.join(out_dir, "manifest.json"), "w", encoding="utf-8") as fh:
        json.dump(manifest, fh, indent=2, ensure_ascii=False)

    ok = sum(1 for r in results if r.get("status") == "ok")
    print(f"\nbitti: {ok} indeks tam, {len(results) - ok} indeks uyarili, "
          f"{len(failed)} hata. Ozet: {os.path.join(out_dir, 'manifest.json')}")
    sys.exit(1 if failed else 0)


if __name__ == "__main__":
    main()
