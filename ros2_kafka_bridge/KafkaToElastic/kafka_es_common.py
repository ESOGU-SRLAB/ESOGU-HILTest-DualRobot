#!/usr/bin/env python3
"""Ortak Kafka -> Elasticsearch toplu yazma dongusu.

NEDEN VAR: 4 Eyl 2026'da canli hucrede olculdu. Tek tek yazan surumler
(`es.index()` + mesaj basina `consumer.commit()` + mesaj basina `print()`)
114 Hz'de tavan yapiyordu; ayni anda kopru topic basina 495 Hz uretiyordu.
Aradaki 381 msg/s'lik acik her dakika ~46 saniye daha gecikme ekliyordu ve
lag 419.000'e ulasmisti. Zaten toplu yazan KafkatoElastic_JointStates.py ise
ayni yukte 524 Hz ile rahat yetisiyordu. Bu modul o deseni tekillestirir.

run_all.sh yalnizca KafkatoElastic_*.py dosyalarini baslatir; bu dosyanin adi
kasten o kalibin disinda, yoksa modul ayri bir surec olarak da calisirdi.
"""

import json
import math
import time
from datetime import datetime, timezone

from kafka import KafkaConsumer
from elasticsearch import Elasticsearch, helpers


def clean_data(data):
    """NaN/inf degerleri temizler (Elasticsearch bunlari reddeder).

    Kopru json.dumps() kullaniyor ve Python NaN'i `NaN` diye yaziyor - gecerli
    JSON degil ama json.loads() geri okuyor. Temizlenmezse tum toplu yazma
    istegi 400 ile doner, yani tek bozuk deger 500 belgeyi birden dusurur.
    """
    if isinstance(data, dict):
        out = {}
        for k, v in data.items():
            cv = clean_data(v)
            if cv is not None:
                out[k] = cv
        return out
    if isinstance(data, list):
        return [c for c in (clean_data(v) for v in data) if c is not None]
    if isinstance(data, float):
        if math.isnan(data) or math.isinf(data):
            return None
    return data


def _ros_epoch(data):
    """ROS zaman damgasini epoch saniye olarak bulur; bulamazsa None.

    Kopru topic'e gore UC AYRI sekil uretiyor, hepsini desteklemek sart:
      duz    : data["header.stamp.sec"]            (dynamic/sim joint states)
      ic ice : data["header"]["sec"]               (tcp_pose, wrench, tool_data)
      ic ice : data["header"]["stamp"]["sec"]      (bazi eski belgeler)
    """
    if not isinstance(data, dict):
        return None
    sec = data.get("header.stamp.sec")
    nsec = data.get("header.stamp.nanosec")
    if sec is None:
        h = data.get("header")
        if isinstance(h, dict):
            st = h.get("stamp")
            if isinstance(st, dict):
                sec, nsec = st.get("sec"), st.get("nanosec")
            else:
                sec, nsec = h.get("sec"), h.get("nanosec")
    if not isinstance(sec, (int, float)) or sec <= 0:
        return None
    if not isinstance(nsec, (int, float)):
        nsec = 0
    return sec + nsec / 1e9


def add_timestamp(data):
    """@timestamp ekler. HER ZAMAN UTC.

    KafkatoElastic_Sim_JointStates.py burada datetime.fromtimestamp() (yerel
    saat) kullaniyordu. Elasticsearch saat dilimsiz ISO metnini UTC sayar, bu
    yuzden sim belgeleri UTC+3'te 3 saat GELECEGE yaziliyordu ve arayuzdeki
    "son N dakika" sorgusu onlari hic gormuyordu - sim panellerinin bos
    gorunmesinin sebebi buydu. utcfromtimestamp yerine timezone.utc kullanip
    sonda 'Z' ile yaziyoruz, boylece niyet belgede de acik duruyor.
    """
    ts = _ros_epoch(data)
    dt = (datetime.fromtimestamp(ts, tz=timezone.utc) if ts is not None
          else datetime.now(timezone.utc))
    data["@timestamp"] = dt.isoformat().replace("+00:00", "Z")
    return data


def run(topic, index, group_id,
        auto_offset_reset="earliest",
        batch_size=500,
        flush_seconds=0.5,
        report_seconds=30.0,
        should_index=None,
        index_for=None,
        bootstrap="localhost:9092",
        es_url="http://localhost:9200"):
    """Bir Kafka topic'ini bir Elasticsearch indeksine toplu halde aktarir.

    should_index(data) -> bool   : False donerse belge atlanir
    index_for(data)    -> str    : indeks adini belgeye gore secer
    """
    consumer = KafkaConsumer(
        topic,
        bootstrap_servers=bootstrap,
        group_id=group_id,
        auto_offset_reset=auto_offset_reset,
        # Otomatik commit KAPALI: offset'i ancak belgeler Elasticsearch'e
        # yazildiktan SONRA isliyoruz. Otomatikte, yazma basarisiz olsa bile
        # offset ilerleyebilir ve o mesajlar sessizce kaybolur.
        enable_auto_commit=False,
        value_deserializer=lambda m: json.loads(m.decode("utf-8")),
        max_poll_records=batch_size,
    )
    es = Elasticsearch(es_url)
    if not es.ping():
        print(f"❌ Elasticsearch'e baglanilamadi: {es_url}")
        return

    print(f"🚀 {topic} -> {index}  (grup {group_id}, {batch_size}'lik toplu yazma)", flush=True)

    batch = []
    last_flush = time.time()
    last_report = time.time()
    total = skipped = failed = 0
    since_report = 0

    def flush():
        nonlocal batch, last_flush, total, failed, since_report
        if not batch:
            return
        try:
            ok, errs = helpers.bulk(es, batch, stats_only=True, raise_on_error=False)
            consumer.commit()
            total += ok
            since_report += ok
            failed += errs
        except Exception as e:
            failed += len(batch)
            print(f"❌ Toplu yazma hatasi: {e}", flush=True)
        batch = []
        last_flush = time.time()

    try:
        while True:
            # poll(), iterator'un aksine mesaj gelmese de geri doner. Onemli:
            # zamanli flush'i mesaj dongusunun ICINE koyan surumlerde hucre
            # susunca eldeki yarim yigin yazilmadan bekliyordu, yani bir
            # kosunun son belgeleri ancak sonraki kosuda gorunuyordu.
            records = consumer.poll(timeout_ms=200, max_records=batch_size)
            for _tp, msgs in records.items():
                for msg in msgs:
                    data = msg.value
                    if not isinstance(data, dict) or not data:
                        skipped += 1
                        continue
                    if should_index is not None and not should_index(data):
                        skipped += 1
                        continue
                    doc = clean_data(data)
                    if not doc:
                        skipped += 1
                        continue
                    doc = add_timestamp(doc)
                    doc["kafka_metadata"] = {
                        "topic": msg.topic,
                        "partition": msg.partition,
                        "offset": msg.offset,
                        "timestamp": msg.timestamp,
                    }
                    batch.append({
                        "_index": index_for(doc) if index_for else index,
                        "_source": doc,
                    })

            now = time.time()
            if len(batch) >= batch_size or (batch and now - last_flush >= flush_seconds):
                flush()

            # Mesaj basina degil, periyodik tek satir rapor.
            if now - last_report >= report_seconds:
                hz = since_report / (now - last_report)
                print(f"📊 {index}: {hz:.0f} Hz | toplam {total} | "
                      f"atlanan {skipped} | hatali {failed} | yigin {len(batch)}", flush=True)
                since_report = 0
                last_report = now
    except KeyboardInterrupt:
        print("\n🛑 Durduruluyor, eldeki yigin yaziliyor...")
    finally:
        flush()
        print(f"🎯 {index}: toplam {total} belge, {skipped} atlandi, {failed} hatali")
        try:
            consumer.close()
        except Exception:
            pass
