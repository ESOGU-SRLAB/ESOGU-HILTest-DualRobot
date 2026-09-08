#!/bin/bash

for file in KafkatoElastic_*.py; do
    echo "Başlatılıyor: $file"
    python3 "$file" &  # Arka planda çalıştır
done

wait  # Tüm işlemler bitene kadar bekle (opsiyonel)

