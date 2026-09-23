#!/usr/bin/env bash
# 00_ortam_kontrol.sh — hiçbir şey çalıştırmadan önce ortamı teşhis eder.
# Kullanım:  bash 00_ortam_kontrol.sh
set -u

ok()   { printf '  \033[32m✔\033[0m %s\n' "$1"; }
warn() { printf '  \033[33m!\033[0m %s\n' "$1"; }
bad()  { printf '  \033[31m✘\033[0m %s\n' "$1"; }
hdr()  { printf '\n\033[1m%s\033[0m\n%s\n' "$1" "$(printf '─%.0s' {1..66})"; }

echo "══════════════════════════════════════════════════════════════════"
echo " UR10e ANOMALİ TESPİTİ — ORTAM KONTROLÜ"
echo "══════════════════════════════════════════════════════════════════"

# ── işletim sistemi ──
hdr "1. İşletim sistemi"
if [ -f /etc/os-release ]; then . /etc/os-release; echo "  $PRETTY_NAME"; fi
echo "  çekirdek : $(uname -r)"
echo "  mimari   : $(uname -m)"
[ "$(uname -m)" = "x86_64" ] && ok "x86-64 — solver derlemesiyle uyumlu" \
                             || bad "x86-64 değil — solver .so'su yüklenemez"

# ── python ──
hdr "2. Python"
PY_DEF=$(command -v python3 || true)
if [ -n "$PY_DEF" ]; then
  V=$("$PY_DEF" -c 'import sys;print("%d.%d"%sys.version_info[:2])')
  echo "  python3      : $PY_DEF  (sürüm $V)"
else
  bad "python3 bulunamadı"; V=""
fi

PY310=$(command -v python3.10 || true)
if [ -n "$PY310" ]; then
  ok "python3.10 mevcut : $PY310"
  SOLVER_PY="$PY310"
elif [ "$V" = "3.10" ]; then
  ok "python3 zaten 3.10 — solver için bunu kullanacağız"
  SOLVER_PY="$PY_DEF"
else
  bad "python3.10 YOK. Solver .so'su CPython 3.10 için derlenmiş."
  echo "     Kurulum:"
  echo "       sudo add-apt-repository -y ppa:deadsnakes/ppa"
  echo "       sudo apt update"
  echo "       sudo apt install -y python3.10 python3.10-venv python3.10-dev"
  SOLVER_PY=""
fi

# ── solver dosyası ──
hdr "3. Ters dinamik solver"
SO=$(ls resources/ur10_solver_py*.so 2>/dev/null | head -1 || true)
if [ -n "$SO" ]; then
  echo "  dosya : $SO"
  file "$SO" | sed 's/^/  /'
  case "$SO" in
    *cpython-310*x86_64-linux*) ok "Linux x86-64 / CPython 3.10 derlemesi" ;;
    *) warn "beklenmeyen derleme adı" ;;
  esac
  if [ -n "$SOLVER_PY" ]; then
    if "$SOLVER_PY" -c "import sys;sys.path.insert(0,'resources');import ur10_solver_py" 2>/dev/null; then
      ok "ur10_solver_py $($SOLVER_PY -V 2>&1) ile import EDİLİYOR"
    else
      bad "import edilemedi — 'pip install numpy' bu yorumlayıcıda yapılmış mı?"
      "$SOLVER_PY" -c "import sys;sys.path.insert(0,'resources');import ur10_solver_py" 2>&1 | tail -3 | sed 's/^/     /'
    fi
  fi
else
  bad "resources/ur10_solver_py*.so bulunamadı — resources/ klasörünü kopyaladın mı?"
fi

# ── gpu ──
hdr "4. GPU"
if command -v nvidia-smi >/dev/null 2>&1; then
  nvidia-smi --query-gpu=name,driver_version,memory.total --format=csv,noheader | sed 's/^/  /'
  ok "nvidia-smi çalışıyor"
else
  warn "nvidia-smi yok — eğitim CPU'da çok yavaş olur (sürücü kurulmalı)"
fi
if [ -n "$PY_DEF" ]; then
  "$PY_DEF" - <<'EOF' 2>/dev/null || echo "  torch kurulu değil (adım 2'de kurulacak)"
import torch
print(f"  torch {torch.__version__}  cuda={torch.cuda.is_available()}",
      f" {torch.cuda.get_device_name(0)}" if torch.cuda.is_available() else "")
EOF
fi

# ── gerekli dosyalar ──
hdr "5. Gerekli dosyalar (bu klasörde olmalı)"
need_file() { [ -f "$1" ] && ok "$1  ($(du -h "$1" | cut -f1))" || bad "$1  EKSİK"; }
need_dir()  { [ -d "$1" ] && ok "$1/" || bad "$1/  EKSİK"; }
need_file ros-joint-states.csv
need_file UR10e_InverseDynamics.fmu
need_file modelDescription.xml
need_file ur10e_jacobian.py
need_dir  resources
need_dir  binaries

# ── python paketleri ──
hdr "6. Python paketleri"
if [ -n "$PY_DEF" ]; then
  "$PY_DEF" - <<'EOF'
import importlib
for m in ["numpy","pandas","pyarrow","scipy","sklearn","matplotlib","torch","onnx","fmpy","zmq"]:
    try:
        mod = importlib.import_module(m)
        print(f"  \033[32m✔\033[0m {m:<12} {getattr(mod,'__version__','?')}")
    except Exception:
        print(f"  \033[33m!\033[0m {m:<12} yok")
EOF
fi

hdr "ÖZET"
echo "  Solver için kullanılacak yorumlayıcı : ${SOLVER_PY:-YOK}"
echo "  Eğitim için kullanılacak yorumlayıcı : ${PY_DEF:-YOK}"
echo
echo "  Yukarıda ✘ varsa BASLA.md'deki ilgili adımı tamamla."
echo "══════════════════════════════════════════════════════════════════"
