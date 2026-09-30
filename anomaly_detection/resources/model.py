import pickle
import sys
import os
import numpy as np
from fmi2 import Fmi2FMU, Fmi2Status

# --- DEBUG LOGGING SETUP ---
# FMU'nun iç dünyasını görmek için bir pencere açıyoruz.
DEBUG_FILE = "/tmp/fmu_debug_log.txt"

def log_to_file(msg):
    try:
        with open(DEBUG_FILE, "a") as f:
            f.write(f"{msg}\n")
    except:
        pass # Dosya yazılamazsa akışı bozma

# Her başlatmada log dosyasını temizle veya ayır
log_to_file("\n\n=== YENİ FMU OTURUMU BAŞLATILIYOR ===")
# ---------------------------

# 1. C++ Kütüphanesini Yola Ekle
current_dir = os.path.dirname(os.path.abspath(__file__))
if current_dir not in sys.path:
    sys.path.insert(0, current_dir)
    log_to_file(f"Path eklendi: {current_dir}")

# 2. Kütüphaneyi İçe Aktar
solver_module = None
try:
    import ur10_solver_py
    solver_module = ur10_solver_py
    log_to_file("✅ C++ Modülü (ur10_solver_py) import edildi.")
except ImportError as e:
    log_to_file(f"❌ KRİTİK HATA: Modül import edilemedi: {e}")
except Exception as e:
    log_to_file(f"❌ BEKLENMEYEN HATA (Import): {e}")

class Model(Fmi2FMU):
    def __init__(self, reference_to_attr=None):
        log_to_file("Model.__init__ başladı.")
        super().__init__(reference_to_attr)
        
        # Dahili değişkenler
        self.q = np.zeros(6)
        self.qd = np.zeros(6)
        self.qdd = np.zeros(6)
        self.tau = np.zeros(6)

        # Value Reference Eşleşmeleri
        self.reference_to_attr = {}
        for i in range(6): self.reference_to_attr[i] = f"q{i+1}"      # 0-5
        for i in range(6): self.reference_to_attr[i+6] = f"qd{i+1}"   # 6-11
        for i in range(6): self.reference_to_attr[i+12] = f"qdd{i+1}" # 12-17
        for i in range(6): self.reference_to_attr[i+18] = f"tau{i+1}" # 18-23

        # Değişkenleri sıfırla
        for attr in self.reference_to_attr.values():
            setattr(self, attr, 0.0)

        # 3. C++ Solver Nesnesini Oluştur
        self.solver = None
        if solver_module:
            try:
                log_to_file("C++ Solver nesnesi oluşturulmaya çalışılıyor...")
                self.solver = solver_module.InverseDynamicsSolverUR10()
                log_to_file("✅ C++ Solver nesnesi BAŞARIYLA oluşturuldu.")
            except Exception as e:
                log_to_file(f"❌ HATA: C++ Nesnesi oluşturulamadı: {e}")
        else:
            log_to_file("⚠️ UYARI: Modül yüklü olmadığı için Solver oluşturulmadı.")

        # İlk çıktı güncellemesi
        self._update_outputs()
        log_to_file("Model.__init__ tamamlandı.")

    def _update_outputs(self):
        if not self.solver:
            return

        try:
            # Girdileri al
            for i in range(6):
                self.q[i]   = getattr(self, f"q{i+1}")
                self.qd[i]  = getattr(self, f"qd{i+1}")
                self.qdd[i] = getattr(self, f"qdd{i+1}")

            # C++ Hesaplama
            # log_to_file(f"Hesaplama İsteği -> q: {self.q[:2]}...") # Çok sık log yazar, gerekirse açın
            
            tau_result = self.solver.getTorques(self.q, self.qd, self.qdd)
            
            # Çıktıları yaz
            for i in range(6):
                setattr(self, f"tau{i+1}", float(tau_result[i]))
                
        except Exception as e:
            log_to_file(f"❌ HESAPLAMA HATASI (_update_outputs): {e}")

    def do_step(self, current_time, step_size, no_prior):
        self._update_outputs()
        return Fmi2Status.ok

    def enter_initialization_mode(self):
        log_to_file("enter_initialization_mode çağrıldı.")
        self._update_outputs()
        return Fmi2Status.ok

    def exit_initialization_mode(self):
        log_to_file("exit_initialization_mode çağrıldı.")
        return Fmi2Status.ok
        
    def reset(self):
        log_to_file("reset çağrıldı.")
        self.q[:] = 0.0; self.qd[:] = 0.0; self.qdd[:] = 0.0
        self._update_outputs()
        return Fmi2Status.ok

    def terminate(self):
        log_to_file("terminate çağrıldı. Güle güle.")
        return Fmi2Status.ok

    # --- Standart Boilerplate (Değişmedi) ---
    def serialize(self):
        data = (self.q, self.qd, self.qdd)
        return Fmi2Status.ok, pickle.dumps(data)

    def deserialize(self, bytes):
        (self.q, self.qd, self.qdd) = pickle.loads(bytes)
        self._update_outputs()
        return Fmi2Status.ok

    def get_variable_name(self, vr): return self.reference_to_attr[vr]

    def set_real(self, refs, values):
        for ref, val in zip(refs, values):
            attr = self.get_variable_name(ref)
            setattr(self, attr, val)
        return Fmi2Status.ok

    def get_real(self, refs):
        return [getattr(self, self.get_variable_name(ref)) for ref in refs], Fmi2Status.ok

    def instantiate(self, instanceName, resourceLocation): return Fmi2Status.ok
    def setup_experiment(self, startTime, stopTime, tolerance): return Fmi2Status.ok

def create_fmu_instance():
    return Model()