# Sistem Koordinatörü Zamanlama ve Görev Planı
## Ne Zaman Ne Yapmalı?

Bu doküman, sistem koordinatörünün görevlerini zamanlama açısından organize eder.

---

## 📅 ZAMANLAMA STRATEJİSİ

### 🟢 ŞİMDİ YAPILABİLECEKLER (Bağımsız Görevler)

Bu görevler **diğer ekip üyelerini beklemeden** yapılabilir:

---

### 1. Raspberry Pi 5 Kurulumu (Öncelik: YÜKSEK)

**Neden şimdi:**
- ✅ Bağımsız görev (STM32/LIDAR'a bağlı değil)
- ✅ Zaman alıcı (2-3 gün)
- ✅ Erken bitirince test için hazır olur
- ✅ Sorunlar erken çözülür

**Yapılacaklar:**
- [ ] Ubuntu 24.04 Server kurulumu
- [ ] ROS2 Jazzy kurulumu
- [ ] Micro-ROS agent kurulumu
- [ ] Workspace oluşturma
- [ ] Proje dosyalarını kopyalama
- [ ] Build testi

**Tahmini Süre:** 2-3 gün

**Kontrol:**
```bash
# Kurulum doğru mu?
ros2 --version  # ROS 2 Jazzy Jalisco
ros2 pkg list | grep my_robot  # Paketler görünüyor mu?
```

---

### 2. Mevcut Kodu İnceleme ve Anlama (Öncelik: ORTA)

**Neden şimdi:**
- ✅ Sistemi anlamak için zaman gerekir
- ✅ Test planları hazırlamak için gerekli
- ✅ Sorun çözmek için bilgi gerekir

**Yapılacaklar:**
- [ ] Launch dosyalarını inceleme (`real_robot_bringup.launch.py`, `real_robot_exploration.launch.py`)
- [ ] Config dosyalarını inceleme (Nav2, SLAM, Explorer)
- [ ] URDF yapısını anlama
- [ ] Sistem mimarisini anlama
- [ ] Test senaryolarını planlama

**Tahmini Süre:** 1-2 gün

**Kontrol:**
- Launch dosyaları ne yapıyor anladım mı?
- Config parametrelerinin anlamlarını biliyor muyum?
- Sistem akışını anlıyor muyum?

---

### 3. Test Planları Hazırlama (Öncelik: ORTA)

**Neden şimdi:**
- ✅ Test senaryolarını hazırlamak zaman alır
- ✅ Test checklist'leri hazır olmalı
- ✅ Test scriptleri yazılabilir

**Yapılacaklar:**
- [ ] Test senaryoları dokümantasyonu
- [ ] Test checklist'leri oluşturma
- [ ] Test scriptleri yazma (opsiyonel)
- [ ] Test sonuçları şablonu hazırlama

**Tahmini Süre:** 1 gün

---

### 4. Dokümantasyon Hazırlığı (Öncelik: DÜŞÜK)

**Neden şimdi (kısmen):**
- ✅ Kurulum rehberi yazılabilir
- ✅ README hazırlanabilir
- ✅ Troubleshooting rehberi başlatılabilir

**Yapılacaklar:**
- [ ] README.md güncelleme
- [ ] Kurulum rehberi yazma
- [ ] Troubleshooting rehberi başlatma

**Tahmini Süre:** 1-2 gün (kısmen)

---

## 🟡 DİĞER GÖREVLER BİTİNCE YAPILACAKLAR

Bu görevler **STM32 ve/veya LIDAR hazır olunca** yapılır:

---

### 5. STM32 Entegrasyon Testi (STM32 Hazır Olunca)

**Ne zaman:**
- ⏳ STM32 geliştirici firmware'i hazırladığında
- ⏳ STM32'yi Raspberry Pi'ye bağladığında
- ⏳ İlk testleri yaptığında

**Yapılacaklar:**
- [ ] Micro-ROS agent testi
- [ ] `/cmd_vel` ve `/odom` topic testleri
- [ ] TF tree kontrolü
- [ ] Motor kontrolü testi (STM32 geliştirici ile)
- [ ] Odometry doğruluğu testi

**Tahmini Süre:** 1-2 gün

**Bağımlılık:** STM32 firmware hazır olmalı ✅

---

### 6. LIDAR Entegrasyon Testi (LIDAR Hazır Olunca)

**Ne zaman:**
- ⏳ LIDAR geliştirici driver'ı kurduğunda
- ⏳ LIDAR'ı Raspberry Pi'ye bağladığında
- ⏳ İlk testleri yaptığında

**Yapılacaklar:**
- [ ] LIDAR driver kurulumunu kontrol
- [ ] `/scan` topic testi
- [ ] TF tree'de `laser_link` kontrolü
- [ ] LIDAR veri kalitesi testi
- [ ] RViz görselleştirme testi

**Tahmini Süre:** 1 gün

**Bağımlılık:** LIDAR driver kurulu ve çalışıyor olmalı ✅

---

### 7. Real Robot Bringup Testi (STM32 + LIDAR Hazır Olunca)

**Ne zaman:**
- ⏳ STM32 entegrasyon testi başarılı olduğunda
- ⏳ LIDAR entegrasyon testi başarılı olduğunda

**Yapılacaklar:**
- [ ] `real_robot_bringup.launch.py` testi
- [ ] Tüm node'ların çalıştığını kontrol
- [ ] Topic'lerin doğru çalıştığını kontrol
- [ ] TF tree'nin tam olduğunu kontrol

**Tahmini Süre:** 1 gün

**Bağımlılık:** STM32 ✅ + LIDAR ✅

---

### 8. SLAM ve Nav2 Konfigürasyon Kontrolü (Sistem Hazır Olunca)

**Ne zaman:**
- ⏳ Real robot bringup başarılı olduğunda
- ⏳ Temel sistem çalışıyor olduğunda

**Yapılacaklar:**
- [ ] SLAM parametrelerini kontrol
- [ ] Nav2 parametrelerini kontrol
- [ ] Explorer parametrelerini kontrol
- [ ] Parametre optimizasyonu (gerekirse)

**Tahmini Süre:** 1-2 gün

**Bağımlılık:** Sistem çalışıyor olmalı ✅

---

### 9. Full System Integration Test (Tüm Sistem Hazır Olunca)

**Ne zaman:**
- ⏳ Real robot bringup başarılı olduğunda
- ⏳ SLAM/Nav2 konfigürasyonları kontrol edildiğinde

**Yapılacaklar:**
- [ ] Full system launch testi
- [ ] Sistem sağlık kontrolleri
- [ ] Fonksiyonel testler:
  - Manuel motor kontrolü
  - Odometry testi
  - LIDAR testi
  - SLAM testi
  - Nav2 testi
  - Frontier Explorer testi

**Tahmini Süre:** 2-3 gün

**Bağımlılık:** Tüm sistem çalışıyor olmalı ✅

---

### 10. Troubleshooting ve Debug (Sürekli)

**Ne zaman:**
- ⏳ Her aşamada sorun çıktığında
- ⏳ Testler sırasında hata olduğunda

**Yapılacaklar:**
- [ ] Log analizi
- [ ] Hata çözümü
- [ ] Geliştiricilerle koordinasyon
- [ ] Sorun dokümantasyonu

**Tahmini Süre:** Sürekli

---

### 11. Final Test ve Demo Hazırlığı (Sistem Stabil Olunca)

**Ne zaman:**
- ⏳ Full system test başarılı olduğunda
- ⏳ Sistem stabil çalışıyor olduğunda
- ⏳ Yarışma/demo yaklaştığında

**Yapılacaklar:**
- [ ] End-to-end test
- [ ] Uzun süreli stabilite testi
- [ ] Demo senaryosu hazırlama
- [ ] Acil durum prosedürleri

**Tahmini Süre:** 2-3 gün

**Bağımlılık:** Sistem stabil çalışıyor olmalı ✅

---

## 📊 GÖREV ZAMAN ÇİZELGESİ

```
ŞİMDİ (Bağımsız):
├── Raspberry Pi Kurulumu (2-3 gün) ✅ ŞİMDİ YAP
├── Kod İnceleme (1-2 gün) ✅ ŞİMDİ YAP
├── Test Planları (1 gün) ✅ ŞİMDİ YAP
└── Dokümantasyon (kısmen) ✅ ŞİMDİ YAP

BEKLE (STM32/LIDAR Hazır Olunca):
├── STM32 Entegrasyon Testi (1-2 gün) ⏳ STM32 hazır olunca
├── LIDAR Entegrasyon Testi (1 gün) ⏳ LIDAR hazır olunca
├── Real Robot Bringup (1 gün) ⏳ STM32 + LIDAR hazır olunca
├── SLAM/Nav2 Kontrol (1-2 gün) ⏳ Sistem çalışıyor olunca
├── Full System Test (2-3 gün) ⏳ Sistem hazır olunca
└── Final Test/Demo (2-3 gün) ⏳ Sistem stabil olunca

SÜREKLI:
└── Troubleshooting ⏳ Her aşamada
```

---

## 🎯 ÖNERİLEN YAKLAŞIM

### Faz 1: Şimdi Yap (1-2 hafta)

**Hemen başla:**
1. ✅ **Raspberry Pi 5 kurulumu** (ÖNEMLİ!)
   - Bu en uzun ve en önemli görev
   - Erken bitirince diğer testler için hazır olursun
   - Sorunlar erken çözülür

2. ✅ **Kod inceleme ve anlama**
   - Sistemi anlamak kritik
   - Test planları için gerekli

3. ✅ **Test planları hazırlama**
   - Test senaryolarını hazırla
   - Checklist'leri oluştur

### Faz 2: Bekle ve Koordine Et (STM32/LIDAR Hazır Olunca)

**STM32 geliştirici ile:**
- STM32 hazır olduğunda test et
- Sorunları birlikte çöz
- Motor/odometry testleri yap

**LIDAR geliştirici ile:**
- LIDAR hazır olduğunda test et
- Scan veri kalitesini kontrol et
- TF tree'yi kontrol et

### Faz 3: Entegrasyon ve Test (Tüm Sistem Hazır Olunca)

- Full system entegrasyonu
- Kapsamlı testler
- Optimizasyon
- Demo hazırlığı

---

## ✅ ŞİMDİ YAPILACAKLAR CHECKLIST

### Hemen Başla (Bu Hafta):

- [ ] **Raspberry Pi 5 kurulumu başlat**
  - [ ] Ubuntu 24.04 Server image indir
  - [ ] SD karta yaz
  - [ ] İlk boot ve SSH kurulumu
  - [ ] ROS2 Jazzy kurulumu
  - [ ] Workspace oluşturma

- [ ] **Kod inceleme**
  - [ ] Launch dosyalarını oku
  - [ ] Config dosyalarını incele
  - [ ] URDF yapısını anla
  - [ ] Sistem akışını çiz/kavra

- [ ] **Test planları**
  - [ ] Test senaryoları listesi
  - [ ] Test checklist'leri
  - [ ] Test scriptleri (opsiyonel)

### Bu Hafta veya Gelecek Hafta:

- [ ] **Dokümantasyon (kısmen)**
  - [ ] README güncelle
  - [ ] Kurulum rehberi yaz
  - [ ] Troubleshooting rehberi başlat

---

## 📝 ÖNEMLİ NOTLAR

### Neden Raspberry Pi Kurulumu Öncelikli?

1. **Zaman alıcı:** 2-3 gün sürer
2. **Kritik:** Diğer testler için gerekli
3. **Bağımsız:** STM32/LIDAR'a bağlı değil
4. **Sorun çözme:** Erken kurulunca sorunlar erken çözülür

### Neden Diğer Görevler Beklemeli?

1. **STM32 entegrasyonu:** STM32 firmware hazır olmalı
2. **LIDAR entegrasyonu:** LIDAR driver kurulu olmalı
3. **Full system test:** Her şey hazır olmalı

### Koordinasyon Stratejisi

1. **Haftalık toplantı:** Diğer geliştiricilerle durum paylaşımı
2. **Test tarihleri:** STM32 ve LIDAR test tarihlerini planla
3. **Sorun takibi:** Sorunları dokümante et ve çöz

---

## 🚀 ÖNERİLEN İLK ADIMLAR (Bugün/Yarın)

1. **Raspberry Pi 5 kurulumunu başlat**
   ```bash
   # Ubuntu 24.04 Server image indir
   # SD karta yaz
   # İlk boot
   ```

2. **Kod inceleme başlat**
   ```bash
   # Launch dosyalarını oku
   # Config dosyalarını incele
   ```

3. **Test planları başlat**
   ```bash
   # Test senaryoları listesi oluştur
   ```

---

**Son Güncelleme:** 2024
**Sorumlu:** Sistem Koordinatörü
