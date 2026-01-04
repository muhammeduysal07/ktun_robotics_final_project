# Süpürge Robotu: Oda Bazlı Temizlik ve QR Doğrulama
**Konya Teknik Üniversitesi (KTÜN) - Robotiğe Giriş Dersi Final Ödevi**

## 📌 Proje Özeti
Bu proje, ROS Noetic ve TurtleBot3 (Waffle Pi) platformu kullanılarak geliştirilmiş otonom bir "Süpürge Robotu" simülasyonudur. Robot, Gazebo ortamında önceden haritalandırılmış (SLAM) bir ev içerisinde görev listesine uygun olarak hareket eder.

**Temel Yetenekler:**
1.  **Otonom Navigasyon:** AMCL ve Move Base kullanarak belirlenen odalara (Salon, Mutfak, Koridor, Yatak Odası) engellere çarpmadan gider.
2.  **Görsel Doğrulama (QR Kod):** Her odanın girişinde bulunan QR kodları kamera ile okuyarak doğru odada olup olmadığını teyit eder.
3.  **Dinamik Görev Yönetimi (YAML):** Oda koordinatlarını ve temizlik rotalarını kod içine gömmek yerine, harici bir `mission.yaml` dosyasından okur.
4.  **Otomatik Raporlama:** Görev bitiminde temizlik durumunu (Başarılı/Hatalı/Atlandı) raporlar.

---

## 📂 Proje Dosya Yapısı
Teslim edilen `final_odev` paketi aşağıdaki yapıya sahiptir:

```text
final_odev/
├── config/
│   └── mission.yaml       # Odaların giriş ve temizlik koordinatlarını tutan konfigürasyon dosyası.
├── launch/
│   └── final_app.launch   # Gazebo dünyasını, robotu ve gerekli parametreleri başlatan ana dosya.
├── maps/
│   ├── house_map.pgm      # Gmapping ile çıkarılmış evin harita görseli.
│   └── house_map.yaml     # Harita meta verileri.
├── scripts/
│   └── gorev_yoneticisi.py # Ana Python düğümü (Görev yönetimi, QR okuma, Raporlama).
├── world/
│   ├── final_world.world  # QR kodların yerleştirildiği Gazebo dünya dosyası.
├── CMakeLists.txt
├── package.xml
└── README.md
└── Demo_video.mp4

⚙️ Gereksinimler ve Kurulum
Bu projenin çalıştırılabilmesi için aşağıdaki bağımlılıkların kurulu olması gerekmektedir:

ROS Sürümü: Noetic Ninjemys

Simülasyon: Gazebo

TurtleBot3 Paketleri: turtlebot3, turtlebot3_msgs, turtlebot3_simulations, turtlebot3_navigation

Ek Python Kütüphaneleri (QR Okuma ve YAML için): Aşağıdaki komutlarla gerekli kütüphaneleri kurunuz:
sudo apt-get update
sudo apt-get install libzbar0
pip3 install pyzbar pyyaml rospkg opencv-python

Çalıştırma İzni: Script dosyasının çalıştırılabilir olduğundan emin olun:
chmod +x ~/robotg_ws/src/final_odev/scripts/gorev_yoneticisi.py

🚀 Çalıştırma Adımları
Simülasyonu başlatmak için 3 farklı terminal açarak sırasıyla aşağıdaki komutları giriniz:

1. Adım: Simülasyon Ortamını Başlatma
Gazebo'yu, evi ve QR kodları yükler. Robot başlangıç noktasına yerleşir.
roslaunch final_odev final_app.launch

2. Adım: Navigasyon Sistemini Başlatma
AMCL (Lokalizasyon) ve Move Base (Navigasyon) sistemini başlatır. (Not: Robotun başlangıç koordinatları launch dosyasına tanımlanmıştır, RViz üzerinden "2D Pose Estimate" yapılmasına gerek yoktur, otomatik oturur.)
roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:=$HOME/robotg_ws/src/final_odev/maps/house_map.yaml

3. Adım: Görev Yöneticisini Başlatma
Robotun mission.yaml dosyasını okuyarak temizlik görevine başladığı ana düğümdür.
rosrun final_odev gorev_yoneticisi.py

🧠 Algoritma Akışı
gorev_yoneticisi.py düğümü şu mantıkla çalışır:

INIT: config/mission.yaml dosyasını okur.

GO_TO_ENTRY: Sıradaki odanın kapı girişine gider.

QR_VERIFY: Kamera görüntüsünden QR kod okur. mission.yaml'daki oda ismi ile QR verisini karşılaştırır.

Örnek: QR içinde ROOM=SALON yazıyorsa ve hedef "SALON" ise onaylar.

CLEANING: Doğrulama başarılıysa odanın içindeki belirlenmiş 3 noktayı (Waypoint) sırayla gezer.

REPORT: Odanın sonucunu kaydeder ve sıradaki odaya geçer.

FINISH: Tüm odalar bitince terminale detaylı bir rapor tablosu basar.

🛠️ Konfigürasyon (mission.yaml)
Robotun davranışını kod değiştirmeden yönetmek için mission.yaml kullanılır. Örnek yapı:

YAML

SALON:
  Giris:
    x: -0.7384400599893532
    y: 1.1176209288360368
    z: -0.0498330199734705
    w: 0.998757563235605
  Temizlik:
    - {x: -1.0966168642044067, y: 3.5416643619537354, z: 0.0, w: 1.0}
    - {x: -3.7649736404418945, y: 1.073171854019165, z: 0.0, w: 1.0}
    - {x: -0.8032229542732239, y: 1.2328864336013794, z: 0.0, w: 1.0}
Yeni bir oda eklemek veya koordinat değiştirmek için bu dosyayı düzenlemek yeterlidir.

## 📹 Demo Videosu ve Senaryo Akışı

Aşağıdaki demo videosunda proje, tüm gereksinimleri karşılayacak şekilde çalıştırılmıştır. Videodaki işlem basamakları şunlardır:

**1. Başlatma Aşaması:**
* **00:00 - 00:15:** `final_app.launch` ile Gazebo simülasyon ortamı, ev modeli ve QR kodlar yüklendi.
* **00:15 - 00:25:** `turtlebot3_navigation` başlatıldı. Robot, AMCL algoritması ile harita üzerinde konumunu (Initial Pose) başarıyla buldu.
* **00:27:** `rqt_image_view` aracı açılarak robotun kamera akışı (Gözü) ekrana yansıtıldı.

**2. Görev İcrası:**
* **00:37:** `gorev_yoneticisi.py` ana düğümü çalıştırıldı ve `mission.yaml` dosyası okundu.
* **00:40 - 01:00 (Salon):** Robot Salon girişine gitti, QR kodu ("ROOM=SALON") doğruladı ve temizlik rotasını tamamladı.
* **01:35 - 02:20 (Mutfak):** Robot Mutfak girişine gitti, QR doğrulamasını yaptı ve temizlik noktalarını gezdi.
* **02:20 - 03:00 (Koridor):** Koridor görevi başarıyla tamamlandı.
* **03:00 - 04:30 (Yatak Odası):** Robot en uzak nokta olan Yatak Odası'na gidip görevi tamamladı.

**3. Raporlama ve Sonuç:**
* **06:00:** Görev bitiminde terminal ekranına detaylı **"Final Temizlik Raporu"** tablosu basıldı.
* **06:05:** Proje klasörü içerisinde otomatik oluşturulan `temizlik_raporu.txt` dosyası açılarak raporun kalıcı olarak kaydedildiği doğrulandı.

Hazırlayan: Muhammed Mustafa Uysal Tarih: Aralık 2025
