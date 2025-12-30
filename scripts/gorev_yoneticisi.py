#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import actionlib
import rospkg
import yaml
import os
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist  # Hareket için gerekli
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from pyzbar.pyzbar import decode

class RobotGorev:
    def __init__(self):
        rospy.init_node('gorev_yoneticisi_final')
        
        self.sonuc_raporu = {} 
        self.bolgeler = {}
        
        # --- DOSYA OKUMA İŞLEMİ (MISSION.YAML) ---
        try:
            rospack = rospkg.RosPack()
            dosya_yolu = os.path.join(rospack.get_path('final_odev'), 'config', 'mission.yaml')
            rospy.loginfo(f"Görev dosyası okunuyor: {dosya_yolu}")
            
            with open(dosya_yolu, 'r') as dosya:
                self.bolgeler = yaml.safe_load(dosya)
                rospy.loginfo("✅ mission.yaml başarıyla yüklendi!")
                
        except Exception as e:
            rospy.logerr(f"DOSYA OKUMA HATASI: {e}")
            return

        # Sıralama
        self.gorev_sirasi = ["SALON", "MUTFAK", "KORIDOR", "YATAKODASI"]

        self.bridge = CvBridge()
        self.son_okunan_qr = None
        self.kamera_aktif = False 
        
        # Hareket Yayıncısı (Kurtarma manevraları için)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo("Navigasyon sistemi bekleniyor...")
        self.client.wait_for_server()
        rospy.Subscriber("/camera/rgb/image_raw", Image, self.kamera_callback)

    def kamera_callback(self, data):
        if not self.kamera_aktif: return 
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            decoded_objects = decode(cv_image)
            for obj in decoded_objects:
                self.son_okunan_qr = obj.data.decode("utf-8")
        except CvBridgeError as e: rospy.logerr(e)

    def manuel_hareket(self, lin_x, ang_z, sure):
        """ Robotu manuel olarak hareket ettirir (Kurtarma için) """
        msg = Twist()
        msg.linear.x = lin_x
        msg.angular.z = ang_z
        
        bitis_zamani = rospy.Time.now() + rospy.Duration(sure)
        while rospy.Time.now() < bitis_zamani:
            self.cmd_vel_pub.publish(msg)
            rospy.sleep(0.1)
            
        # Durdur
        msg.linear.x = 0
        msg.angular.z = 0
        self.cmd_vel_pub.publish(msg)
        rospy.sleep(0.5) # Durunca görüntü netleşsin diye bekle

    def qr_ara_ve_bul(self):
        """ QR Kodu arar, bulamazsa hareket edip tekrar dener """
        self.son_okunan_qr = None
        self.kamera_aktif = True
        
        # 1. DENEME: Olduğun yerde bak (3 saniye)
        rospy.loginfo("👀 1. Deneme: Sabit bakılıyor...")
        baslangic = rospy.Time.now()
        while (rospy.Time.now() - baslangic).to_sec() < 3.0:
            if self.son_okunan_qr: return self.son_okunan_qr
            rospy.sleep(0.1)
            
        # Bulunamadıysa...
        
        # 2. DENEME: Biraz Geri Çekil (Çok yakınsa göremez)
        rospy.logwarn("⚠️ QR Görünmedi. Biraz geri çekiliniyor...")
        self.manuel_hareket(-0.15, 0.0, 1.5) # 15 cm/s hızla 1.5 sn geri
        
        baslangic = rospy.Time.now()
        while (rospy.Time.now() - baslangic).to_sec() < 2.0:
            if self.son_okunan_qr: return self.son_okunan_qr
            rospy.sleep(0.1)

        # 3. DENEME: Sağa Dön Bak
        rospy.logwarn("⚠️ Hala yok. Sağa bakılıyor...")
        self.manuel_hareket(0.0, -0.3, 1.0) # Sağa dön
        
        baslangic = rospy.Time.now()
        while (rospy.Time.now() - baslangic).to_sec() < 2.0:
            if self.son_okunan_qr: return self.son_okunan_qr
            rospy.sleep(0.1)
            
        # 4. DENEME: Sola Dön Bak (Önce ortala sonra sola git)
        rospy.logwarn("⚠️ Hala yok. Sola bakılıyor...")
        self.manuel_hareket(0.0, 0.6, 1.0) # Sola dön (daha çok dön ki diğer tarafı gör)
        
        baslangic = rospy.Time.now()
        while (rospy.Time.now() - baslangic).to_sec() < 2.0:
            if self.son_okunan_qr: return self.son_okunan_qr
            rospy.sleep(0.1)
            
        return None # Hiçbir şekilde bulunamadı

    def hedefe_git(self, koordinat, mesaj):
        """
        Hedefe gitmeye çalışır. Başarısız olursa 1 kez daha dener.
        """
        MAX_DENEME = 2  # İlk deneme + 1 tekrar
        
        for deneme in range(1, MAX_DENEME + 1):
            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = "map"
            goal.target_pose.header.stamp = rospy.Time.now()
            goal.target_pose.pose.position.x = koordinat['x']
            goal.target_pose.pose.position.y = koordinat['y']
            goal.target_pose.pose.orientation.z = koordinat['z']
            goal.target_pose.pose.orientation.w = koordinat['w']
            
            if deneme == 1:
                rospy.loginfo(f"--- {mesaj} (Deneme 1) ---")
            else:
                rospy.logwarn(f"⚠️ İlk deneme başarısız! Tekrar deneniyor... ({mesaj})")

            self.client.send_goal(goal)
            
            # Sonucu bekle
            wait = self.client.wait_for_result()
            state = self.client.get_state()

            # Eğer başarılıysa (State 3 = SUCCEEDED)
            if wait and state == 3:
                return True
            
            # Başarısızsa döngü başa döner ve tekrar dener
            rospy.sleep(1.0) # Robot bi nefes alsın

        rospy.logerr(f"❌ HEDEFE GİDİLEMEDİ: {mesaj}")
        return False

    def qr_dogrula_ve_temizle(self, oda_ismi):
        rospy.loginfo(f"{oda_ismi} kapısına gelindi. QR taranıyor...")
        
        # YENİ FONKSİYONU ÇAĞIRIYORUZ (Hareketli Arama)
        bulunan_qr = self.qr_ara_ve_bul()
        self.kamera_aktif = False
        
        durum = "BAŞARISIZ"

        if bulunan_qr:
            okunan = bulunan_qr.lower()
            aranan = oda_ismi.lower()
            
            if aranan in okunan:
                rospy.loginfo(f"✅ {oda_ismi} DOĞRULANDI! Temizlik Başlıyor.")
                if self.temizlik_turu_yap(oda_ismi):
                    durum = "TEMİZLENDİ (BAŞARILI)"
                else:
                    durum = "YARIDA KALDI"
            else:
                rospy.logwarn(f"⛔ İSİM TUTMADI! ({okunan} != {aranan}) - ODA ATLANDI.")
                durum = "ATLANDI (QR HATALI)"
        else:
            rospy.logwarn(f"❌ TÜM DENEMELERE RAĞMEN QR YOK! - ODA ATLANDI.")
            durum = "ATLANDI (QR BULUNAMADI)"
        
        self.sonuc_raporu[oda_ismi] = durum

    def temizlik_turu_yap(self, oda_ismi):
        rospy.loginfo(f"🧹 {oda_ismi} içi temizleniyor...")
        try:
            noktalar = self.bolgeler[oda_ismi]["Temizlik"]
            for i, nokta in enumerate(noktalar, 1):
                self.hedefe_git(nokta, f"{oda_ismi} - TEMİZLİK NOKTASI {i}")
                rospy.sleep(0.5)
            rospy.loginfo(f"✨ {oda_ismi} bitti!")
            return True
        except Exception as e:
            rospy.logerr(f"Temizlik hatası: {e}")
            return False

    def raporu_yazdir(self):
        rospy.loginfo("\n")
        rospy.loginfo("╔════════════════════════════════════════╗")
        rospy.loginfo("║          FİNAL TEMİZLİK RAPORU         ║")
        rospy.loginfo("╠════════════════════════════════════════╣")
        for oda, durum in self.sonuc_raporu.items():
            bosluk = " " * (36 - len(oda) - len(durum))
            rospy.loginfo(f"║ {oda}: {durum}{bosluk} ║")
        rospy.loginfo("╚════════════════════════════════════════╝")
        rospy.loginfo("\n")

    def baslat(self):
        if not self.bolgeler:
            rospy.logerr("Görev listesi boş! YAML dosyası okunamadı.")
            return

        for oda in self.gorev_sirasi:
            rospy.loginfo(f"***** {oda} GÖREVİ BAŞLADI *****")
            if oda in self.bolgeler:
                giris = self.bolgeler[oda]["Giris"]
                if self.hedefe_git(giris, f"{oda} Kapısına Gidiliyor"):
                    self.qr_dogrula_ve_temizle(oda)
                else:
                    self.sonuc_raporu[oda] = "ULAŞILAMADI"
            else:
                rospy.logerr(f"{oda} koordinatları YAML dosyasında bulunamadı!")

        self.raporu_yazdir()
        rospy.loginfo("GÖREV TAMAMLANDI.")

if __name__ == '__main__':
    try:
        RobotGorev().baslat()
    except rospy.ROSInterruptException: pass
