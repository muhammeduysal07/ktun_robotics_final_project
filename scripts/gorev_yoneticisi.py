#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import actionlib
import rospkg
import yaml
import os
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from pyzbar.pyzbar import decode

class RobotGorev:
    def __init__(self):
        rospy.init_node('gorev_yoneticisi_final')
        
        self.sonuc_raporu = {} 
        self.bolgeler = {}
        
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

        self.gorev_sirasi = ["SALON", "MUTFAK", "KORIDOR", "YATAKODASI"]

        self.bridge = CvBridge()
        self.son_okunan_qr = None
        self.kamera_aktif = False 
        
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
        msg = Twist()
        msg.linear.x = lin_x
        msg.angular.z = ang_z
        
        bitis_zamani = rospy.Time.now() + rospy.Duration(sure)
        while rospy.Time.now() < bitis_zamani:
            self.cmd_vel_pub.publish(msg)
            rospy.sleep(0.1)
            
        msg.linear.x = 0
        msg.angular.z = 0
        self.cmd_vel_pub.publish(msg)
        rospy.sleep(0.5)

    def qr_ara_ve_bul(self):
        self.son_okunan_qr = None
        self.kamera_aktif = True
        
        rospy.loginfo("👀 1. Deneme: Sabit bakılıyor...")
        baslangic = rospy.Time.now()
        while (rospy.Time.now() - baslangic).to_sec() < 3.0:
            if self.son_okunan_qr: return self.son_okunan_qr
            rospy.sleep(0.1)
            
        rospy.logwarn("⚠️ QR Görünmedi. Biraz geri çekiliniyor...")
        self.manuel_hareket(-0.15, 0.0, 1.5) 
        
        baslangic = rospy.Time.now()
        while (rospy.Time.now() - baslangic).to_sec() < 2.0:
            if self.son_okunan_qr: return self.son_okunan_qr
            rospy.sleep(0.1)

        rospy.logwarn("⚠️ Hala yok. Sağa bakılıyor...")
        self.manuel_hareket(0.0, -0.3, 1.0) 
        
        baslangic = rospy.Time.now()
        while (rospy.Time.now() - baslangic).to_sec() < 2.0:
            if self.son_okunan_qr: return self.son_okunan_qr
            rospy.sleep(0.1)
            
        rospy.logwarn("⚠️ Hala yok. Sola bakılıyor...")
        self.manuel_hareket(0.0, 0.6, 1.0) 
        
        baslangic = rospy.Time.now()
        while (rospy.Time.now() - baslangic).to_sec() < 2.0:
            if self.son_okunan_qr: return self.son_okunan_qr
            rospy.sleep(0.1)
            
        return None 

    def hedefe_git(self, koordinat, mesaj):
        MAX_DENEME = 2 
        
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
            
            wait = self.client.wait_for_result()
            state = self.client.get_state()

            if wait and state == 3:
                return True
            
            rospy.sleep(1.0) 

        rospy.logerr(f"❌ HEDEFE GİDİLEMEDİ: {mesaj}")
        return False

    def qr_dogrula_ve_temizle(self, oda_ismi):
        rospy.loginfo(f"{oda_ismi} kapısına gelindi. QR taranıyor...")
        
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
        satirlar = []
        satirlar.append("\n")
        satirlar.append("╔════════════════════════════════════════╗")
        satirlar.append("║          FİNAL TEMİZLİK RAPORU         ║")
        satirlar.append("╠════════════════════════════════════════╣")
        
        for oda, durum in self.sonuc_raporu.items():
            bosluk = " " * (36 - len(oda) - len(durum))
            satir = f"║ {oda}: {durum}{bosluk} ║"
            satirlar.append(satir)
            
        satirlar.append("╚════════════════════════════════════════╝")
        satirlar.append("\n")

        for satir in satirlar:
            rospy.loginfo(satir)

        try:
            rospack = rospkg.RosPack()
            paket_yolu = rospack.get_path('final_odev')
            dosya_yolu = os.path.join(paket_yolu, 'temizlik_raporu.txt')
            
            with open(dosya_yolu, "w") as dosya:
                for satir in satirlar:
                    dosya.write(satir + "\n")
            
            rospy.loginfo(f"📄 Rapor dosyaya kaydedildi: {dosya_yolu}")
        except Exception as e:
            rospy.logerr(f"Rapor dosyaya yazılamadı: {e}")

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
