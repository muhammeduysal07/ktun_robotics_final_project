#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import actionlib
import rospkg
import yaml
import os
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
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
            rospy.logerr("Lütfen 'src/final_odev/config/mission.yaml' dosyasının olduğundan emin olun!")
            return

        # Sıralama
        self.gorev_sirasi = ["SALON", "MUTFAK", "KORIDOR", "YATAKODASI"]

        self.bridge = CvBridge()
        self.son_okunan_qr = None
        self.kamera_aktif = False 
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

    def hedefe_git(self, koordinat, mesaj):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = koordinat['x']
        goal.target_pose.pose.position.y = koordinat['y']
        goal.target_pose.pose.orientation.z = koordinat['z']
        goal.target_pose.pose.orientation.w = koordinat['w']
        rospy.loginfo(f"--- {mesaj} ---")
        self.client.send_goal(goal)
        wait = self.client.wait_for_result()
        return wait

    def qr_dogrula_ve_temizle(self, oda_ismi):
        rospy.loginfo(f"{oda_ismi} kapısına gelindi. QR bekleniyor...")
        self.son_okunan_qr = None
        self.kamera_aktif = True
        baslangic = rospy.Time.now()
        
        while (rospy.Time.now() - baslangic).to_sec() < 5.0:
            if self.son_okunan_qr: break
            rospy.sleep(0.1)
        
        self.kamera_aktif = False
        durum = "BAŞARISIZ"

        if self.son_okunan_qr:
            okunan = self.son_okunan_qr.lower()
            aranan = oda_ismi.lower()
            rospy.loginfo(f"🔍 Aranan: '{aranan}' | Okunan: '{okunan}'")
            
            if aranan in okunan:
                rospy.loginfo(f"✅ {oda_ismi} DOĞRULANDI! Temizlik Başlıyor.")
                if self.temizlik_turu_yap(oda_ismi):
                    durum = "TEMİZLENDİ (BAŞARILI)"
                else:
                    durum = "YARIDA KALDI"
            else:
                rospy.logwarn(f"⛔ İSİM TUTMADI! Yine de temizleniyor...")
                self.temizlik_turu_yap(oda_ismi)
                durum = "QR HATALI / TEMİZLENDİ"
        else:
            rospy.logwarn("❌ QR OKUNAMADI! Yine de temizleniyor...")
            self.temizlik_turu_yap(oda_ismi)
            durum = "QR YOK / TEMİZLENDİ"
        
        self.sonuc_raporu[oda_ismi] = durum

    def temizlik_turu_yap(self, oda_ismi):
        rospy.loginfo(f"🧹 {oda_ismi} içi temizleniyor...")
        try:
            # YAML'dan gelen liste üzerinde dönüyoruz
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
