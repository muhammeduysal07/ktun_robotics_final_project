#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import actionlib
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from pyzbar.pyzbar import decode

class RobotGorev:
    def __init__(self):
        rospy.init_node('gorev_yoneticisi_final')
        
        # --- RAPOR LİSTESİ (YENİ) ---
        self.sonuc_raporu = {} 
        self.bolgeler = {
            "SALON": {
                "Giris":      {'x': -0.7384400599893532,  'y': 1.1176209288360368, 'z': -0.0498330199734705, 'w': 0.998757563235605},
                "Temizlik_1": {'x': -1.0966168642044067,  'y': 3.5416643619537354,  'z': 0.0, 'w': 1.0}, 
                "Temizlik_2": {'x': -3.7649736404418945, 'y': 1.073171854019165,  'z': 0.0, 'w': 1.0},
                "Temizlik_3": {'x': -0.8032229542732239, 'y': 1.2328864336013794,  'z': 0.0, 'w': 1.0}
            },
            
            "MUTFAK": {
                "Giris":     {'x': -4.145525381580453,  'y': 1.7769802403333055, 'z': 0.9999790948150499, 'w': 0.006466060073457085},
                
                "Temizlik_1": {'x': -6.655375003814697,  'y': 2.2351694107055664,  'z': 0.0, 'w': 1.0}, 
                "Temizlik_2": {'x': -5.850343704223633, 'y': 2.8236496448516846,  'z': 0.0, 'w': 1.0},
                "Temizlik_3": {'x': -5.688170433044434, 'y': 4.62660551071167,  'z': 0.0, 'w': 1.0}
            },
            
            "KORIDOR": {
                 "Giris":      {'x': 0.41669725743072716, 'y': 0.27383626175020215, 'z': 0.7262110222650667, 'w': 0.6874718547989632},
                 
                 "Temizlik_1": {'x': 0.6796004772186279, 'y': 0.11687768995761871, 'z': 0.0, 'w': 1.0},
                 "Temizlik_2": {'x': 1.1562927961349487, 'y': 0.5019264817237854, 'z': 0.0, 'w': 1.0},
                 "Temizlik_3": {'x': 2.0399017333984375, 'y': 0.240562811493873, 'z': 0.0, 'w': 1.0}
            },
            
            "YATAKODASI": {
                 "Giris":      {'x':  2.906681515875883,  'y': 0.6435640469695788, 'z': -0.7689501871557356, 'w': 0.6393086966975807},
                 
                "Temizlik_1": {'x': 6.252098083496094,  'y': 1.0236765146255493,  'z': 0.0, 'w': 1.0}, 
                "Temizlik_2": {'x': 6.493058204650879, 'y': 3.9582583904266357,  'z': 0.0, 'w': 1.0},
                "Temizlik_3": {'x': 3.0199759006500244, 'y': 2.8727474212646484,  'z': 0.0, 'w': 1.0}
            }
        }
        
        self.gorev_sirasi = ["SALON", "MUTFAK", "KORIDOR", "YATAKODASI"]

        self.bridge = CvBridge()
        self.son_okunan_qr = None
        self.kamera_aktif = False 
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo("Navigasyon bekleniyor...")
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
        
        # Varsayılan sonuç
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
                    durum = "TEMİZLİK YARIDA KALDI"
            else:
                rospy.logwarn(f"⛔ İSİM TUTMADI! Yine de devam ediliyor...")
                self.temizlik_turu_yap(oda_ismi)
                durum = "QR HATALI / TEMİZLENDİ"
        else:
            rospy.logwarn("❌ QR OKUNAMADI! Devam ediliyor...")
            self.temizlik_turu_yap(oda_ismi)
            durum = "QR OKUNAMADI / TEMİZLENDİ"
        
        self.sonuc_raporu[oda_ismi] = durum

    def temizlik_turu_yap(self, oda_ismi):
        rospy.loginfo(f"🧹 {oda_ismi} içi temizleniyor...")
        try:
            for i in range(1, 4):
                hedef = self.bolgeler[oda_ismi][f"Temizlik_{i}"]
                self.hedefe_git(hedef, f"{oda_ismi} - Nokta {i}")
                rospy.sleep(0.5)
            rospy.loginfo(f"✨ {oda_ismi} bitti!")
            return True
        except:
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
        for oda in self.gorev_sirasi:
            rospy.loginfo(f"***** {oda} GÖREVİ BAŞLADI *****")
            giris = self.bolgeler[oda]["Giris"]
            
            if self.hedefe_git(giris, f"{oda} Kapısına Gidiliyor"):
                self.qr_dogrula_ve_temizle(oda)
            else:
                rospy.logwarn(f"{oda} kapısına gidilemedi!")
                self.sonuc_raporu[oda] = "ULAŞILAMADI"
        self.raporu_yazdir()
        rospy.loginfo("GÖREV TAMAMLANDI. ROBOT ŞARJA GİDEBİLİR.")

if __name__ == '__main__':
    try:
        RobotGorev().baslat()
    except rospy.ROSInterruptException: pass
