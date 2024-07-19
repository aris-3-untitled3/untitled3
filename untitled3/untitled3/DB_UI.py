from PyQt5.QtWidgets import QTableWidgetItem, QAbstractItemView, QApplication, QMainWindow, QDialog, QMessageBox
from PyQt5 import uic
from PyQt5.QtCore import QTimer
# import pygame
import os
import threading
from DB_manager import DB_Manager
from datetime import datetime
import time

path = '/home/sophie/untitled3/UI/'
age_gender_path = '/home/sophie/untitled3/'


# UI 파일 경로 설정
ui_file = os.path.join(path, "Title.ui")
ui_file2 = os.path.join(path, "Loading.ui")
ui_Direction = os.path.join(path, "Direction.ui")
ui_Recommend = os.path.join(path, "Recommend_kor_origin.ui")
ui_Preparing = os.path.join(path, "Preparing.ui")
ui_Making = os.path.join(path, "Making.ui")
ui_Maked = os.path.join(path, "Maked.ui")
ui_Coupon = os.path.join(path, "Coupon.ui")
ui_Payment = os.path.join(path, "Payment.ui")
ui_Bye = os.path.join(path, "Bye.ui")

from_class = uic.loadUiType(ui_file)[0]
from_class2 = uic.loadUiType(ui_file2)[0]
from_class_Direction = uic.loadUiType(ui_Direction)[0]
from_class_Recommend = uic.loadUiType(ui_Recommend)[0]
from_class_Preparing = uic.loadUiType(ui_Preparing)[0]
from_class_Making = uic.loadUiType(ui_Making)[0]
from_class_Maked = uic.loadUiType(ui_Maked)[0]
from_class_Coupon = uic.loadUiType(ui_Coupon)[0]
from_class_Payment = uic.loadUiType(ui_Payment)[0]
from_class_Bye = uic.loadUiType(ui_Bye)[0]


# DB_test.py 함수 구현 부분
class DB_main:
    def __init__(self):
        self.mydb = DB_Manager()
        self.mydb.DB_connect()
        self.mydb.create_table()

        # 팀원과 임의로 정한 맛 추천 테이블
        self.None_flavor = {'0-6': {'F':'berry', 'M':'choco'},
                    '7-18': {'F':'berry', 'M':'choco'},
                    '19-29': {'F':'choco', 'M':'choco'},
                    '30-49': {'F': 'vanilla', 'M':'vanilla'},
                        '50-': {'F': 'vanilla', 'M':'vanilla'}
                        }

        self.None_topping = {'0-6': {'F':'topA', 'M':'topC'},
                    '7-18': {'F':'topB', 'M':'topA'},
                    '19-29': {'F':'topA', 'M':'topC'},
                    '30-49': {'F': 'topB', 'M':'topB'},
                        '50-': {'F': 'topB', 'M':'topA'}
                        }

    def load_age_gender(self):
        with open('latest_age_gender.txt','r') as f:
            info_txt = f.read()

        infos = info_txt.split('\t')  # 탭 문자로 분리

        if len(infos) >= 2:
            age = infos[0].strip()  # 첫 번째 요소를 age 변수에 저장하고 좌우 공백 제거
            gender = infos[1].strip()  # 두 번째 요소를 gender 변수에 저장하고 좌우 공백 제거
            print(f"{age} {gender} Load Success")
        else:
            print("Failed to load age and gender information from the file.")

        return age,gender

    def recommend_flavor_topping(self):
        age, gender = self.load_age_gender()
        most_flavor, most_topping = self.mydb.load_sales(age, gender)

        if most_flavor is None:
            most_flavor = self.None_flavor[age][gender]
            print(f"맛 추천 정보가 없어서 임의로 출력함 fl  avor = {self.None_flavor[age][gender]}")
        if most_topping is None:
            most_topping = self.None_topping[age][gender]
            print(f"토핑 추천 정보가 없어서 임의로 출력함 topping = {self.None_topping[age][gender]}")

        print (f"맛 추천 {most_flavor}, {most_topping}")

        return age, gender, most_flavor, most_topping

    def API_module(self, audio_file, text_file, response_time):
        # 추천 메뉴에 대한 응답 음성 인식
        if response_time == 3:
            print("추천드린 맛으로 선택하시겠습니까? (네/아니요)")
            time.sleep(0.5)
        if response_time == 5:
            print("메뉴 선택 후 말씀해주세요")
            print("예시) 딸기맛 하나랑 토핑 A로 부탁해")
            time.sleep(0.5)

        record_api = speech_API.Record_API(audio_file, text_file, response_time)
        flag = record_api.run()

        if text_file:
            print(f"audio_file = {audio_file}. VoicetoText = {text_file} 성공적으로 저장됨")
        else:
            print("API_module --> text_file 생성되지 않음")
            audio_file, response_time = None

        return text_file, response_time, flag
    
    def word_detect(self, text_file, response_time, flag):
        flavor_list = ["딸기", 
                       "초코",
                       "바닐라"]
        
        topping_list = ["A", "에이",
                        "B", "비", "삐",
                        "C", "씨", "시"]

        # 응답 Y/N 판별하는 구문
        if response_time == 3:
            with open(text_file, 'r') as f:
                vtot = f.read()
            if '네' in vtot:
                YorN = 'Y'
            elif "아니요" in vtot or "아니오" in vtot or "아니" in vtot:
                YorN = 'N'
            else:
                print("다시 말씀해주세요")
                YorN = 'E' #error
            return YorN

        # Flavor, Topping 녹화-변환-저장 파일 불러오기(ex. 딸기맛 하나랑 토핑 A로 부탁해)
        if response_time == 5:
            if flag == 1:
                with open(text_file,'r') as f:
                    text_file = f.read()
                text_file = text_file.upper()
                flavor_topping = text_file.split(' ') # (ex. ['딸기맛', '하나랑', '토핑A로', '부탁해'])
                print(f"flavor_topping --> {flavor_topping}")
                flavor = [flavor for flavor in flavor_list 
                    if any(flavor in word for word in flavor_topping)]
            else:
                flavor = None
                
            print(f"인식된 flavor --> {flavor}")


            if flag == 1 and len(flavor) == 1:
                if flavor[0] == flavor_list[0]:
                    Flavor = "berry"
                if flavor[0] == flavor_list[1]:
                    Flavor = "choco"
                if flavor[0] == flavor_list[2]:
                    Flavor = "vanilla"
            elif flag == 1 and len(flavor) > 1:
                print("flavor -- 중복되었습니다.")
                Flavor = "error"
            else:
                print("flavor -- 다시 말씀해주세요")
                Flavor = "error"

            if flag == 1:
                topping = [topping for topping in topping_list
                        if any(topping in word for word in flavor_topping)]
            else:
                topping = None
            print(f"인식된 topping --> {topping}")


            if flag == 1 and len(topping) == 1:
                if topping[0] in [topping_list[0], topping_list[1]]:
                    Topping = "topA"
                if topping[0] in [topping_list[2], topping_list[3], topping_list[4]]:
                    Topping = "topB"
                if topping[0] in [topping_list[5], topping_list[6], topping_list[7]]:
                    Topping = "topC"

            elif flag == 1 and len(topping) > 1:
                print("topping -- 중복되었습니다")
                Topping = "error"
            else:
                print("topping -- 다시 말씀해주세요")
                Topping = "error"

            return Flavor, Topping
        
    def YorN_response(self, most_flavor, most_topping): #(self, response_time, most_flavor, most_topping):
        current_time = datetime.now().strftime("%Y%m%d_%H%M%S")

        print(f"{current_time} %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%")
        YorN_audio = f"YorN_response{current_time}.wav"
        YorN_vtot = f"YorN_response{current_time}.txt"

        text_file, response_time, flag = self.API_module(YorN_audio, YorN_vtot, 3)
        YorN = self.word_detect(text_file, response_time, flag)

        cnt = 0 #다시 음성 요청할 때, 리밋하기 위한 카운터
        # 추천 메뉴 Y --> 그대로 return, N --> (1) 음성으로 메뉴 고르기 (2) 클릭으로 메뉴 고르기 값이 return
        if YorN == 'Y':
            Flavor = most_flavor
            Topping = most_topping

        # 만약 음성으로 인식받는다고 하면 아래 파일 실행, 아니면 버튼에서 눌린 데이터값 불러와서 저장
        elif YorN == 'N':
            flavor_audio = f"Flavor_Topping{current_time}.wav"
            flavor_vtot = f"Flavor_Topping{current_time}.txt"
            # error값이 나오면 cnt = 0, 1, 2로 증가하는데 3이 되기 전까지 API_module과 word_detect 반복하는?
            cnt = 0 #다시 음성 요청할 때, 리밋하기 위한 카운터
            while cnt < 2:
                text_file, response_time, flag = self.API_module(flavor_audio, flavor_vtot, 5)
                Flavor, Topping = self.word_detect(text_file, response_time, flag)

                if Flavor == "error" or Topping == "error":
                    Flavor = None
                    Topping = None
                    cnt += 1
                else:
                    break
        else:
            flavor_audio = f"Flavor_Topping{current_time}.wav"
            flavor_vtot = f"Flavor_Topping{current_time}.txt"
            while cnt < 2:
                text_file, response_time, flag = self.API_module(YorN_audio, YorN_vtot, 3)
                YorN = self.word_detect(text_file, response_time, flag)

                if YorN == 'E':
                    Flavor = None
                    Topping = None
                    cnt += 1
                else:
                    break

        print(f"YorN_response return {Flavor}, {Topping} _=====================================================")
        return Flavor, Topping

    def YorN_stamp(self, YorN, phone):
        if YorN == 'Y':
            print("continue coupon")

            Phone = phone
            coupon = self.mydb.load_customer_info(Phone) # 쿠폰 갯수 load

        else:
            Phone = ""
            coupon = None

        return str(Phone), coupon
    
    def YorN_use_coupon(self, coupon, use_coupon):
        if use_coupon == 'Y' and coupon >= 10:
                price = 0
                coupon -= 10
                print(f"{price}, 쿠폰 사용 {use_coupon} --> 선택한 use coupon 값")
        return coupon, price

    def show_price(self, Topping, topping_time):
        flavor_price, toppic_price = self.mydb.load_price(Topping)
        price = flavor_price + (toppic_price*topping_time)

        return price

    def update_infos(self, Flavor, Topping, price, Phone, age, gender, Use_coupon): 
        save_sale = self.mydb.save_sales(Flavor, Topping, price, Phone, age, gender, Use_coupon)

        if Phone:
            user_id, coupon = self.mydb.save_customer_info(Phone, age, gender, Use_coupon)
            print(f"고객정보 업데이트 완료 --> {user_id}")
            return user_id, coupon
        else:
            coupon = None
            print("쿠폰 적립하지 않는다고 해서 손님 저장하지 않음")
            print(f"saved sales ----> {save_sale}")
            return Phone, coupon
    
    def update_stock(self, Flavor, Topping):
        result, stock_dict, flavor_flag, topping_flag = self.mydb.save_stock(Flavor, Topping)

        if stock_dict and flavor_flag and topping_flag is not None:
            print("**********************************재고 리스트****************************************")
            print(f"*-{stock_dict}-*")
            print("************************************************************************************")

            if flavor_flag is not None:
                print(f"Out of stock {flavor_flag}")
            if topping_flag is not None:
                print(f"Out of stock {topping_flag}")

            print("성공적으로 마쳤다!!")
        else:
            print("Final ERROR------------------------------")

        return result, stock_dict, flavor_flag, topping_flag


class RecommendWindow(QMainWindow, from_class_Recommend):
    def __init__(self):
        super().__init__()

        # 각 버튼에 대해 토글 이벤트 연결
        self.pushButton_1.clicked.connect(self.toggle_button) 
        self.pushButton_2.clicked.connect(self.toggle_button)
        self.pushButton_3.clicked.connect(self.toggle_button)
        self.pushButton_4.clicked.connect(self.toggle_button)
        self.pushButton_5.clicked.connect(self.toggle_button)
        self.pushButton_6.clicked.connect(self.toggle_button)


        print("@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@잘 넘어옴")
        # 맛 추천 표시 두 가지 방법 중 선택하기
        age, gender, recommended_flavor, recommended_topping = DB_main().recommend_flavor_topping()
        print(f"{recommended_flavor}, {recommended_topping}")
        self.label_2.setText(f"손님께 추천드리는 맛과 토핑은 {recommended_flavor}와 {recommended_topping}입니다.")

        self.Flavor, self.Topping = DB_main().YorN_response(recommended_flavor, recommended_topping)

        print(f"@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@{self.Flavor} {self.Topping}")

        # 인식 못했으면 클릭 활성화 --> 손님 직접 메뉴 선택
        if self.Flavor is None and self.Topping is None:
            # QPushButton을 토글 버튼으로 만들기
            self.pushButton_1.setCheckable(True) # choco
            self.pushButton_2.setCheckable(True) # vanilla
            self.pushButton_3.setCheckable(True) # berry
            self.pushButton_4.setCheckable(True) # topC
            self.pushButton_5.setCheckable(True) # topB
            self.pushButton_6.setCheckable(True) # topA

            # 소리
            self.pushButton_1.clicked.connect(self.on_click)
            self.pushButton_2.clicked.connect(self.on_click)
            self.pushButton_3.clicked.connect(self.on_click)
            self.pushButton_4.clicked.connect(self.on_click)
            self.pushButton_5.clicked.connect(self.on_click)
            self.pushButton_6.clicked.connect(self.on_click)
            # Next 버튼 이벤트 연결
            self.pushButton.clicked.connect(self.check_selection_and_next)
        else:
            Flavor = self.Flavor
            Topping = self.Topping
            print(f"check_selection_and_next ==========> global4 {Flavor}, {Topping}")
            print(f"check_selection_and_next ==========> global5 self.{Flavor}, self.{Topping}")
            QTimer.singleShot(30,  self.open_Preparing_window)


    def toggle_button(self):
        global Flavor, Topping
        button = self.sender()
        if button.isChecked():
            if button == self.pushButton_1:
                button.setStyleSheet('background-color: brown')  # 초코 색
                Flavor = "choco"
            elif button == self.pushButton_2:
                button.setStyleSheet('background-color: beige')  # 바닐라 색
                Flavor = "vanilla"
            elif button == self.pushButton_3:
                button.setStyleSheet('background-color: red')  # 딸기 색
                Flavor = "berry"
            elif button == self.pushButton_4:
                button.setStyleSheet('background-color: lightgray')  # 토핑 색상
                Topping = "topC"
            elif button == self.pushButton_5:
                button.setStyleSheet('background-color: lightgray')  # 토핑 색상
                Topping = "topB"
            elif button == self.pushButton_6:
                button.setStyleSheet('background-color: lightgray')  # 토핑 색상
                Topping = "topA"
        else:
            button.setStyleSheet('')  # 초기 상태로 되돌림

        # if recommended_flavor == "choco":
        #     button.setStyleSheet(self.RecommendSheetChoco)
        # elif recommended_flavor == "vanilla":
        #     button.setStyleSheet(self.RecommendSheetVanilla)
        # elif recommended_flavor == "berry":
        #     button.setStyleSheet(self.RecommendSheetBerry)
        # else:
        #     print("Error_recommended Flavor Style Sheet")

        # if recommended_topping == "topC":
        #     button.setStyleSheet(self.RecommendSheetTopC)
        # elif recommended_topping == "topB":
        #     button.setStyleSheet(self.RecommendSheetTopB)
        # elif recommended_topping == "topA":
        #     button.setStyleSheet(self.RecommendSheetTopA)
        # else:
        #     print("Error_recommended Topping Style Sheet")


    def check_selection_and_next(self):
        global Flavor, Topping
        print(f"check_selection_and_next ==========> global1 {Flavor}, {Topping}")
        # 아이스크림 맛이 하나가 선택되었는지 확인
        ice_cream_selected = (
            (self.pushButton_1.isChecked() and not self.pushButton_2.isChecked() and not self.pushButton_3.isChecked()) or
            (not self.pushButton_1.isChecked() and self.pushButton_2.isChecked() and not self.pushButton_3.isChecked()) or
            (not self.pushButton_1.isChecked() and not self.pushButton_2.isChecked() and self.pushButton_3.isChecked())
        )
        topping_selected = (
            (self.pushButton_4.isChecked() and not self.pushButton_5.isChecked() and not self.pushButton_6.isChecked()) or
            (not self.pushButton_4.isChecked() and self.pushButton_5.isChecked() and not self.pushButton_6.isChecked()) or
            (not self.pushButton_4.isChecked() and not self.pushButton_5.isChecked() and self.pushButton_6.isChecked())
        )

        if not ice_cream_selected:
            self.on_click_warn()
            QMessageBox.warning(self, 'Warning', '아이스크림 맛을 한 가지만 골라주세요.')

        if not topping_selected:
            self.on_click_warn()
            QMessageBox.warning(self, 'Warning', '토핑을 한 가지만 골라주세요.')

        if ice_cream_selected and topping_selected:
            if self.pushButton_1.isChecked():
                self.Flavor = "choco"
            elif self.pushButton_2.isChecked():
                self.Flavor = "vanilla"
            else:
                self.Flavor = "berry"

            if self.pushButton_4.isChecked():
                self.Topping = "topC"
            elif self.pushButton_5.isChecked():
                self.Topping = "topB"
            else:
                self.Topping = "topA"

            Flavor = self.Flavor
            Topping = self.Topping

            print(f"check_selection_and_next ==========> self.{self.Flavor}, {self.Topping}")
            print(f"check_selection_and_next ==========> global2 {Flavor}, {Topping}")
            self.open_Preparing_window()

class PaymentWindow(QMainWindow, from_class_Payment):
    def __init__(self, coupon_value, number):
        super().__init__()
        global Flavor, Topping, age, gender
        self.setupUi(self)
        self.coupon_value = coupon_value # stamp
        self.number = number # str



        self.Use_coupon = 'N' # default 값
        
        if self.coupon_value == 1:  # stamp 적립 Y
            YorN = 'Y'
        if self.coupon_value == 0:
            YorN = 'N'

        if self.number is not None and self.number != "":
            phone = int(self.number)
            self.number, self.coupon = DB_main().YorN_stamp(YorN, phone)
            print(f"고객 정보 --> {self.number}, {self.coupon}")
        else:
            phone = None



        topping_time = 5 # topping time은 토크 혹은 나오는 실제 시간으로 판단할 예정

        # 임의의 금액
        self.price = DB_main().show_price(Topping, topping_time)
        self.priceText(self.price)

        # 임의의 쿠폰 수 적용 및 텍스트 출력
        if self.coupon_value != 0:
            self.set_coupon_text(self.coupon)
        
        # 사용 버튼 시
        self.pushButton_6.clicked.connect(self.use_coupon)   
        # 결제 버튼 시
        self.pushButton_8.clicked.connect(self.payment)
        #self.pushButton_8.clicked.connect(self.no_use_coupon)
        # Back 버튼 추가
        self.pushButton_back.clicked.connect(self.open_Coupon_window)
        
        ## 소리!
        # 사용 버튼 시
        self.pushButton_6.clicked.connect(self.on_click)
        # 결제 버튼 시
        self.pushButton_8.clicked.connect(self.on_click)
        # Back 버튼 추가
        self.pushButton_back.clicked.connect(self.on_click)

        # if self.Use_coupon == 'N':
        #     self.coupon, self.price = DB_main().YorN_use_coupon(self.coupon, self.Use_coupon)

        # update_info, update_coupon = DB_main().update_infos(Flavor, Topping, self.price, int(self.number), age, gender, self.Use_coupon)
        # if update_info:
        #     print(f"손님 정보 업데이트 완료 ===> {update_info, update_coupon}")
        # else:
        #     print("Error-----------------------------")      

    def priceText(self, price):
        text = str(price) + ' 원'
        self.textBrowser_2.setText(text)
        
    def set_coupon_text(self, coupon):
        message = f"회원님의 쿠폰 수는 {coupon}개입니다."
        self.textBrowser.setText(message)
        message2 = f"회원번호: 010-" + self.number[:4] + '-' + self.number[4:]
        self.textBrowser_3.setText(message2)

    def use_coupon(self):
        if self.number is not None and self.number != "":
            if self.coupon >= 10 and self.coupon_value == 1: #여기 coupons --> coupon
                self.Use_coupon = 'Y' # 사용하면 Y로 바꿈
                coupon = self.coupon
                coupon -= 10
                price = self.price
                price = 0
                phone = self.number
                self.priceText(price)
                self.set_coupon_text(coupon)
                message2 = f"남은 쿠폰 수는 {coupon}개 입니다."
                if self.Use_coupon == 'Y':
                    update_info, coupon = DB_main().update_infos(Flavor, Topping, price, int(phone), age, gender, self.Use_coupon)
                    print(f"update_info | coupon {update_info}, {coupon}")
                QMessageBox.warning(self, '쿠폰 사용 완료', message2)
            else:
                message2 = f"쿠폰이 10개 미만일 경우에는 사용이 불가합니다."
                QMessageBox.warning(self, '쿠폰 사용 불가', message2)
            #self.open_Bye_window()

    def payment(self):
        if self.Use_coupon == 'N' and self.coupon_value == 1 and self.number is not None and self.number != "":
            phone = self.number
            update_info, coupon = DB_main().update_infos(Flavor, Topping, self.price, int(phone), age, gender, self.Use_coupon)
            print(f"적립은 하지만 쿠폰 사용 x update_info | coupon {update_info}, {coupon}")
            self.open_Bye_window()
        
        elif self.Use_coupon == 'Y':
            print("쿠폰 사용한 고객은 이미 업데이트 했으니 그냥 바로 넘어감")
            self.open_Bye_window()

        elif self.coupon_value == 0:
            phone = None
            coupon = None
            Use_coupon = 'N'
            update_info, coupon = DB_main().update_infos(Flavor, Topping, self.price, phone, age, gender, Use_coupon)
            print(f"적립 안함 update_info | coupon {update_info}, {coupon}")
            self.open_Bye_window()
        else:
            print("payment Error--------------")        

class ByeWindow(QMainWindow, from_class_Bye):
    def __init__(self):
        super().__init__()
        self.setupUi(self)

        self.pushButton.clicked.connect(self.showValues)
        self.show()

    def showValues(self):
        global Flavor, Topping
        print(f"show Values ==> {Flavor}, {Topping}")
        result, stock, flavor_flag, topping_flag = DB_main().update_stock(Flavor, Topping)

        print(f"Final list ====================== {result}")

        self.tableWidget.setItem(0,0,QTableWidgetItem(str(result[0]))) #vanilla
        self.tableWidget.setItem(0,1,QTableWidgetItem(str(result[1]))) # choco
        self.tableWidget.setItem(0,2,QTableWidgetItem(str(result[2]))) # berry
        self.tableWidget.setItem(0,3,QTableWidgetItem(str(result[3]))) # topA
        self.tableWidget.setItem(0,4,QTableWidgetItem(str(result[4]))) # topB
        self.tableWidget.setItem(0,5,QTableWidgetItem(str(result[5]))) # topC
        self.tableWidget.setEditTriggers(QAbstractItemView.NoEditTriggers)