# DB connection --> create & connect Table --> 저장된 age, gender txt 파일 불러오기 -->
# mydb.load_sales(age, gender) 맛 추천 --> 메뉴 선택 --> mydb.load_price(Topping) 계산 -->
# mydb.save_stock(Flavor Topping) 재고 업데이트 
import mysql.connector
import pandas as pd
from DB_manager import DB_Manager

import speech_API

class DB_main:
    def __init__(self):
        self.mydb = DB_Manager()

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
        with open('age_gender/latest_age_gender.txt','r') as f:
            info_txt = f.read()

        infos = info_txt.split('\t')  # 탭 문자로 분리

        if len(infos) >= 2:
            age = infos[0].strip()  # 첫 번째 요소를 age 변수에 저장하고 좌우 공백 제거
            gender = infos[1].strip()  # 두 번째 요소를 gender 변수에 저장하고 좌우 공백 제거
            print(f"{age} {gender} Load Success")
        else:
            print("Failed to load age and gender information from the file.")

        return age,gender

    def API_module(self, audio_file, text_file, response_time):
        # 추천 메뉴에 대한 응답 음성 인식
        if response_time == 3:
            print("추천드린 맛으로 선택하시겠습니까? (네/아니요)")
        if response_time == 5:
            print("메뉴 선택 후 말씀해주세요")
            print("예시) 딸기맛 하나랑 토핑 A로 부탁해")

        record_api = speech_API.Record_API(audio_file, text_file, response_time)
        record_api.run()

        if text_file:
            print(f"audio_file = {audio_file}. VoicetoText = {text_file} 성공적으로 저장됨")
        else:
            print("API_module --> text_file 생성되지 않음")
            audio_file, response_time = None

        return text_file, response_time
    
    def word_detect(self, text_file, response_time):
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
            with open(text_file,'r') as f:
                text_file = f.read()
            text_file = text_file.upper()
            flavor_topping = text_file.split(' ') # (ex. ['딸기맛', '하나랑', '토핑A로', '부탁해'])
            print(f"flavor_topping --> {flavor_topping}")
            flavor = [flavor for flavor in flavor_list 
                if any(flavor in word for word in flavor_topping)]
            
            print(f"인식된 flavor --> {flavor}")

            if len(flavor) == 1:
                if flavor[0] == flavor_list[0]:
                    Flavor = "berry"
                if flavor[0] == flavor_list[1]:
                    Flavor = "choco"
                if flavor[0] == flavor_list[2]:
                    Flavor = "vanilla"
            elif len(flavor) > 1:
                print("flavor -- 중복되었습니다.")
                Flavor = "error"
            else:
                print("flavor -- 다시 말씀해주세요")
                Flavor = "error"

            topping = [topping for topping in topping_list
                       if any(topping in word for word in flavor_topping)]

            print(f"인식된 topping --> {topping}")


            if len(topping) == 1:
                if topping[0] in [topping_list[0], topping_list[1]]:
                    Topping = "topA"
                if topping[0] in [topping_list[2], topping_list[3], topping_list[4]]:
                    Topping = "topB"
                if topping[0] in [topping_list[5], topping_list[6], topping_list[7]]:
                    Topping = "topC"

            elif len(topping) > 1:
                print("topping -- 중복되었습니다")
                Topping = "error"
            else:
                print("topping -- 다시 말씀해주세요")
                Topping = "error"

            return Flavor, Topping
    
    def main(self):
        age, gender = self.load_age_gender()

        most_flavor, most_topping = self.mydb.load_sales(age, gender)

        if most_flavor is None:
            most_flavor = self.None_flavor[age][gender]
            print(f"맛 추천 정보가 없어서 임의로 출력함 flavor = {self.None_flavor[age][gender]}")
        if most_topping is None:
            most_topping = self.None_topping[age][gender]
            print(f"토핑 추천 정보가 없어서 임의로 출력함 topping = {self.None_topping[age][gender]}")

        print (f"맛 추천 {most_flavor}, {most_topping}")

        print("################################################################################################")
        print("################################################################################################")
        print("################################################################################################")

        # Flavor = 'choco'
        # Topping = 'topC'
        # time = 3

        # Flavor = 'vanilla'
        # Topping = 'topA'

        YorN_audio = "YorN_response.wav"
        YorN_vtot = "YorN_response.txt"

        text_file, response_time = self.API_module(YorN_audio, YorN_vtot, 3)
        YorN = self.word_detect(text_file, response_time)

        cnt = 0 #다시 음성 요청할 때, 리밋하기 위한 카운터

        if YorN == 'Y':
            Flavor = most_flavor
            Topping = most_topping

        # 만약 음성으로 인식받는다고 하면 아래 파일 실행, 아니면 버튼에서 눌린 데이터값 불러와서 저장
        elif YorN == 'N':
            flavor_audio = "Flavor_Topping.wav"
            flavor_vtot = "Flavor_Topping.txt"
            # error값이 나오면 cnt = 0, 1, 2로 증가하는데 3이 되기 전까지 API_module과 word_detect 반복하는?
            cnt = 0 #다시 음성 요청할 때, 리밋하기 위한 카운터
            while cnt < 2:
                text_file, response_time = self.API_module(flavor_audio, flavor_vtot, 5)
                Flavor, Topping = self.word_detect(text_file, response_time)

                if Flavor == "error" or Topping == "error":
                    cnt += 1
                else:
                    break
        else:
            while cnt < 2:
                text_file, response_time = self.API_module(YorN_audio, YorN_vtot, 3)
                YorN = self.word_detect(text_file, response_time)

                if YorN == 'E':
                    cnt += 1
                else:
                    break

        topping_time = 5

        print("결제 전.....")

        stamp = input("쿠폰 적립하시겠습니까? (Y/N) = ").strip().upper()

        if stamp == 'Y':
            print("continue coupon")

            # Phone = 33331234
            Phone = 50302060

            coupon = self.mydb.load_customer_info(Phone)
            print(f"coupon 갯수 = {coupon}")

            Use_coupon = 'N'
            if coupon >= 10:
                Use_coupon = input("쿠폰 사용하시겠습니까? (Y/N) = ").strip().upper()
                print(f"{Use_coupon} --> 선택한 use coupon 값")
        else: 
            print("No stamp")
            Phone = None
            coupon = None
            Use_coupon = 'N'


        flavor_price, toppic_price = self.mydb.load_price(Topping)
        price = flavor_price + (toppic_price*topping_time)
        if Use_coupon == 'Y':
            price = 0
        
        print(f"price = {price}")


        save_sale = self.mydb.save_sales(Flavor, Topping, price, Phone, age, gender, Use_coupon)
        print(f"{save_sale}")

        if Phone:
            succeed = self.mydb.save_customer_info(Phone, age, gender, Use_coupon)
            print(f"고객 정보 업데이트 완료 --> {succeed}")
        else:
            print("쿠폰 적립 하지 않는다고 해서 손님 저장하지 않았음 ")

        stock_dict, flavor_flag, topping_flag = self.mydb.save_stock(Flavor, Topping)

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

    def run(self):
        self.mydb.DB_connect()
        self.mydb.create_table()
        self.main()

if __name__ == "__main__":
    DB_run = DB_main()
    DB_run.run()
