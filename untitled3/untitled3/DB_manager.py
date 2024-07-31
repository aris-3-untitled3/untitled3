# 데이터베이스 연결, 생성, 삽입, 버튼 연동
#DB_table.sql 에서 date Foreign key 줄건지 말건지 고민
#FOREIGN KEY (date) REFERENCES sales (date)
import mysql.connector
import pandas as pd
from datetime import datetime

class DB_Manager:
    def __init__(self):
        self.host = "localhost"
        self.user = "root"
        self.database = "untitled3"
        self.password = "0226"
        self.cur = None
        self.conn = None
    
    def DB_connect(self, database=None):
        if database is None:
            database = self.database
        try:
            self.conn = mysql.connector.connect(
                host=self.host,
                user=self.user,
                database=database,
                password=self.password
            )
            
        except mysql.connector.Error as err:
            if err.errno == mysql.connector.errorcode.ER_BAD_DB_ERROR:
                self.conn = mysql.connector.connect(
                    host=self.host,
                    user=self.user,
                    password=self.password
                )
                self.cur = self.conn.cursor()
                self.DB_create(database)
                self.conn.database = database
            else:
                raise
        self.cur = self.conn.cursor()
        
        
    def DB_create(self, database):
        try:
            self.cur.execute(f"CREATE DATABASE {database}")
        except mysql.connector.Error as err:
            print(f"Failed creating database: {err}")
            exit(1)
        print(f"Database {database} created successfully.")
        self.cur.execute(f"USE {database}")
        
        
    def create_table(self):
        self.execute_table("DB_table.sql")
        
    def create_sample_data(self):
        #customer_info
        # query = "INSERT INTO customer_info (phone, age, gender, coupon) VALUES (12341244, '19-29', 'F', 1);"
        # self.cur.execute(query)
        # query = "INSERT INTO customer_info (phone, age, gender, coupon) VALUES (12331223, '19-29', 'F', 1);"
        # self.cur.execute(query)
        # query = "INSERT INTO customer_info (phone, age, gender, coupon) VALUES (32331234, '19-29', 'F', 1);"
        # self.cur.execute(query)
        # query = "INSERT INTO customer_info (phone, age, gender, coupon) VALUES (12312363, '19-29', 'F', 1);"
        # self.cur.execute(query)
        # query = "INSERT INTO customer_info (phone, age, gender, coupon) VALUES (50302060, '19-29', 'F', 1);"
        # self.cur.execute(query)
        # self.conn.commit()

        #sales
        # query = "INSERT INTO sales (flavor, topping, price, phone, age, gender, use_coupon) VALUES ('berry', 'topA', 3650, 12341234, '19-29', 'F', 'N');"
        # self.cur.execute(query)
        # query = "INSERT INTO sales (flavor, topping, price, phone, age, gender, use_coupon) VALUES ('choco', 'topB', 3652, 12341223, '7-18', 'M', 'N');"
        # self.cur.execute(query)
        # query = "INSERT INTO sales (flavor, topping, price, phone, age, gender, use_coupon) VALUES ('vanilla', 'topA', 3590, 12331234, '19-29', 'F', 'N');"
        # self.cur.execute(query)
        # query = "INSERT INTO sales (flavor, topping, price, phone, age, gender, use_coupon) VALUES ('berry', 'topC', 3650, 50302060, '19-29', 'M', 'N');"
        # self.cur.execute(query)
        # query = "INSERT INTO sales (flavor, topping, price, phone, age, gender, use_coupon) VALUES ('vanilla', 'topC', 3650, 12341224, '19-29', 'F', 'N');"
        # self.cur.execute(query)
        # query = "INSERT INTO sales (flavor, topping, price, phone, age, gender, use_coupon) VALUES ('berry', 'topA', 0, 11341234, '19-29', 'F', 'Y');"
        # self.cur.execute(query)
        # self.conn.commit()

        #price
        query = "INSERT INTO price (topA , topB , topC) VALUES (18,15,20);"
        self.cur.execute(query)
        self.conn.commit()

        #stock
        query = "INSERT INTO stock (topC) VALUES (500);"
        self.cur.execute(query)
        self.conn.commit()

    def execute_table(self, file_path):
        with open(file_path, 'r') as f:
            table_file = f.read()
            
        commands = table_file.split(';')
        
        for command in commands:
            try:
                if command.strip() != '':
                    self.cur.execute(command)
            except mysql.connector.Error as err:
                print(f"Error occurred: {err}")
        
        self.conn.commit()
    
    #쿠폰 반환하는 함수
    def load_customer_info(self, Phone):
        query = "SELECT phone FROM customer_info;"
        self.cur.execute(query)
        result = self.cur.fetchall()
        phone_numbers = [row[0] for row in result]

        if Phone in phone_numbers: #기존 회원
            query = "SELECT coupon FROM customer_info WHERE phone = %s;"
            self.cur.execute(query, (Phone, ))
            result = self.cur.fetchone()[0]
            return result
        else:                       #신규 회원
            print("No Customer data")
            result = 0
            return result

    # 계산 후 손님 정보 저장하는 함수 (Age, Gender는 얼굴 인식 때, main에서 저장된 txt 파일에서 불러옴) & return = user_id // 최최최최최종
    def save_customer_info(self, Phone, Age, Gender, Use_coupon):
        # Phone 데이터 좌르륵 출력
        query = "SELECT phone FROM customer_info;"
        self.cur.execute(query)
        result = self.cur.fetchall()
        phone_numbers = [row[0] for row in result]

        if Phone in phone_numbers: #기존 회원
            # 결제 단계에서 (쿠폰 사용 유무 체크)
            if Use_coupon == 'Y':
                flag = 1
            else:
                flag = 0

            if flag == 0: #쿠폰 사용 안함 --> coupon + 1
                query = "UPDATE customer_info SET coupon=coupon+1 WHERE phone = %s;"
            elif flag == 1: #쿠폰 사용함 --> coupon 초기화
                query = "UPDATE customer_info SET coupon=coupon-10 WHERE phone = %s;"
    
            self.cur.execute(query, (Phone, )) #파라미터가 하나인 경우에도 튜플 형식으로 전달해야 됨
            self.conn.commit()
        else:                   #신규 회원
            query = "INSERT INTO customer_info (phone, age, gender, coupon) VALUES (%s, %s, %s, 1);"
            self.cur.execute(query, (Phone, Age, Gender))
            self.conn.commit()

        # 이건 그냥 return 준건데 사용 안해도 됨 ------------------
        query = "SELECT phone, coupon FROM customer_info WHERE phone=%s;"
        self.cur.execute(query, (Phone, ))
        result_update = self.cur.fetchone()
        user_id = result_update[0]
        coupon = result_update[1]
        
        return user_id, coupon
    
    #매출 저장 함수 // 매출 저장 -> 손님 정보 저장 or 업데이트
    def save_sales(self, Flavor, Topping, Price, Phone, Age, Gender, Use_coupon):
        query = "INSERT INTO sales (flavor, topping, price, phone, age, gender, use_coupon) VALUES (%s, %s, %s, %s, %s, %s, %s);"
        self.cur.execute(query, (Flavor, Topping, Price, Phone, Age, Gender, Use_coupon))
        self.conn.commit()

        query = "SELECT * FROM sales ORDER BY UserID DESC LIMIT 1;"
        self.cur.execute(query)
        result = self.cur.fetchone()
        if result:
            return result
        else:
            return None

    '''
    맛 추천을 위해 연령별&성별 ice cream, topping max 값을 불러옴
    return값은 가장 많은 flavor, topping 인데, 만약 한 번도 구매하지 않은 조합이면 None으로 반환
    None을 읽은 문장에서는 추천하는 UI 따로 띄우지 않기??
    ------------------------------------------------------------------------해결되지 않은 부분: max가 중복이면?

    선호도 추천시에 정보가 없거나 중복이면 팀원과 정한 데이터를 기반으로 추천해준다.
    '''

    def load_sales(self, Age, Gender):
        query = "SELECT flavor, COUNT(*) as count FROM sales WHERE age = %s and gender = %s GROUP BY flavor;"
        self.cur.execute(query, (Age, Gender))
        result = self.cur.fetchall()
        flavor_dict = {row[0]:row[1] for row in result}
        if not flavor_dict:
            print("flavor none")
            most_flavor = None
        else:
            most_flavor = max(flavor_dict, key=flavor_dict.get)
            # cnt_flavor = list(flavor_dict.values()).count(flavor_dict[most_flavor])
            # if cnt_flavor > 1:
            #     most_flavor = [flavor for flavor, count in flavor_dict.items() if count == flavor_dict[most_flavor]]

        query = "SELECT topping, COUNT(*) as count FROM sales WHERE age = %s and gender = %s GROUP BY topping;"
        self.cur.execute(query, (Age, Gender))
        result = self.cur.fetchall()
        topping_dict = {row[0]:row[1] for row in result}
        if not topping_dict:
            print("topping none")
            most_topping = None
        else:
            most_topping = max(topping_dict, key=topping_dict.get)
            # cnt_topping = list(topping_dict.values()).count(flavor_dict[most_flavor])
            # if cnt_topping > 1:
            #     most_topping = [topping for topping, count in topping_dict.items() if count == topping_dict[most_topping]]
        
        return most_flavor, most_topping

    # topping 초당 금액 가져오는 함수 --> return 값은 int형
    def load_price(self, Topping):
        query = f"SELECT ice_cream,{Topping} from price;"
        self.cur.execute(query)
        result = self.cur.fetchone()
        
        if result is not None:
            flavor_price = result[0]
            topping_price = result[1]
            return flavor_price, topping_price
        else:
            print("None Price ERROR")
            result = None
            return result
    
    '''
    ice cream, topping 재고 업데이트 하는 함수
    return --> stock_dict 딕셔너리 형으로 반환됨, 만약 값이 1회 기용비용 밖에 안남았다면, flag=on
    flag = on 이 되면 더 이상 손님을 받지 않고 관리자 모드? 로 들어감
    '''
    def save_stock(self, Flavor, Topping):
        flavor_flag = None #default값
        topping_flag = None

        if Topping == 'topA':
            dec = 18 # 한 번 공급될 때 10g씩 빠진다고 가정
        elif Topping == 'topB':
            dec = 15
        elif Topping == 'topC':
            dec = 20
        
        query = f"UPDATE stock SET {Flavor}={Flavor}-1, {Topping}={Topping}-{dec};"
        self.cur.execute(query)
        self.conn.commit()

        query = "SELECT vanilla, choco, berry, topA, topB, topC from stock;"
        self.cur.execute(query)
        result = self.cur.fetchone()

        if result is not None:
            stock_dict = {'vanilla':result[0],
                        'choco':result[1],
                        'berry':result[2],
                        'topA':result[3],
                        'topB':result[4],
                        'topC':result[5]}
        else:
            print("Stock ERROR")
            result = None
            return result
            
        # 1회 이상 공급이 더 이상 안 될 시 flag를 1로 바꿔서 return, 관리자에게 넘겨주는 코드가 될 것임
        if result[0] <= 1:
            flavor_flag = 'val'
        elif result[1] <= 1:
            flavor_flag = 'cho'
        elif result[2] <= 1:
            flavor_flag = 'ber'
        else:
            flavor_flag = None #default값
        
        if result[3] <= 18:
            topping_flag = 'A'
        elif result[4] <= 15:
            topping_flag = 'B'
        elif result[5] <= 20:
            topping_flag = 'C'
        else:
            topping_flag = None

        return result, stock_dict, flavor_flag, topping_flag
    
class DB_main:
    def __init__(self):
        self.mydb = DB_Manager()
        self.mydb.DB_connect()
        # self.mydb.create_table()
        # self.mydb.create_sample_data()

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
        with open('/home/jchj/Untitled3/src/untitled3/resource/latest_age_gender.txt','r') as f:
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

    # def API_module(self, audio_file, text_file, response_time):
    #     # 추천 메뉴에 대한 응답 음성 인식
    #     if response_time == 3:
    #         print("추천드린 맛으로 선택하시겠습니까? (네/아니요)")
    #         # time.sleep(0.5)
    #     if response_time == 5:
    #         print("메뉴 선택 후 말씀해주세요")
    #         print("예시) 딸기맛 하나랑 토핑 A로 부탁해")
    #         # time.sleep(0.5)

    #     # record_api = Record_API(audio_file, text_file, response_time)
    #     # flag = record_api.run()

    #     if text_file:
    #         print(f"audio_file = {audio_file}. VoicetoText = {text_file} 성공적으로 저장됨")
    #     else:
    #         print("API_module --> text_file 생성되지 않음")
    #         audio_file, response_time = None

    #     # return text_file, response_time, flag
    
    def word_detect(self, text_file, response_time, flag):
        flavor_list = ["딸기", 
                       "초코",
                       "바닐라"]
        
        topping_list = ["A", "에이",
                        "B", "비", "삐",
                        "C", "씨", "시"]

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
    
if __name__ == '__main__':
    # app = QApplication(sys.argv)
    main_window = DB_main()

    # main_window.show()
    # sys.exit(app.exec_())