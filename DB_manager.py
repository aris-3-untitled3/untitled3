# 데이터베이스 연결, 생성, 삽입, 버튼 연동
#DB_table.sql 에서 date Foreign key 줄건지 말건지 고민
#FOREIGN KEY (date) REFERENCES sales (date)
import mysql.connector
import pandas as pd

class DB_Manager:
    def __init__(self):
        self.host = "localhost"
        self.user = "soo"
        self.database = "testdb"
        self.password = "1"
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

        if Phone in phone_numbers:
            query = "SELECT coupon FROM customer_info WHERE phone = %s;"
            self.cur.execute(query, (Phone, ))
            result = self.cur.fetchone()[0]
            return result
        else:
            print("No Customer data")
            result = None
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
        query = "SELECT phone FROM customer_info WHERE phone=%s;"
        self.cur.execute(query, (Phone, ))
        user_id = self.cur.fetchone()[0]
        
        return user_id
    

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

        query = "SELECT topping, COUNT(*) as count FROM sales WHERE age = %s and gender = %s GROUP BY topping;"
        self.cur.execute(query, (Age, Gender))
        result = self.cur.fetchall()
        topping_dict = {row[0]:row[1] for row in result}
        if not topping_dict:
            print("topping none")
            most_topping = None
        else:
            most_topping = max(topping_dict, key=topping_dict.get)
        
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

        return stock_dict, flavor_flag, topping_flag

