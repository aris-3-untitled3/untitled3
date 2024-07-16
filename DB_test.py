#test용
# DB connection --> create & connect Table --> 저장된 age, gender txt 파일 불러오기 -->
# mydb.load_sales(age, gender) 맛 추천 --> 메뉴 선택 --> mydb.load_price(Topping) 계산 -->
# mydb.save_stock(Flavor Topping) 재고 업데이트 
import mysql.connector
import pandas as pd
from DB_manager import DB_Manager

mydb = DB_Manager()
mydb.DB_connect()
mydb.create_table()

with open('latest_age_gender.txt','r') as f:
    info_txt = f.read()

infos = info_txt.split('\t')  # 탭 문자로 분리

if len(infos) >= 2:
    age = infos[0].strip()  # 첫 번째 요소를 age 변수에 저장하고 좌우 공백 제거
    gender = infos[1].strip()  # 두 번째 요소를 gender 변수에 저장하고 좌우 공백 제거
    print(f"{age} {gender} Load Success")
else:
    print("Failed to load age and gender information from the file.")

# 팀원과 임의로 정한 맛 추천 테이블
None_flavor = {'0-6': {'F':'berry', 'M':'choco'},
               '7-18': {'F':'berry', 'M':'choco'},
               '19-29': {'F':'choco', 'M':'choco'},
               '30-49': {'F': 'vanilla', 'M':'vanilla'},
                '50-': {'F': 'vanilla', 'M':'vanilla'}
                }

None_topping = {'0-6': {'F':'topA', 'M':'topC'},
               '7-18': {'F':'topB', 'M':'topA'},
               '19-29': {'F':'topA', 'M':'topC'},
               '30-49': {'F': 'topB', 'M':'topB'},
                '50-': {'F': 'topB', 'M':'topA'}
                }



most_flavor, most_topping = mydb.load_sales(age, gender)

if most_flavor is None:
    most_flavor = None_flavor[age][gender]
    print(f"맛 추천 정보가 없어서 임의로 출력함 flavor = {None_flavor[age][gender]}")
if most_topping is None:
    most_topping = None_topping[age][gender]
    print(f"토핑 추천 정보가 없어서 임의로 출력함 topping = {None_topping[age][gender]}")

print (f"맛 추천 {most_flavor}, {most_topping}")

# Flavor = 'choco'
# Topping = 'topC'
# time = 3

Flavor = 'vanilla'
Topping = 'topA'
time = 5

print("결제 전.....")

stamp = input("쿠폰 적립하시겠습니까? (Y/N) = ").strip().upper()

if stamp == 'Y':
    print("continue coupon")

    # Phone = 33331234
    Phone = 12341234

    coupon = mydb.load_customer_info(Phone)
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


flavor_price, toppic_price = mydb.load_price(Topping)
price = flavor_price + (toppic_price*time)
print(f"price = {price}")

if Use_coupon == 'Y':
    price = 0

save_sale = mydb.save_sales(Flavor, Topping, price, Phone, age, gender, Use_coupon)
print(f"{save_sale}")

if Phone:
    succeed = mydb.save_customer_info(Phone, age, gender, coupon)
else:
    print("쿠폰 적립 하지 않는다고 해서 손님 저장하지 않았음 ")

stock_dict, flavor_flag, topping_flag = mydb.save_stock(Flavor, Topping)

if stock_dict and flavor_flag and topping_flag is not None:
    print("재고 리스트\n")
    print(f"{stock_dict}")

    if flavor_flag is not None:
        print(f"Out of stock {flavor_flag}")
    if topping_flag is not None:
        print(f"Out of stock {topping_flag}")

    print("성공적으로 마쳤다!!")
else:
    print("Final ERROR------------------------------")
