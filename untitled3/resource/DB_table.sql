CREATE TABLE IF NOT EXISTS customer_info (
    UserID INT AUTO_INCREMENT PRIMARY KEY,
    phone INT(15) UNIQUE NOT NULL,
    age varchar(6) NOT NULL,
    gender varchar(1) NOT NULL,
    coupon int(2) default 1
);

CREATE TABLE IF NOT EXISTS price(
    date timestamp default now(),
    ice_cream int(4) default 3500,
    topA int(2) NOT NULL,
    topB int(2) NOT NULL,
    topC int(2) NOT NULL
);

CREATE TABLE IF NOT EXISTS sales(
    UserID INT AUTO_INCREMENT PRIMARY KEY,
    date timestamp default now(),
    flavor varchar(7) NOT NULL,
    topping varchar(4) NOT NULL,
    price int NOT NULL,
    phone INT(15),
    age varchar(6) NOT NULL,
    gender varchar(1) NOT NULL,
    use_coupon varchar(1) NOT NULL
);

CREATE TABLE IF NOT EXISTS stock(
    date timestamp default now(),
    vanilla int default 10,
    choco int default 10,
    berry int default 10,
    topA int default 500,
    topB int default 500,
    topC int default 500
);