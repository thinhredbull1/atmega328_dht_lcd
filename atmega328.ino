#include <Wire.h>               // Thư viện I2C
#include <LiquidCrystal_I2C.h>  // Thư viện LCD I2C
#include <DHT.h>                // Thư viện DHT
#define run_every(t) for (static uint16_t last_; \
                          (uint16_t)(uint16_t(millis()) - last_) >= (t); \
                          last_ += (t))
// Khởi tạo LCD I2C địa chỉ 0x27, 16 ký tự và 2 dòng
LiquidCrystal_I2C lcd(0x27, 16, 2);

// Chọn loại cảm biến DHT11
#define DHTTYPE DHT11

enum stateMain {
  HIGH_MODE = 0,
  LOW_MODE = 1,
  NORMAL = 2
};
#define DHTPIN 10
#define FAN_PIN 5
#define PHUN_SUONG 6
#define TEMP_COIL 7
#define BUT_SETUP 13
#define BUT_UP 12
#define BUT_DOWN 11
#define tolerance_temp 0.6
#define tolerance_hum 0.8
// Khởi tạo đối tượng DHT
DHT dht(DHTPIN, DHTTYPE);
int stateTemp = NORMAL;
int stateHum = NORMAL;
float setupTemp;
float setupHum;
float RealSetupT;
float RealSetupH;
bool error = 0;
bool hum_now;
int count_state_setup=0;

void ProcessBut() {
  if(count_state_setup!=0)
  {
    if (!digitalRead(BUT_UP)) {
      if (hum_now) setupTemp += 1;
      else setupHum += 1;
    } else if (!digitalRead(BUT_DOWN)) {
      if (hum_now) setupTemp -= 1;
      else setupHum -= 1;
    }
  }
}
void setup() {
  // Khởi tạo LCD
  lcd.begin();
  lcd.backlight();  // Bật đèn nền LCD

  // Hiển thị thông báo khởi động
  lcd.setCursor(0, 1);
  lcd.print("Wait DHT11 Sensor");
  lcd.setCursor(0, 0);
  lcd.print("Init...");
  pinMode(BUT_SETUP, INPUT_PULLUP);
  pinMode(BUT_DOWN, INPUT_PULLUP);
  pinMode(BUT_UP, INPUT_PULLUP);
  pinMode(FAN_PIN,OUTPUT);
  pinMode(PHUN_SUONG,OUTPUT);
  pinMode(TEMP_COIL,OUTPUT);
  // Khởi động DHT11
  dht.begin();
  // delay(2000);  // Đợi khởi động
  lcd.clear();
  float humidity = dht.readHumidity();
  float temperature = dht.readTemperature();
  setupHum = humidity;
  setupTemp = temperature;
  RealSetupT=setupTemp;
  RealSetupH=setupHum;
  // Kiểm tra nếu không đọc được dữ liệu
  if (isnan(humidity) || isnan(temperature)) {
    lcd.setCursor(0, 0);
    Serial.println("ERROR");
    lcd.print("Sensor Error");
    error = 1;
    // delay(5000);
  }
}

void loop() {
  if (error) return;
  run_every(150) ProcessBut();
  run_every(200)
  {
    if (!digitalRead(BUT_SETUP)){
      Serial.println("PRESS");
      count_state_setup+=1;
      if(count_state_setup>=3)count_state_setup=0;
    }
  }
  hum_now=count_state_setup==1?1:0;
  unsigned long start_time = millis();
  // Đọc giá trị nhiệt độ và độ ẩm từ DHT11
  float humidity = dht.readHumidity();
  float temperature = dht.readTemperature();
  if (isnan(humidity) || isnan(temperature)) {
    lcd.setCursor(0, 0);
    lcd.print("Sensor Error");
    return;
  }
  float diff_hum = RealSetupH - humidity;
  float diff_temp = RealSetupT - temperature;
  if (fabs(diff_hum) < tolerance_hum) stateHum = NORMAL;
  else if (diff_hum > tolerance_hum) stateHum = LOW_MODE;
  else if (diff_hum < -tolerance_hum) stateHum = HIGH_MODE;
  if (fabs(diff_temp) < tolerance_temp) stateTemp = NORMAL;
  else if (diff_temp > tolerance_temp) stateTemp = LOW_MODE;
  else if (diff_temp < -tolerance_temp) stateTemp = HIGH_MODE;
  // Kiểm tra nếu không đọc được dữ liệu
  run_every(600)
  {
  // Hiển thị độ ẩm
    lcd.noCursor();
    lcd.setCursor(0, 0);
    lcd.print("Hum:");
    lcd.print(humidity);
    lcd.print("%");
    lcd.setCursor(12, 0);
    lcd.print(setupHum);
  
    // Hiển thị nhiệt độ
    lcd.setCursor(0, 1);
    lcd.print("Tem:");
    lcd.print(temperature);
    lcd.print((char)223);  // Ký tự độ
    lcd.setCursor(12, 1);
    lcd.print(setupTemp);
    if(stateTemp==HIGH_MODE)
    {
      digitalWrite(FAN_PIN,1);
      digitalWrite(TEMP_COIL,0);
    }
    else if(stateTemp==NORMAL)
    {
      digitalWrite(FAN_PIN,0);
      digitalWrite(TEMP_COIL,0);
    }
    else if(stateTemp==LOW_MODE)
    {
      digitalWrite(FAN_PIN,0);
      digitalWrite(TEMP_COIL,1);
    }
    if(stateHum==LOW_MODE)digitalWrite(PHUN_SUONG,1);
    else digitalWrite(PHUN_SUONG,0);
  }
  if(count_state_setup!=0)
  {
    lcd.setCursor(11,hum_now);
    lcd.cursor();
  }
  else 
  {
    lcd.noCursor();
    // RealSetupT=setupTemp;
    // RealSetupH=setupHum;
  }
   RealSetupT=setupTemp;
    RealSetupH=setupHum;
}