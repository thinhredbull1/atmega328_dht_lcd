#include <Wire.h>               
#include <LiquidCrystal_I2C.h>   
#include <DHT.h>                 
#define run_every(t) for (static uint16_t last_; \
                          (uint16_t)(uint16_t(millis()) - last_) >= (t); \
                          last_ += (t))
LiquidCrystal_I2C lcd(0x27, 16, 2);
#define DHTTYPE DHT11
#define DHTPIN 2
#define BUT_UP 3
#define BUT_DOWN 4
#define BUT_CHANGE 8
#define RELAY_FAN 5
#define RELAY_COIL 6
#define RELAY_FOG 7
#define hum 0
#define temp 1
int param;
float para_desired[2];
DHT dht(DHTPIN, DHTTYPE);
void lcd_print(int x,int y,String str)
{
    lcd.setCursor(x, y);
    lcd.print(" "+str+" ");
}
bool getBut(int button)
{
    bool get_button=digitalRead(button);
    return get_button;
}
void setup()
{
    Serial.begin(9600);
    lcd.begin();
    lcd.backlight(); // Bật đèn nền LCD
    
    // Hiển thị thông báo khởi động
    lcd.setCursor(0, 0);
    lcd.print("DHT11 Sensor");
    lcd.setCursor(0, 1);
    lcd.print("Init...");
    pinMode(BUT_UP,INPUT_PULLUP);
    pinMode(BUT_CHANGE,INPUT_PULLUP);
    pinMode(BUT_DOWN,INPUT_PULLUP);
    pinMode(RELAY_FAN,OUTPUT);
    pinMode(RELAY_COIL,OUTPUT);
    pinMode(RELAY_FOG,OUTPUT);
    // Khởi động DHT11
    dht.begin();
    
    delay(4000); // Đợi khởi động
    float humidity = dht.readHumidity();
    float temperature = dht.readTemperature();
  
  // Kiểm tra nếu không đọc được dữ liệu
    if (isnan(humidity) || isnan(temperature)) {
        lcd.setCursor(0, 0);
        lcd.print("Sensor Error");
        return;
    }
    para_desired[hum]=humidity;
    para_desired[temp]=temperature;
    lcd.clear();
}
void loop()
{

  static unsigned long last_time=millis();
  float humidity = dht.readHumidity();
  float temperature = dht.readTemperature();
  if(getBut(BUT_UP)){
    para_desired[param]+=1;
  }
  else if(getBut(BUT_DOWN))
  {
    para_desired[param]-=1;
  }
  // Kiểm tra nếu không đọc được dữ liệu
  if (isnan(humidity) || isnan(temperature)) {
    lcd.setCursor(0, 0);
    lcd.print("Sensor Error");
    return;
  }
  lcd.setCursor(0, 0);
  lcd.print("Hum: ");
  lcd.print(humidity);
  lcd.print("%");
  lcd.setCursor(12, 0);
  lcd.print(hum_desired);
  lcd.print("%");
  // Hiển thị nhiệt độ
  lcd.setCursor(0, 1);
  lcd.print("Temp: ");
  lcd.print(temperature);
  lcd.print((char)223); // Ký tự độ
  lcd.print("C");
  lcd.setCursor(13, 1);
  lcd.print(temp_desired);
  Serial.println(millis()-last_time);
  last_time=millis();
}