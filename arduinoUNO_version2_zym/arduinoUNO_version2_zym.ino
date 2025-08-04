//测速+接收GPS数据并显示 可以考虑拿GPS的测速和霍尔测速的两个结果比对后产出精度更高的速度

#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <U8glib.h>
#include <Wire.h>

//测速模块的参数
#define HALL_SENSOR_PIN A0  // 霍尔传感器连接到模拟引脚A0
#define WHEEL_DIAMETER 0.66 // 车轮直径（米），根据实际车轮尺寸修改
#define WHEEL_CIRCUMFERENCE (WHEEL_DIAMETER * PI) // 车轮周长（米）
#define HALL_THRESHOLD 600  // 霍尔传感器触发阈值（0-1023）
unsigned long lastTriggerTime = 0;  // 上次触发时间（毫秒）
unsigned long pulseInterval = 0;    // 脉冲间隔时间（毫秒）
float currentSpeed = 0.0;           // 当前速度（km/h）
float totalDistance = 0.0;          // 总里程（米）
int pulseCount = 0;                 // 脉冲计数

//通信模块的引脚和参数
RF24 radio(9, 10); 
const byte address[6] = "00001"; 
static float latitude =0.0;
static float longitude =0.0;

//显示模块的对象
U8GLIB_SSD1306_128X32 u8g(U8G_I2C_OPT_NONE);

//所有的函数
void setupOLED();
void Read_GPS();
void Read_Speed();
void Print_GPS_Speed(float latitude,float longitude,float currentSpeed,float totalDistance);

//主程序
void setup() {
  Serial.begin(9600);
  //通信模块设置
  radio.openReadingPipe(0, address);
  radio.setPALevel(RF24_PA_MIN); 
  radio.startListening(); 
  /*   通信模块检查报错-初始化
  if (!radio.begin()) {
    Serial.println("nRF24L01硬件连接失败！");
    while (1); 
  }
  */
  //测速模块设置
  pinMode(HALL_SENSOR_PIN, INPUT);
  lastTriggerTime = millis();
  //显示模块初始化
  setupOLED();
}

void loop() {
  Read_GPS();
  Read_Speed();
  Print_GPS_Speed(latitude,longitude,currentSpeed,totalDistance);
  delay(50);
}

//函数说明
void Read_GPS(){
  if (radio.available()) { 
    char received[50];
    radio.read(received, sizeof(received));
    char* token = strtok(received, ",");
    latitude = atof(token);
    token = strtok(NULL, ",");
    longitude = atof(token);
  }
}

void ReadPrint_Speed(){
  static int lastSensorValue = 0;
  int sensorValue = analogRead(HALL_SENSOR_PIN);
  if (lastSensorValue < HALL_THRESHOLD && sensorValue >= HALL_THRESHOLD) {
    unsigned long currentTime = millis();
    pulseInterval = currentTime - lastTriggerTime;
    lastTriggerTime = currentTime;
    if (pulseInterval > 50) {
      pulseCount++;
      currentSpeed = (WHEEL_CIRCUMFERENCE / (pulseInterval / 1000.0)) * 3.6;
      totalDistance += WHEEL_CIRCUMFERENCE;
    }
  }
  lastSensorValue = sensorValue;
  if (millis() - lastTriggerTime > 2000 && currentSpeed > 0) {
    currentSpeed = 0;
  }
}

void Print_GPS_Speed(float latitude,float longitude,float currentSpeed,float totalDistance){
  u8g.firstPage();
  do {
    u8g.setFont(u8g_font_6x10);
    // 显示纬度和经度（纬度在前）
    u8g.drawStr(0, 10, "纬度:");
    char buffer[16];
    dtostrf(latitude, 9, 6, buffer);  
    u8g.drawStr(30, 10, buffer);
    u8g.drawStr(0, 20, "经度:");
    dtostrf(longitude, 9, 6, buffer);
    u8g.drawStr(30, 20, buffer);
    // 显示速度和里程
    u8g.drawStr(0, 30, "速度:");
    dtostrf(currentSpeed, 5, 2, buffer);  
    u8g.drawStr(30, 30, buffer);
    u8g.drawStr(60, 30, "km/h");
    u8g.drawStr(0, 40, "里程:");
    dtostrf(totalDistance, 5, 2, buffer);
    u8g.drawStr(30, 40, buffer);
    u8g.drawStr(60, 40, "km");
  } while (u8g.nextPage());
}

void setupOLED() {
  if (u8g.getMode() == U8G_MODE_R3G3B2) {
    u8g.setColorIndex(255);  // 白色
  } else if (u8g.getMode() == U8G_MODE_GRAY2BIT) {
    u8g.setColorIndex(3);    // 最高灰度级别
  } else if (u8g.getMode() == U8G_MODE_BW) {
    u8g.setColorIndex(1);    // 黑色背景上的白色
  }
  u8g.firstPage();
  do {
    u8g.setFont(u8g_font_6x10);
    u8g.drawStr(0, 10, "GPS & Speed Monitor");
    u8g.drawStr(0, 25, "Initializing...");
  } while (u8g.nextPage());
  delay(1000);
}