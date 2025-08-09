#include <TinyGPS++.h>
#include <HardwareSerial.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Wire.h>
#include <math.h>
// ========== GPS 设置 ==========
TinyGPSPlus gps;
HardwareSerial GPS(2);
#define GPS_RX 16
#define GPS_TX 17
#define BUZZER_PIN 27
// ========== 新增：按钮与拍照控制 ==========
#define BUTTON_PIN 15    // 使用D15引脚
bool isTakingPhotos = false;  // 拍照状态
bool lastButtonState = HIGH;  // 按钮状态记录
unsigned long lastDebounceTime = 0;
unsigned long debounceDelay = 50;  // 去抖延迟
// ========== 无线通信设置 ==========
RF24 radio(2, 5);  // CE, CSN
const byte address[6] = "00001";

// ========== MPU6050 设置 ==========
const int MPU_ADDR = 0x68;
int16_t ax_raw, ay_raw, az_raw;
const float ACCEL_SCALE = 4096.0;

// ========== 数据结构 ==========
struct Packet {
  float lat;
  float lng;
  float ax;
  float ay;
  float az;
  bool alarm;  // 碰撞警报
};

// ========== 碰撞检测阈值 ==========
const float ACC_THRESHOLD = 2.5;  // g-force，大于此值视为碰撞

// ========== 滑动平均处理 ==========
#define FILTER_SIZE 5
float axBuf[FILTER_SIZE] = {0}, ayBuf[FILTER_SIZE] = {0}, azBuf[FILTER_SIZE] = {0};
int filterIndex = 0;

// ========== 初始化 ==========
void setup() {
  Serial.begin(115200);

  // GPS
  GPS.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX);
  pinMode(14,OUTPUT);
  pinMode(BUZZER_PIN,OUTPUT);
  // 新增：按钮初始化
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  
  // MPU6050
  Wire.begin();
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);  // 关闭睡眠模式
  Wire.write(0x00);
  Wire.endTransmission();
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1C);  // 设置加速度量程 ±2g
  Wire.write(0x10);
  Wire.endTransmission();

  // nRF24L01
  if (!radio.begin()) {
    Serial.println("nRF24L01 初始化失败！");
    while (1);
  }
  radio.openWritingPipe(address);
  radio.setPALevel(RF24_PA_LOW);
  radio.setChannel(100);
  radio.stopListening();
  Serial.println("ESP32 系统初始化完成");
}

// ========== 主循环 ==========
void loop() {
  // 新增：检查按钮状态
  checkButtonState();
  
  float ax = 0, ay = 0, az = 0;
  readAccelerometer(ax, ay, az);
  float accMag = sqrt(ax * ax + ay * ay + az * az);
  bool alarm = accMag > ACC_THRESHOLD;

  float lat = 0.0, lng = 0.0;
  readGPS(lat, lng);

  Packet pkt = {lat, lng, ax, ay, az, alarm};
  sendData(pkt);

  Serial.print("发送数据 -> ");
  Serial.print("lat: "); Serial.print(lat, 6);
  Serial.print(", lng: "); Serial.print(lng, 6);
  Serial.print("总加速度: ");Serial.print(sqrt(ax*ax+ay*ay+az*az),2);
  // 新增：显示拍照状态
  Serial.print(", 拍照: "); Serial.print(isTakingPhotos ? "开启" : "关闭");
  
  if(sqrt(ax*ax+ay*ay+az*az)>6)
  {
    while(1)
    {
      tone(BUZZER_PIN,1000);
      delay(100);
    }
  }
  Serial.print(", ALARM: "); Serial.println(alarm ? "YES" : "NO");
  
  // 修改：根据状态控制拍照模块
  digitalWrite(14, isTakingPhotos ? HIGH : LOW);
  
  delay(20);  // 调整发送频率
}

// ========== 新增：按钮状态检测函数 ==========
void checkButtonState() {
  int currentState = digitalRead(BUTTON_PIN);
  
  if (currentState != lastButtonState) {
    lastDebounceTime = millis();
  }
  
  if ((millis() - lastDebounceTime) > debounceDelay) {
    if (currentState == LOW) {  // 按钮按下
      isTakingPhotos = !isTakingPhotos;  // 切换状态
    }
  }
  
  lastButtonState = currentState;
}

// ========== GPS 读取 ==========
void readGPS(float &lat, float &lng) {
  while (GPS.available()) {
    char c = GPS.read();
    gps.encode(c);
  }
  if (gps.location.isUpdated()) {
    lat = gps.location.lat();
    lng = gps.location.lng();
  }
}

// ========== MPU6050 读取并滤波 ==========
void readAccelerometer(float &ax, float &ay, float &az) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 6, true);

  ax_raw = Wire.read() << 8 | Wire.read();
  ay_raw = Wire.read() << 8 | Wire.read();
  az_raw = Wire.read() << 8 | Wire.read();

  float rawAx = ax_raw / ACCEL_SCALE;
  float rawAy = ay_raw / ACCEL_SCALE;
  float rawAz = az_raw / ACCEL_SCALE;

  axBuf[filterIndex] = rawAx;
  ayBuf[filterIndex] = rawAy;
  azBuf[filterIndex] = rawAz;

  filterIndex = (filterIndex + 1) % FILTER_SIZE;

  ax = ay = az = 0;
  for (int i = 0; i < FILTER_SIZE; i++) {
    ax += axBuf[i];
    ay += ayBuf[i];
    az += azBuf[i];
  }
  ax /= FILTER_SIZE;
  ay /= FILTER_SIZE;
  az /= FILTER_SIZE;
}

// ========== 无线发送 ==========
void sendData(Packet &pkt) {
  radio.write(&pkt, sizeof(pkt));
}
