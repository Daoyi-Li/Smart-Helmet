#include <Servo.h>
#include <OneButton.h>

const int NUM_MOTORS = 5;

// 舵机引脚（PWM）
const int servoPins[NUM_MOTORS] = {3, 5, 6, 9, 10};
// 压力传感器模拟引脚
const int pressurePins[NUM_MOTORS] = {A0, A1, A2, A3, A4};
// 舵机是否需要反转
const bool servoReversed[NUM_MOTORS] = {false, true, false, false, true};  // 你可以修改这些 true/false

const int PRESSURE_THRESHOLD = 300;
const int MIN_ANGLE = 0;
const int RESET_ANGLE = 140;

const int BUTTON_PIN = 2;

Servo servos[NUM_MOTORS];
int currentAngles[NUM_MOTORS] = {0};
bool adjustMode = false;

OneButton button(BUTTON_PIN, true);  // true 表示低电平触发（按钮接 GND）

void setup() {
  Serial.begin(9600);

  for (int i = 0; i < NUM_MOTORS; i++) {
    servos[i].attach(servoPins[i]);
    servos[i].write(RESET_ANGLE);
    currentAngles[i] = RESET_ANGLE;
  }

  button.attachClick(onButtonClick);

  Serial.println("智能头盔系统启动");
}

void loop() {
  button.tick();  // 检查按钮状态

  if (adjustMode) {
    for (int i = 0; i < NUM_MOTORS; i++) {
      int pressureVal = analogRead(pressurePins[i]);

      if (pressureVal > PRESSURE_THRESHOLD && currentAngles[i] > MIN_ANGLE) {
        currentAngles[i]-=5;
        servos[i].write(currentAngles[i]);
      }

      Serial.print("传感器 ");
      Serial.print(i);
      Serial.print(" 值: ");
      Serial.print(pressureVal);
      Serial.print(" | 舵机角度: ");
      Serial.println(currentAngles[i]);
    }
    delay(10);  // 延时防止过快转动
  }
}

// 处理按钮点击
void onButtonClick() {
  adjustMode = !adjustMode;

  if (adjustMode) {
    Serial.println("🟢 进入压力调节模式");
  } else {
    Serial.println("🔁 重置所有舵机角度");
    for (int i = 0; i < NUM_MOTORS; i++) {
      currentAngles[i] = RESET_ANGLE;
      servos[i].write(currentAngles[i]);
    }
  }
}

