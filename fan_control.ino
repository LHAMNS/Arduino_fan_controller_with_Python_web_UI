// Arduino Mega 风扇控制程序 - 改进版

// 引角定义
const uint8_t FAN1_PWM_PIN = 11;    // 风扇1的PWM引角
const uint8_t FAN2_PWM_PIN = 5;     // 风扇2的PWM引角
const uint8_t FAN1_TACH_PIN = 2;    // 风扇1的测速引角
const uint8_t FAN2_TACH_PIN = 3;    // 风扇2的测速引角

// 全局变量
volatile uint32_t fan1_pulse_count = 0;  // 风扇1测速脉冲计数
volatile uint32_t fan2_pulse_count = 0;  // 风扇2测速脉冲计数
uint32_t fan1_rpm = 0;                   // 风扇1的RPM
uint32_t fan2_rpm = 0;                   // 风扇2的RPM
uint16_t pwm_frequency_fan1 = 25000;     // 风扇1的PWM频率
uint16_t pwm_frequency_fan2 = 25000;     // 风扇2的PWM频率
uint8_t duty_cycle_fan1 = 0;             // 风扇1的PWM占空比
uint8_t duty_cycle_fan2 = 0;             // 风扇2的PWM占空比

// 参数配置
unsigned long rpm_update_interval = 100;       // RPM计算时间间隔（0.1秒）
unsigned long serial_update_interval = 100;     // 串口数据发送时间间隔（0.05秒）
unsigned long last_rpm_time = 0;               // 上次计算风扇转速的时间戳
unsigned long last_serial_time = 0;            // 上次发送串口数据的时间戳

// 函数声明
void setupTimers();
void updateDutyCycleFan1(uint8_t duty);
void updateDutyCycleFan2(uint8_t duty);
void processSerial();
void sendDataToPython();
void verifyAndApplySettings();

// 设置Timer1和Timer3用于PWM输出
void setupTimers() {
  pinMode(FAN1_PWM_PIN, OUTPUT);
  pinMode(FAN2_PWM_PIN, OUTPUT);

  // 配置Timer1用于风扇1
  TCCR1A = 0;
  TCCR1B = 0;
  TCCR1A |= (1 << WGM11);
  TCCR1B |= (1 << WGM13) | (1 << WGM12);
  TCCR1A |= (1 << COM1A1);
  TCCR1B |= (1 << CS10);
  ICR1 = (16000000 / pwm_frequency_fan1) - 1;
  OCR1A = 0;

  // 配置Timer3用于风扇2
  TCCR3A = 0;
  TCCR3B = 0;
  TCCR3A |= (1 << WGM31);
  TCCR3B |= (1 << WGM33) | (1 << WGM32);
  TCCR3A |= (1 << COM3A1);
  TCCR3B |= (1 << CS30);
  ICR3 = (16000000 / pwm_frequency_fan2) - 1;
  OCR3A = 0;
}

void updateDutyCycleFan1(uint8_t duty) {
  if (duty > 100) duty = 100;
  OCR1A = (ICR1 * duty) / 100;
}

void updateDutyCycleFan2(uint8_t duty) {
  if (duty > 100) duty = 100;
  OCR3A = (ICR3 * duty) / 100;
}

// 外部中断服务程序，风扇1转速计数
void fan1Tach() {
  fan1_pulse_count++;
}

// 外部中断服务程序，风扇2转速计数
void fan2Tach() {
  fan2_pulse_count++;
}

void setup() {
  // 初始化串口通信
  Serial.begin(115200);

  // 初始化PWM
  setupTimers();

  // 设置转速输入引角
  pinMode(FAN1_TACH_PIN, INPUT_PULLUP);
  pinMode(FAN2_TACH_PIN, INPUT_PULLUP);

  // 附加外部中断
  attachInterrupt(digitalPinToInterrupt(FAN1_TACH_PIN), fan1Tach, RISING);
  attachInterrupt(digitalPinToInterrupt(FAN2_TACH_PIN), fan2Tach, RISING);
}

void loop() {
  unsigned long current_time = millis();

  // 每rpm_update_interval计算一次转速
  if (current_time - last_rpm_time >= rpm_update_interval) {
    noInterrupts();
    uint32_t pulses_fan1 = fan1_pulse_count;
    uint32_t pulses_fan2 = fan2_pulse_count;
    fan1_pulse_count = 0;
    fan2_pulse_count = 0;
    interrupts();

    // 计算RPM并乘以300
    fan1_rpm = pulses_fan1 * 300;
    fan2_rpm = pulses_fan2 * 300;

    last_rpm_time = current_time;
  }

  // 处理串口通信
  processSerial();

  // 每serial_update_interval发送数据给Python程序
  if (current_time - last_serial_time >= serial_update_interval) {
    sendDataToPython();
    last_serial_time = current_time;
  }
}

// 处理串口接收的数据
void processSerial() {
  static String inputBuffer = "";
  while (Serial.available() > 0) {
    char receivedChar = Serial.read();
    if (receivedChar == '\n') {
      inputBuffer.trim();
      if (inputBuffer.length() > 0) {
        // 进行指令解析
        Serial.println("Received: " + inputBuffer);

        int index1 = inputBuffer.indexOf(',');
        int index2 = inputBuffer.indexOf(',', index1 + 1);
        int index3 = inputBuffer.indexOf(',', index2 + 1);

        String cmd1 = inputBuffer.substring(0, index1);
        String cmd2 = inputBuffer.substring(index1 + 1, index2);
        String cmd3 = inputBuffer.substring(index2 + 1);

        if (cmd1.startsWith("FAN1_DC:")) {
          duty_cycle_fan1 = cmd1.substring(8).toInt();
          updateDutyCycleFan1(duty_cycle_fan1);
        }

        if (cmd2.startsWith("FAN2_DC:")) {
          duty_cycle_fan2 = cmd2.substring(8).toInt();
          updateDutyCycleFan2(duty_cycle_fan2);
        }

        if (cmd3.startsWith("FAN1_FREQ:")) {
          pwm_frequency_fan1 = cmd3.substring(10).toInt();
          if (pwm_frequency_fan1 < 20 || pwm_frequency_fan1 > 50000) {
            pwm_frequency_fan1 = 25000;
          }
          delay(5); // 延时以确保稳定
          noInterrupts();
          ICR1 = (16000000 / pwm_frequency_fan1) - 1;
          interrupts();
          updateDutyCycleFan1(duty_cycle_fan1);
        }

        if (cmd3.startsWith("FAN2_FREQ:")) {
          pwm_frequency_fan2 = cmd3.substring(10).toInt();
          if (pwm_frequency_fan2 < 20 || pwm_frequency_fan2 > 50000) {
            pwm_frequency_fan2 = 25000;
          }
          delay(5); // 延时以确保稳定
          noInterrupts();
          ICR3 = (16000000 / pwm_frequency_fan2) - 1;
          interrupts();
          updateDutyCycleFan2(duty_cycle_fan2);
        }

        verifyAndApplySettings();
      }
      inputBuffer = "";
    } else {
      inputBuffer += receivedChar;
    }
  }
}

// 验证并应用设置
void verifyAndApplySettings() {
  if (duty_cycle_fan1 > 100) duty_cycle_fan1 = 100;
  if (duty_cycle_fan2 > 100) duty_cycle_fan2 = 100;
  if (pwm_frequency_fan1 < 20 || pwm_frequency_fan1 > 50000) pwm_frequency_fan1 = 25000;
  if (pwm_frequency_fan2 < 20 || pwm_frequency_fan2 > 50000) pwm_frequency_fan2 = 25000;

  delay(5); // 延时以确保稳定
  noInterrupts();
  ICR1 = (16000000 / pwm_frequency_fan1) - 1;
  ICR3 = (16000000 / pwm_frequency_fan2) - 1;
  interrupts();

  updateDutyCycleFan1(duty_cycle_fan1);
  updateDutyCycleFan2(duty_cycle_fan2);
}

// 向Python程序发送数据
void sendDataToPython() {
  Serial.print("FAN1_RPM:");
  Serial.print(fan1_rpm);
  Serial.print(",FAN2_RPM:");
  Serial.print(fan2_rpm);
  Serial.print(",FAN1_DC:");
  Serial.print(duty_cycle_fan1);
  Serial.print(",FAN2_DC:");
  Serial.println(duty_cycle_fan2);
}
