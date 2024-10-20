// Arduino Mega 风扇控制程序
// 作者：Assistant

// 引脚定义
const uint8_t FAN1_PWM_PIN = 11;    // 使用Timer1，通道A
const uint8_t FAN2_PWM_PIN = 5;     // 使用Timer3，通道A
const uint8_t FAN1_TACH_PIN = 2;    // 外部中断0
const uint8_t FAN2_TACH_PIN = 3;    // 外部中断1

// 全局变量
volatile uint16_t fan1_pulse_count = 0;
volatile uint16_t fan2_pulse_count = 0;
uint16_t fan1_rpm = 0;
uint16_t fan2_rpm = 0;
uint16_t pwm_frequency_fan1 = 25000; // 默认25kHz
uint16_t pwm_frequency_fan2 = 25000; // 默认25kHz
uint8_t duty_cycle_fan1 = 0; // 0-100%
uint8_t duty_cycle_fan2 = 0; // 0-100%

unsigned long last_rpm_time = 0;
unsigned long last_serial_time = 0;

// 函数声明
void setupTimers();
void updateDutyCycleFan1(uint8_t duty);
void updateDutyCycleFan2(uint8_t duty);
void processSerial();
void sendDataToPython();

// 设置Timer1和Timer3用于PWM输出
void setupTimers() {
  // 设置PWM引脚为输出
  pinMode(FAN1_PWM_PIN, OUTPUT);
  pinMode(FAN2_PWM_PIN, OUTPUT);

  // 配置Timer1用于风扇1
  // 清除控制寄存器
  TCCR1A = 0;
  TCCR1B = 0;

  // 设置Fast PWM模式，使用ICR1作为TOP值
  TCCR1A |= (1 << WGM11);
  TCCR1B |= (1 << WGM13) | (1 << WGM12);

  // 设置非反相PWM输出
  TCCR1A |= (1 << COM1A1); // 通道A（Pin 11）

  // 设置无预分频
  TCCR1B |= (1 << CS10);

  // 设置PWM频率
  ICR1 = (16000000 / pwm_frequency_fan1) - 1;

  // 初始占空比为0
  OCR1A = 0;

  // 配置Timer3用于风扇2
  // 清除控制寄存器
  TCCR3A = 0;
  TCCR3B = 0;

  // 设置Fast PWM模式，使用ICR3作为TOP值
  TCCR3A |= (1 << WGM31);
  TCCR3B |= (1 << WGM33) | (1 << WGM32);

  // 设置非反相PWM输出
  TCCR3A |= (1 << COM3A1); // 通道A（Pin 5）

  // 设置无预分频
  TCCR3B |= (1 << CS30);

  // 设置PWM频率
  ICR3 = (16000000 / pwm_frequency_fan2) - 1;

  // 初始占空比为0
  OCR3A = 0;
}

void updateDutyCycleFan1(uint8_t duty) {
  if (duty > 100) duty = 100;
  OCR1A = ((ICR1 + 1) * duty) / 100 - 1;
}

void updateDutyCycleFan2(uint8_t duty) {
  if (duty > 100) duty = 100;
  OCR3A = ((ICR3 + 1) * duty) / 100 - 1;
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

  // 设置转速输入引脚
  pinMode(FAN1_TACH_PIN, INPUT_PULLUP);
  pinMode(FAN2_TACH_PIN, INPUT_PULLUP);

  // 附加外部中断
  attachInterrupt(digitalPinToInterrupt(FAN1_TACH_PIN), fan1Tach, FALLING);
  attachInterrupt(digitalPinToInterrupt(FAN2_TACH_PIN), fan2Tach, FALLING);
}

void loop() {
  unsigned long current_time = millis();

  // 每1000ms计算一次转速
  if (current_time - last_rpm_time >= 1000) {
    noInterrupts();
    uint16_t pulses_fan1 = fan1_pulse_count;
    uint16_t pulses_fan2 = fan2_pulse_count;
    fan1_pulse_count = 0;
    fan2_pulse_count = 0;
    interrupts();

    // 计算RPM（每转2个脉冲，一般风扇为2或4，根据实际情况调整）
    fan1_rpm = (pulses_fan1 * 60) / 2;
    fan2_rpm = (pulses_fan2 * 60) / 2;

    last_rpm_time = current_time;
  }

  // 处理串口通信
  processSerial();

  // 定期发送数据给Python程序
  if (current_time - last_serial_time >= 500) { // 每500ms发送一次
    sendDataToPython();
    last_serial_time = current_time;
  }
}

// 处理串口接收的数据
void processSerial() {
  while (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim();
    if (input.length() == 0) return;

    // 调试输出接收到的命令
    Serial.println("Received: " + input);

    // 解析命令，使用逗号分隔
    int index1 = input.indexOf(',');
    int index2 = input.indexOf(',', index1 + 1);
    int index3 = input.indexOf(',', index2 + 1);

    String cmd1 = input.substring(0, index1);
    String cmd2 = input.substring(index1 + 1, index2);
    String cmd3 = input.substring(index2 + 1);

    // 处理FAN1_DC命令
    if (cmd1.startsWith("FAN1_DC:")) {
      duty_cycle_fan1 = cmd1.substring(8).toInt();
      updateDutyCycleFan1(duty_cycle_fan1);
    }

    // 处理FAN2_DC命令
    if (cmd2.startsWith("FAN2_DC:")) {
      duty_cycle_fan2 = cmd2.substring(8).toInt();
      updateDutyCycleFan2(duty_cycle_fan2);
    }

    // 处理FAN1_FREQ命令
    if (cmd3.startsWith("FAN1_FREQ:")) {
      pwm_frequency_fan1 = cmd3.substring(10).toInt();
      if (pwm_frequency_fan1 < 20 || pwm_frequency_fan1 > 50000) {
        pwm_frequency_fan1 = 25000; // 限制频率范围，防止异常
      }
      // 更新PWM频率
      ICR1 = (16000000 / pwm_frequency_fan1) - 1;
      updateDutyCycleFan1(duty_cycle_fan1);
    }

    // 检查是否有FAN2_FREQ命令
    if (cmd3.startsWith("FAN2_FREQ:")) {
      pwm_frequency_fan2 = cmd3.substring(10).toInt();
      if (pwm_frequency_fan2 < 20 || pwm_frequency_fan2 > 50000) {
        pwm_frequency_fan2 = 25000;
      }
      // 更新PWM频率
      ICR3 = (16000000 / pwm_frequency_fan2) - 1;
      updateDutyCycleFan2(duty_cycle_fan2);
    }
  }
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

  // 调试输出发送的数据
  Serial.print("Sent: ");
  Serial.print("FAN1_RPM:");
  Serial.print(fan1_rpm);
  Serial.print(",FAN2_RPM:");
  Serial.print(fan2_rpm);
  Serial.print(",FAN1_DC:");
  Serial.print(duty_cycle_fan1);
  Serial.print(",FAN2_DC:");
  Serial.println(duty_cycle_fan2);
}
