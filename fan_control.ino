// Arduino Mega 风扇控制程序 - 优化版
// 作者：Assistant

// 引脚定义
const uint8_t FAN1_PWM_PIN = 11;    // 风扇1的PWM引脚，连接到Timer1，通道A，用于控制风扇转速
const uint8_t FAN2_PWM_PIN = 5;     // 风扇2的PWM引脚，连接到Timer3，通道A，用于控制风扇转速
const uint8_t FAN1_TACH_PIN = 2;    // 风扇1的测速引脚，外部中断0，用于测量风扇转速
const uint8_t FAN2_TACH_PIN = 3;    // 风扇2的测速引脚，外部中断1，用于测量风扇转速

// 全局变量
volatile uint32_t fan1_pulse_count = 0;  // 风扇1测速脉冲计数，每转产生多个脉冲，用于计算转速
volatile uint32_t fan2_pulse_count = 0;  // 风扇2测速脉冲计数
uint16_t fan1_rpm = 0;                   // 风扇1的每分钟转数（RPM），通过测速脉冲计算得出
uint16_t fan2_rpm = 0;                   // 风扇2的每分钟转数（RPM）
uint16_t pwm_frequency_fan1 = 25000;     // 风扇1的PWM频率，默认为25kHz，用于控制风扇速度
uint16_t pwm_frequency_fan2 = 25000;     // 风扇2的PWM频率，默认为25kHz
uint8_t duty_cycle_fan1 = 0;             // 风扇1的PWM占空比，范围0-100%，用于控制风扇转速
uint8_t duty_cycle_fan2 = 0;             // 风扇2的PWM占空比，范围0-100%

// 参数配置
unsigned long rpm_update_interval = 100;      // RPM计算的时间间隔（毫秒），用于定期计算风扇转速
unsigned long serial_update_interval = 500;   // 串口数据发送的时间间隔（毫秒），用于定期发送风扇数据
unsigned long last_rpm_time = 0;              // 上次计算风扇转速的时间戳，用于定期计算风扇转速
unsigned long last_serial_time = 0;           // 上次发送串口数据的时间戳，用于定期发送风扇数据

// 函数声明
void setupTimers();                      // 初始化Timer1和Timer3，用于PWM输出
void updateDutyCycleFan1(uint8_t duty);  // 更新风扇1的PWM占空比
void updateDutyCycleFan2(uint8_t duty);  // 更新风扇2的PWM占空比
void processSerial();                    // 处理从串口接收到的命令
void sendDataToPython();                 // 向Python程序发送风扇的实时数据
void verifyAndApplySettings();           // 验证并应用设置，确保占空比和频率在有效范围内

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

  // 设置转速输入引脚
  pinMode(FAN1_TACH_PIN, INPUT_PULLUP);
  pinMode(FAN2_TACH_PIN, INPUT_PULLUP);

  // 附加外部中断
  attachInterrupt(digitalPinToInterrupt(FAN1_TACH_PIN), fan1Tach, FALLING);
  attachInterrupt(digitalPinToInterrupt(FAN2_TACH_PIN), fan2Tach, FALLING);
}

void loop() {
  unsigned long current_time = millis();

  // 每rpm_update_interval计算一次转速，以适应高速风扇
  if (current_time - last_rpm_time >= rpm_update_interval) {
    noInterrupts();
    uint32_t pulses_fan1 = fan1_pulse_count;
    uint32_t pulses_fan2 = fan2_pulse_count;
    fan1_pulse_count = 0;
    fan2_pulse_count = 0;
    interrupts();

    // 计算RPM（每转4个脉冲，根据实际情况调整）
    fan1_rpm = (pulses_fan1 * 60) / 4 * (1000 / (current_time - last_rpm_time));
    fan2_rpm = (pulses_fan2 * 60) / 4 * (1000 / (current_time - last_rpm_time));

    last_rpm_time = current_time;
  }

  // 处理串口通信
  processSerial();

  // 定期发送数据给Python程序
  if (current_time - last_serial_time >= serial_update_interval) { // 每serial_update_interval发送一次
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

    // 验证并应用设置
    verifyAndApplySettings();
  }
}

// 验证并应用设置，确保占空比和频率正确
void verifyAndApplySettings() {
  if (duty_cycle_fan1 > 100) duty_cycle_fan1 = 100;
  if (duty_cycle_fan2 > 100) duty_cycle_fan2 = 100;
  if (pwm_frequency_fan1 < 20 || pwm_frequency_fan1 > 50000) pwm_frequency_fan1 = 25000;
  if (pwm_frequency_fan2 < 20 || pwm_frequency_fan2 > 50000) pwm_frequency_fan2 = 25000;

  // 更新PWM频率和占空比
  ICR1 = (16000000 / pwm_frequency_fan1) - 1;
  updateDutyCycleFan1(duty_cycle_fan1);

  ICR3 = (16000000 / pwm_frequency_fan2) - 1;
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
