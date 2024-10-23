# -*- coding: utf-8 -*-
"""
风扇控制程序 - WebUI 后端

"""

from flask import Flask, render_template
from flask_socketio import SocketIO, emit
import threading
import serial
import time
import sys
import os
import logging

# 设置日志格式
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')

# 确定资源文件路径
if getattr(sys, 'frozen', False):
    # 被打包为可执行文件
    basedir = sys._MEIPASS
else:
    basedir = os.path.abspath(os.path.dirname(__file__))

app = Flask(__name__, template_folder=os.path.join(basedir, 'templates'), static_folder=os.path.join(basedir, 'static'))
app.config['SECRET_KEY'] = 'secret!'
socketio = SocketIO(app)

# 串口设置
SERIAL_PORT = 'COM3'  # 根据实际情况修改
BAUD_RATE = 115200

# 全局变量
fan1_rpm = 0
fan2_rpm = 0
fan1_dc = 0
fan2_dc = 0
pwm_frequency_fan1 = 25000
pwm_frequency_fan2 = 25000

# 数据存储
data_time = []
data_fan1_dc = []
data_fan2_dc = []
data_fan1_rpm = []
data_fan2_rpm = []

max_data_points = 100  # 数据点数量限制

# 串口通信线程
ser = None  # 串口对象
ser_lock = threading.Lock()
auto_test_running = False  # 自动测试状态

def serial_thread():
    global fan1_rpm, fan2_rpm, fan1_dc, fan2_dc
    start_time = time.time()
    while True:
        try:
            line = ser.readline().decode('utf-8').strip()
            if line:
                logging.debug(f"Received from Arduino: {line}")
                data = line.split(',')
                for item in data:
                    key_value = item.split(':')
                    if len(key_value) != 2:
                        continue
                    key, value = key_value
                    if key == 'FAN1_RPM':
                        fan1_rpm = int(value)
                        data_fan1_rpm.append(fan1_rpm)
                    elif key == 'FAN2_RPM':
                        fan2_rpm = int(value)
                        data_fan2_rpm.append(fan2_rpm)
                    elif key == 'FAN1_DC':
                        fan1_dc = int(value)
                        data_fan1_dc.append(fan1_dc)
                    elif key == 'FAN2_DC':
                        fan2_dc = int(value)
                        data_fan2_dc.append(fan2_dc)
                current_time = time.time() - start_time
                data_time.append(current_time)
                # 限制数据长度，防止内存占用过大
                if len(data_time) > max_data_points:
                    data_time.pop(0)
                    data_fan1_dc.pop(0)
                    data_fan2_dc.pop(0)
                    data_fan1_rpm.pop(0)
                    data_fan2_rpm.pop(0)
                # 向前端发送数据
                socketio.emit('update_data', {
                    'fan1_rpm': fan1_rpm,
                    'fan2_rpm': fan2_rpm,
                    'fan1_dc': fan1_dc,
                    'fan2_dc': fan2_dc,
                    'time': current_time
                })
                # 输出当前状态
                logging.info(f"Fan1 RPM: {fan1_rpm}, Fan2 RPM: {fan2_rpm}, Fan1 DC: {fan1_dc}, Fan2 DC: {fan2_dc}")
        except Exception as e:
            logging.error(f"串口错误: {e}")
        time.sleep(0.1)

# 与前端的事件处理
@socketio.on('connect')
def handle_connect():
    logging.info('客户端已连接')

@socketio.on('disconnect')
def handle_disconnect():
    logging.info('客户端已断开连接')

@socketio.on('set_duty_cycle')
def handle_set_duty_cycle(data):
    global fan1_dc, fan2_dc
    fan1_dc = int(data['fan1_dc'])
    fan2_dc = int(data['fan2_dc'])
    send_duty_cycle()
    logging.info(f"Set Duty Cycle - Fan1: {fan1_dc}%, Fan2: {fan2_dc}%")

@socketio.on('set_pwm_frequency')
def handle_set_pwm_frequency(data):
    global pwm_frequency_fan1, pwm_frequency_fan2
    pwm_frequency_fan1 = int(data['fan1_freq'])
    pwm_frequency_fan2 = int(data['fan2_freq'])
    send_pwm_frequency()
    logging.info(f"Set PWM Frequency - Fan1: {pwm_frequency_fan1}Hz, Fan2: {pwm_frequency_fan2}Hz")

@socketio.on('toggle_auto_test')
def handle_toggle_auto_test(data):
    global auto_test_running
    auto_test = data['auto_test']
    if auto_test and not auto_test_running:
        auto_test_running = True
        threading.Thread(target=auto_test_thread, daemon=True).start()
        logging.info("Auto Test Started")
    elif not auto_test and auto_test_running:
        auto_test_running = False
        logging.info("Auto Test Stopped")

def send_duty_cycle():
    cmd = f"FAN1_DC:{fan1_dc},FAN2_DC:{fan2_dc},FAN1_FREQ:{pwm_frequency_fan1}\n"
    with ser_lock:
        ser.write(cmd.encode('utf-8'))
    logging.debug(f"Sent to Arduino: {cmd.strip()}")

def send_pwm_frequency():
    cmd = f"FAN1_FREQ:{pwm_frequency_fan1},FAN2_FREQ:{pwm_frequency_fan2},FAN1_DC:{fan1_dc},FAN2_DC:{fan2_dc}\n"
    with ser_lock:
        ser.write(cmd.encode('utf-8'))
    logging.debug(f"Sent to Arduino: {cmd.strip()}")

def auto_test_thread():
    global fan1_dc, fan2_dc, auto_test_running
    duty_values = list(range(0, 101, 5))
    while auto_test_running:
        for dc in duty_values:
            if not auto_test_running:
                break
            fan1_dc = dc
            fan2_dc = dc
            send_duty_cycle()
            socketio.emit('update_dc_sliders', {
                'fan1_dc': fan1_dc,
                'fan2_dc': fan2_dc
            })
            time.sleep(3)
    socketio.emit('auto_test_finished')
    logging.info("Auto Test Finished")

@app.route('/')
def index():
    return render_template('index.html')

def start_serial():
    global ser
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
        threading.Thread(target=serial_thread, daemon=True).start()
        logging.info(f"Connected to Arduino on {SERIAL_PORT} at {BAUD_RATE} baud.")
    except serial.SerialException:
        logging.error(f"无法打开串口 {SERIAL_PORT}")
        exit()

if __name__ == '__main__':
    start_serial()
    import webbrowser
    webbrowser.open('http://localhost:5000')
    socketio.run(app, host='0.0.0.0', port=5000)
