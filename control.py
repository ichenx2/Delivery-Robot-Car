from flask import Flask, render_template, request, jsonify
import RPi.GPIO as GPIO
import time
import statistics
from hx711 import HX711
import battery

app = Flask(__name__)

# Create INA219 instance
ina219 = battery.INA219(addr=0x42)

# Set GPIO mode
GPIO.setmode(GPIO.BOARD)

# Motor control pins
IN1, IN2, IN3, IN4 = 11, 12, 15, 16 # Front wheels
IN1_2, IN2_2, IN3_2, IN4_2 = 29, 31, 36, 37 # Rear wheels
GPIO.setup([IN1, IN2, IN3, IN4, IN1_2, IN2_2, IN3_2, IN4_2], GPIO.OUT)

# Motor control functions
def stop():
    GPIO.output([IN1, IN2, IN3, IN4, IN1_2, IN2_2, IN3_2, IN4_2], GPIO.LOW)

def forward():
    GPIO.output(IN1, False)
    GPIO.output(IN2, True)
    GPIO.output(IN3, False)
    GPIO.output(IN4, True)
    GPIO.output(IN1_2, False)
    GPIO.output(IN2_2, True)
    GPIO.output(IN3_2, False)
    GPIO.output(IN4_2, True)

def backward():
    GPIO.output(IN1, True)
    GPIO.output(IN2, False)
    GPIO.output(IN3, True)
    GPIO.output(IN4, False)
    GPIO.output(IN1_2, True)
    GPIO.output(IN2_2, False)
    GPIO.output(IN3_2, True)
    GPIO.output(IN4_2, False)

def turn_left():
    GPIO.output(IN1, False)
    GPIO.output(IN2, True)
    GPIO.output(IN3, True)
    GPIO.output(IN4, False)
    GPIO.output(IN1_2, False)
    GPIO.output(IN2_2, True)
    GPIO.output(IN3_2, True)
    GPIO.output(IN4_2, False)

def turn_right():
    GPIO.output(IN1, True)
    GPIO.output(IN2, False)
    GPIO.output(IN3, False)
    GPIO.output(IN4, True)
    GPIO.output(IN1_2, True)
    GPIO.output(IN2_2, False)
    GPIO.output(IN3_2, False)
    GPIO.output(IN4_2, True)

# Initialize HX711
hx = HX711(21, 23)
hx.set_reading_format("MSB", "MSB")
hx.reset()
hx.tare()

# Define threshold
THRESHOLD = 1000
object_present = False
last_check_time = 0
DEBOUNCE_TIME = 0.5
stable_start_time = None
STABLE_TIME = 0.5

# Initialize sensor state
initial_weight = hx.get_weight(1)
object_present = initial_weight > THRESHOLD

# Weight check function
def check_weight():
    global object_present, last_check_time, stable_start_time
    raw_val = hx.get_weight(1)
    current_time = time.time()

    if current_time - last_check_time < DEBOUNCE_TIME:
        return None

    last_check_time = current_time

    if raw_val > THRESHOLD:
        if not object_present:
            if stable_start_time is None:
                stable_start_time = current_time
            elif current_time - stable_start_time >= STABLE_TIME:
                object_present = True
                stable_start_time = None
                return "Object placed"
        else:
            stable_start_time = None
    else:
        if object_present:
            if stable_start_time is None:
                stable_start_time = current_time
            elif current_time - stable_start_time >= STABLE_TIME:
                object_present = False
                stable_start_time = None
                return "Object removed"
        else:
            stable_start_time = None

    return None

# Ultrasonic sensor setup
trigger_pin = 22
echo_pin = 18
number_of_samples = 5
sample_sleep = 0.01
calibration1 = 30
calibration2 = 1750
time_out = 0.05
GPIO.setup(trigger_pin, GPIO.OUT)
GPIO.setup(echo_pin, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

samples_list = []
stack = []

# Distance measurement callback
def timer_call(channel):
    now = time.monotonic()
    stack.append(now)

# Trigger ultrasonic pulse
def trigger():
    GPIO.output(trigger_pin, GPIO.HIGH)
    time.sleep(0.00001) # 10-microsecond pulse
    GPIO.output(trigger_pin, GPIO.LOW)

# Check distance
def check_distance():
    samples_list.clear()
    while len(samples_list) < number_of_samples:
        trigger()
        while len(stack) < 2: # waiting for the stack to fill with a start and end time
            start = time.monotonic()
            while time.monotonic() < start + time_out:
                pass
            trigger()

        if len(stack) == 2:
            samples_list.append(stack.pop() - stack.pop())
        elif len(stack) > 2:
            stack.clear()
        time.sleep(sample_sleep)

    return (statistics.median(samples_list) * 1000000 * calibration1 / calibration2)

GPIO.add_event_detect(echo_pin, GPIO.BOTH, callback=timer_call)

@app.route('/')
def index():
    return render_template('index.html')

@app.route('/forward', methods=['POST'])
def handle_forward():
    distance = check_distance()
    if distance < 20:
        return jsonify({'status': 'error', 'message': 'Too close! Moving forward might cause a collision.'})
    forward()
    time.sleep(1)
    stop()
    return jsonify({'status': 'success', 'message': 'Vehicle moved forward'})

@app.route('/backward', methods=['POST'])
def handle_backward():
    backward()
    time.sleep(1)
    stop()
    return jsonify({'status': 'success', 'message': 'Vehicle moved backward'})

@app.route('/left', methods=['POST'])
def handle_left():
    distance = check_distance()
    if distance < 20:
        return jsonify({'status': 'error', 'message': 'Too close! Turning left might cause a collision.'})
    turn_left()
    time.sleep(1)
    stop()
    return jsonify({'status': 'success', 'message': 'Vehicle turned left'})

@app.route('/right', methods=['POST'])
def handle_right():
    distance = check_distance()
    if distance < 20:
        return jsonify({'status': 'error', 'message': 'Too close! Turning right might cause a collision.'})
    turn_right()
    time.sleep(1)
    stop()
    return jsonify({'status': 'success', 'message': 'Vehicle turned right'})

@app.route('/stop', methods=['POST'])
def handle_stop():
    stop()
    return jsonify({'status': 'success', 'message': 'Vehicle stopped'})

@app.route('/status', methods=['GET'])
def get_status():
    status = check_weight()
    if status == "Object placed":
        return jsonify({'status': "Object currently placed", 'alert': "Object detected"})
    elif status == "Object removed":
        return jsonify({'status': "No object placed", 'alert': "Object removed"})

    current_status = "Object currently placed" if object_present else "No object placed"
    return jsonify({'status': current_status, 'alert': None})

@app.route('/battery', methods=['GET'])
def get_battery_status():
    try:
        bus_voltage = ina219.getBusVoltage_V()  # Get bus voltage
        p = (bus_voltage - 6) / 2.4 * 100  # Calculate remaining battery percentage
        p = max(0, min(100, p))  # Limit to 0-100% range
        return jsonify({'voltage': round(bus_voltage, 2), 'percent': round(p, 1)})
    except Exception as e:
        return jsonify({'error': str(e)}), 500

if __name__ == '__main__':
    try:
        app.run(host='0.0.0.0', port=5000, debug=True)
    except KeyboardInterrupt:
        GPIO.cleanup()