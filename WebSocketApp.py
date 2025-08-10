from flask import Flask, render_template, send_file, redirect,Response
import cv2
from time import sleep, time
from gpiozero import Servo, RotaryEncoder,LED, Button, DistanceSensor,LineSensor
from gpiozero.pins.pigpio import PiGPIOFactory
import threading
# import RPi.GPIO as GPIO      # Optional if using RPi.GPIO

# from picamera2 import Picamera2  # Optional for camera
# import os
#from picamera2 import Picamera2                                          




timeout_set=5
latest_frame = None
start_time= time()


camera = cv2.VideoCapture(0)  # 0 = default USB webcam

#to improve streaming efficiency
camera.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
camera.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))  # If supported
camera.set(cv2.CAP_PROP_FPS, 20)  # Optional
#start_time=time()
def update_camera():
    global latest_frame
    print("timee out")
    start_time= time()
    #while time() - start_time < timeout_set:
    while True:
        ret, frame = camera.read()
        if not ret:
            continue
        latest_frame = frame
        sleep(0.03)  # ~10 FPS


threading.Thread(target=update_camera, daemon=True).start()

PWM=PiGPIOFactory()
#cam= Picamera2
Motor1=Servo(19,min_pulse_width=0.0004,max_pulse_width=0.0024,pin_factory=PWM)
Motor2=Servo(22,min_pulse_width=0.0005,max_pulse_width=0.0025,pin_factory=PWM)
led=LED(26)
esc_dir =Servo(23,min_pulse_width=1e-3,   # 1.0 ms
            max_pulse_width=2e-3,   # 2.0 ms
            frame_width=20e-3,      # 50 Hz
            pin_factory=PiGPIOFactory())

esc = Servo(24,min_pulse_width=1e-3,   # 1.0 ms
            max_pulse_width=2e-3,   # 2.0 ms
            frame_width=20e-3,      # 50 Hz
            pin_factory=PiGPIOFactory())

Motor1.value=0
Motor2.value=0.0
motor1_pos=0
motor2_pos=0.0
move_step=0.05
right_limit=-0.8
down_limit=-0.5
up_limit=0.7
app = Flask(__name__)


# Initialize your devices here
# motor = Motor(forward=17, backward=18)

@app.route('/')
def index():
    return render_template('index1.html')

# @app.route('/LiveStream')
# def LiveStream():
#     return render_template('LiveStream.html')
    
#threading.Thread(target=update_camera, daemon=True).start()    
@app.route('/Live')
def Live():
    video_feed()               
    return render_template('LiveStream.html')

@app.route('/LiveStream')
def video_feed():
    #                                                                                                                                                                                                                            update_camera()
    return Response(gen_frames(),mimetype='multipart/x-mixed-replace; boundary=frame')

def gen_frames():
    global latest_frame,start_time
    

    #while time() - start_time < timeout_set:
    while True:
        if latest_frame is None:
            continue
        # Encode frame as JPEG
        ret, latest_frame = camera.read()
        ret, jpeg = cv2.imencode('.jpg', latest_frame, [cv2.IMWRITE_JPEG_QUALITY, 50]) #0 #(worst quality, max compression) to 100 (best quality, large file)
        if not ret:
            continue
        frame_bytes = jpeg.tobytes()
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')

# --- Test ---
@app.route('/test', methods=['POST'])
def test():
    # Call your custom Python function
    result = test_function()
    return render_template('index1.html', result=result)

def test_function():
    # Replace this with any logic you want
    print("Web User Testing LED will blink")
    led.blink(0.2,0.2,3)
    return "Hello from Dr. L's Pi!"
    
    
# --- Motor Control Routes ---
@app.route('/forward')
def forward():
    m_left()
    return redirect('/')

    
def m_left():
    global motor1_pos
    
    motor1_pos=motor1_pos+move_step
    print(motor1_pos)
    if motor1_pos>0.9:
        Motor1.value=0.9
        motor1_pos=0.9
    else: Motor1.value=motor1_pos
@app.route('/reverse')
def reverse():
    m_right()
    return redirect('/')
    
def m_right():
    global motor1_pos
    
    motor1_pos=motor1_pos-move_step
    print(motor1_pos)
    if motor1_pos<right_limit:
        Motor1.value=right_limit
        motor1_pos=right_limit
    else: Motor1.value=motor1_pos

@app.route('/up')
def up():
    m_up()
    return redirect('/')
    
    
def m_up():

    global motor2_pos
    print(motor2_pos)
    motor2_pos=motor2_pos+move_step
    if motor2_pos>up_limit:
        Motor2.value=up_limit
        motor2_pos=up_limit
    else:  Motor2.value=motor2_pos



@app.route('/down')
def down():
    m_down()
    return redirect('/')
    
def m_down():
    global motor2_pos
    print(motor2_pos)
    motor2_pos=motor2_pos-move_step
    if motor2_pos<down_limit:
        Motor2.value=down_limit
        motor2_pos=down_limit
    else:  Motor2.value=motor2_pos
    
    
@app.route('/stop')
def stop():
    m_center()
    return redirect('/')
def m_center():
    Motor1.mid()
    Motor2.mid()

@app.route('/RC_forward')
def RC_forward():
    RCm_forward()
    return redirect('/')

def RCm_forward():
    esc.value=0.25
    sleep(0.5)
    esc.value=0
    print("motor is forward")

@app.route('/RC_reverse')
def RC_reverse():
    RCm_reverse()
    return redirect('/')

def RCm_reverse():
    esc.value=-0.21
    sleep(0.5)
    esc.value=0
    print("motor is reverse")
    
@app.route('/RC_left')
def RC_left():
    RCm_left()
    return redirect('/')

def RCm_left():
    esc_dir.value=0.7+offset
    print("motor is left")
offset=0.2   
@app.route('/RC_right')
def RC_right():
    RCm_right()
    return redirect('/')

def RCm_right():
    global offset
    esc_dir.value=-0.7
    print("motor is right")

@app.route('/RC_center')
def RC_center():
    RCm_center()
    return redirect('/')

def RCm_center():
    global offset
    esc_dir.value=0+offset
    print("motor is center")
#servo =AngularServo(18, initial_angle=0, min_pulse_width=0.0006, max_pulse_width=0.0023)

# --- Start the server ---
if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000,debug=False,threaded=True)




