import cv2
import numpy as np
import time
import RPi.GPIO as GPIO

STOP = 0
FORWARD = 1
BACKWARD = 2

CH1 = 0
CH2 = 1

OUTPUT = 1
INPUT = 0

HIGH = 1
LOW = 0

ENA = 26
ENB = 11

IN1 = 19
IN2 = 13
IN3 = 6
IN4 = 5

pin_pwm = 18
frequency = 50

GPIO.setmode(GPIO.BCM)
GPIO.setup(pin_pwm, GPIO.OUT)
GPIO.setup(27, GPIO.OUT)
GPIO.setup(17, GPIO.IN)
p = GPIO.PWM(pin_pwm, frequency)

max_time_out = 1979


def distance_check(trig, echo):
    fail = False

    GPIO.output(trig, GPIO.HIGH)
    time.sleep(0.00001)
    GPIO.output(trig, GPIO.LOW)
    stop = 0
    start = 0

    time_out = time.time()
    while GPIO.input(echo) == GPIO.LOW:
        start = time.time()
        if (((start - time_out) * 1000000) >= max_time_out):
            fail = True
            break
        if fail:
            continue

    while GPIO.input(echo) == GPIO.HIGH:
        stop = time.time()
        if (((stop - start) * 1000000) >= max_time_out):
            fail = True
            break
        if fail:
            continue

    duration = stop - start
    distance = (duration * 340 * 100) / 2

    return distance


def setPinConfig(EN, INA, INB):
    GPIO.setup(EN, GPIO.OUT)
    GPIO.setup(INA, GPIO.OUT)
    GPIO.setup(INB, GPIO.OUT)

    pwm = GPIO.PWM(EN, 50)
    pwm.start(0)

    return pwm


def setMotorControl(pwm, INA, INB, speed, stat):
    pwm.ChangeDutyCycle(speed)

    if stat == FORWARD:
        GPIO.output(INA, HIGH)
        GPIO.output(INB, LOW)

    elif stat == BACKWARD:
        GPIO.output(INA, LOW)
        GPIO.output(INB, HIGH)

    elif stat == STOP:
        GPIO.output(INA, LOW)
        GPIO.output(INB, LOW)


def setMotor(ch, speed, stat):
    if ch == CH1:
        setMotorControl(pwmA, IN1, IN2, speed, stat)
    else:
        setMotorControl(pwmB, IN3, IN4, speed, stat)

    return stat


# lower_red1 = np.array([161, 102, 102])
# upper_red1 = np.array([182, 255, 255])
#
# lower_red2 = np.array([0, 102, 102])
# upper_red2 = np.array([10, 255, 255])
#
# lower_red3 = np.array([127, 102, 102])
# upper_red3 = np.array([137, 255, 255])


def make_coordinates(img, line_parameters):
    try:
        slope, intercept = line_parameters

        y1 = img.shape[0]
        y2 = int(y1 * (2 / 3))
        x1 = int((y1 - intercept) / slope)
        x2 = int((y2 - intercept) / slope)

        return [x1, y1, x2, y2]
    except:
        return None


def average_slope_intercept(img, lines):
    left_fit = []
    right_fit = []

    for line in lines:
        x1, y1, x2, y2 = line.reshape(4)

        x = np.array([x1, x2])
        y = np.array([y1, y2])
        A = np.stack([x, np.ones(len(x))]).T

        slope, intercept = np.linalg.lstsq(A, y, rcond=None)[0]

        x_coord = -((intercept - 160) / slope)

        if x_coord < 100:
            left_fit.append((slope, intercept))

        elif x_coord > 100:
            right_fit.append((slope, intercept))

    left_fit_average = np.mean(left_fit, 0)
    right_fit_average = np.mean(right_fit, 0)

    left_line = make_coordinates(img, left_fit_average)
    left_line[0] = 63
    right_line = make_coordinates(img, right_fit_average)
    right_line[0] = 137

    return [left_line], [right_line]


def display_lines(img, lines):
    line_image = np.zeros_like(img)

    for line in lines:
        x1, y1, x2, y2 = line.reshape(4)
        cv2.line(line_image, (x1, y1), (x2, y2), (255, 255, 0), 10)

    return line_image


def make_canny(img):
    blur = cv2.GaussianBlur(img, (5, 5), 0)
    gray = cv2.cvtColor(blur, cv2.COLOR_BGR2GRAY)
    _, binary = cv2.threshold(gray, 80, 255, cv2.THRESH_BINARY_INV)
    canny_image = cv2.Canny(binary, 80, 120)

    return canny_image


cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 128)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 96)
cap.set(cv2.CAP_PROP_FPS, 20)

h = 160
w = 200

pts1 = np.float32([[30, 87], [45, 52], [83, 52], [98, 87]])
pts2 = np.float32([[50, 160], [50, 0], [150, 0], [150, 160]])
M = cv2.getPerspectiveTransform(pts1, pts2)

l1 = 0
l2 = 0
l1_copy = []
l2_copy = []

i = 7.5
p.start(i)

pwmA = setPinConfig(ENA, IN1, IN2)  # left motor pin setting
pwmB = setPinConfig(ENB, IN3, IN4)  # right motor pin setting

stat1 = setMotor(CH1, 30, STOP)  # left motor
stat2 = setMotor(CH2, 30, STOP)  # right motor

di = 20
try:
    while (True):
        ret, frame = cap.read()
        img2 = cv2.warpPerspective(frame, M, (w, h))

        last_time = time.time()
        result_distance = distance_check(27, 17)
        print(result_distance)

        if result_distance < 20 and result_distance > 0:
            stat1 = setMotor(CH1, 0, STOP)
            stat2 = setMotor(CH2, 0, STOP)

            continue
        else:
            pass

        try:
            canny = make_canny(img2)
            lines = cv2.HoughLinesP(canny, 3, np.pi / 180, 75, np.array([]), 25, 10)
            AA = int(100 + ((i - 7.5) * 10))

            if l1 == 0 and l2 == 0:
                l1, l2 = average_slope_intercept(canny, lines)
            else:
                l1_copy, l2_copy = average_slope_intercept(canny, lines)

            if l1_copy is not None:
                try:
                    if l1_copy[0][2] > l1[0][2] + di or l1_copy[0][2] < l1[0][2] - di:
                        l1 = l1
                    else:
                        l1 = l1_copy

                    if l2_copy is not None:
                        if l2_copy[0][2] > l2[0][2] + di or l2_copy[0][2] < l2[0][2] - di:
                            l2 = l2
                        else:
                            l2 = l2_copy

                except:
                    pass

            elif l1_copy is None:
                try:
                    if l2_copy is not None:
                        if l2_copy[0][2] > l2[0][2] + di or l2_copy[0][2] < l2[0][2] - di:
                            l2 = l2
                        else:
                            l2 = l2_copy

                except:
                    pass

            x2_coord_average = (l2[0][2] + l1[0][2]) / 2

            if x2_coord_average > 122 or x2_coord_average < 78:
                if AA > x2_coord_average + 2 or AA < x2_coord_average - 2:
                    i = 7.5 + (((x2_coord_average - 25) - AA) / 24)
                    i = round(i, 1)
                    if i > 10:
                        i = 10
                    elif i < 5:
                        i = 5
                    p.ChangeDutyCycle(i)

                    print("case 4")

            elif x2_coord_average > 113 or x2_coord_average < 87:
                if AA > x2_coord_average + 2 or AA < x2_coord_average - 2:
                    i = 7.5 + (((x2_coord_average - 25) - AA) / 27)
                    i = round(i, 1)
                    if i > 10:
                        i = 10
                    elif i < 5:
                        i = 5
                    p.ChangeDutyCycle(i)

                    print("case 3")

            elif x2_coord_average > 107 or x2_coord_average < 93:
                if AA > x2_coord_average + 1 or AA < x2_coord_average - 1:
                    i = 7.5 + (((x2_coord_average - 25) - AA) / 30)
                    i = round(i, 1)
                    if i > 10: # 서브모터 질질끌리지않게?..#
                        i = 10
                    elif i < 5:
                        i = 5
                    p.ChangeDutyCycle(i)

                    print("case 2")

            elif x2_coord_average > 102 or x2_coord_average < 98:
                if AA > x2_coord_average + 1 or AA < x2_coord_average - 1:
                    i = 7.5 + (((x2_coord_average - 25) - AA) / 35)
                    i = round(i, 1)
                    p.ChangeDutyCycle(i)
                    print("case 1")
            else:
                i = 7.5
                p.ChangeDutyCycle(i)
                print("case0")

            # left_line_image = display_lines(img2, np.array(l1))
            # right_line_image = display_lines(img2, np.array(l2))
            # 차선인식 보고싶으면 주석 푸르고 봐라
            # line_image = cv2.addWeighted(left_line_image, 1, right_line_image, 1, 1)
            # combo_image = cv2.addWeighted(img2, 1, line_image, 1, 1)

            print('Frame took{}'.format(time.time() - last_time))
            last_time = time.time()

            # cv2.imshow('result', combo_image)

            if stat1 == BACKWARD or stat1 == STOP:
                stat1 = setMotor(CH1, 40, FORWARD)
                stat2 = setMotor(CH2, 40, FORWARD)

        except:
            stat1 = setMotor(CH1, 25, STOP)
            stat2 = setMotor(CH2, 25, STOP)
            pass

        if cv2.waitKey(1) & 0xFF == 27:
            break

except KeyboardInterrupt:
    pass

pwmA.stop()
pwmB.stop()
p.stop()
cv2.destroyAllWindows()
cap.release()
GPIO.cleanup()
