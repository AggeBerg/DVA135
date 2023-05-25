from fpioa_manager import fm
from machine import UART
from board import board_info
import time
import lcd
import struct
import math
import sensor
import image
from Maix import GPIO
from fpioa_manager import fm
fm.register(board_info.LED_R, fm.fpioa.GPIO0, force=True)
led_r = GPIO(GPIO.GPIO0, GPIO.OUT)
led_r.value(1)
fm.register(board_info.LED_G, fm.fpioa.GPIO1, force=True)
led_g = GPIO(GPIO.GPIO1, GPIO.OUT)
led_g.value(1)
fm.register(board_info.LED_B, fm.fpioa.GPIO2, force=True)
led_b = GPIO(GPIO.GPIO2, GPIO.OUT)
led_b.value(1)
# need your connect hardware IO 10/11 to loopback
fm.register(board_info.WIFI_TX, fm.fpioa.UART1_TX, force=True)
fm.register(board_info.WIFI_RX, fm.fpioa.UART1_RX, force=True)

uart = UART(UART.UART1, 115200, 8, None, 1, timeout=1000, read_buf_len=4096)


lcd.init(freq=15000000)
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.set_vflip(1)
sensor.run(1)

green_threshold = (15, 73, -63, -12, 15, 45)
margin = 20
center_x = 320/2
x_boxes = 1
y_boxes = 7
lcd_string = ""
path = [[0, 0], [0, 1], [1, 1], [1, 2], [2, 2], [3, 2], [4, 2], [5, 2]]
path_point = 0
reg = True
log = ""

last_buf = "s\n"
curent_buf = "s\n"


class map():
    def __init__(self, map):
        self.map = map

    def get_cell(self, x, y):
        return self.map[y][x]


class car ():
    def __init__(self, radius, x, y, orientation, cell_size, path):
        self.radius = radius
        self.x = x
        self.start_x = x
        self.y = y
        self.start_y = y
        self.orientation = orientation
        self.start_orientation = orientation
        self.r = 0
        self.last_r = 0
        self.l = 0
        self.last_l = 0
        self.cell_size = cell_size
        self.path = path
        self.path_index = 0

    def calc_orientaion(self):
        return (self.l - self.r)/self.radius

    def new_cell(self):
        if self.r - self.last_r + self.l - self.last_l > 2 * self.cell_size:
            self.last_r = self.r
            self.last_l = self.r
            self.path_index += 1
            self.orientation = radius_mod(self.calc_orientaion())

            return True
        return False

    def check_rotation(self):
        if abs(radius_mod(self.calc_orientaion()) - self.orientation) < 0.2:
            return True
        return False

    def calc_path_orientation(self, index):
        x = self.path[index+1][0] - self.path[index][0]
        y = self.path[index+1][1] - self.path[index][1]
        return math.atan2(y, x)

    def turn(self):
        angel = self.calc_path_orientation(
            self.path_index - 1) - self.calc_path_orientation(self.path_index)
        if angel == 0:
            return None
        if angel == math.pi/2:
            return 'r'
        if angel == -math.pi/2:
            return 'l'


def radius_mod(radius):
    a = radius % (2*math.pi)
    return a - 2 * math.pi * (a > math.pi)


def radian_overflow(radian):
    if radian > math.pi:
        return radian - 2 * math.py
    if radian < -math.pi:
        return radian + 2 * math.py


def existing_segment(segments):
    return [segment[0] for segment in segments if segment]


def which_piece(segments):
    ex = existing_segment(segments)
    if not ex:
        return ""
    # if isLeftCurve(ex):
    #     print("curve")

    first_segment = segment[0] == None

    previous_segment_w = ex[0].w()
    previous_segment_x = 2 * \
        ex[0].x() - ex[1].x() if len(ex) > 1 else ex[0].x()
    previous_segment_w_dif = 0
    previous_segment_x_dif = 0
    average = 0
    strait = True
    w_2dif_list = []
    x_2dif_list = []
    wx_2dif_list = []
    w_dif_list = []
    x_dif_list = []
    wx_dif_list = []
    Lcount = 0
    Rcount = 0
    for (i, s) in enumerate(ex):
        w_dif = s.w() - previous_segment_w
        x_dif = s.x() - previous_segment_x
        w_2dif = w_dif - previous_segment_w_dif
        x_2dif = x_dif - previous_segment_x_dif
        w_dif_list.append(w_dif)
        x_dif_list.append(x_dif)
        wx_dif_list.append(x_dif + w_dif)
        w_2dif_list.append(w_2dif)
        x_2dif_list.append(x_2dif)
        wx_2dif_list.append(x_2dif + w_2dif)
        average += abs(w_dif)

        if previous_segment_w != 0:
            if first_segment:
                if x_dif < -100 or x_dif + w_dif > 100:
                    #print(x_dif, x_dif + w_dif)
                    if x_dif > -10:
                        print("rightT")
                        return "rightT"
                    elif x_dif + w_dif < 10:
                        print("leftT")
                        return "leftT"
                    else:
                        if previous_segment_w < 70:
                            print("4-junction")
                            return "4-juc"
            else:
                if (previous_segment_x == 1 or s.x() == 1) and (previous_segment_w + previous_segment_x == 319 or s.w() + s.x() == 319):
                    print("T")
                    return "T"
                if abs(x_2dif) > 2 and abs(x_2dif) < 60 or previous_segment_x == 1 or previous_segment_w + previous_segment_x == 319:
                    Lcount += 1
                    if Lcount == 3:
                        if ex[0].x() == 1:
                            print("left")
                            return "left"
                        else:
                            print("right")
                            return "right"
                else:
                    Lcount = 0
                # if x_2dif > 5 and x_2dif < 80 or previous_segment_w+previous_segment_x == 319:
                #     Rcount += 1
                #     if Rcount == 3:
                #         print("turn")
                #         # return "right"
                # else:
                #     Rcount = 0
            if abs(w_2dif) > 8:
                strait = False

        previous_segment_w = s.w()
        previous_segment_x = s.x()
        previous_segment_w_dif = w_dif
        previous_segment_x_dif = x_dif

    # print(w_dif_list)
    # print(x_dif_list)
    # print(wx_dif_list)
    # print(w_2dif_list)
    # print(x_2dif_list)
    # print(wx_2dif_list)
    # print(average/9)

    if strait:
        print("strait")
        return "strait"
    return ""


def try_to_turn(segments: list, piece, dir: bool):
    """dir: False = left, True = right"""
    if (dir and piece == "leftT") or (not dir and piece == "rightT") or piece in ["", "strait", "left", "right"]:
        return False
    segments.sort(key=lambda values: values.w(), reverse=True)
    print(segments[0])
    if segments[0].y() > 140:
        return True


def float_to_bytes(command, floats):
    buf = bytearray(2 + 4 * len(floats))
    struct.pack_into("<s", buf, 0, bytes(command, "utf-8"))
    for i, f in enumerate(floats):
        struct.pack_into("<f", buf, 1+4*i, f)
    struct.pack_into("<s", buf, 1 + 4*len(floats), bytes('\n', "utf-8"))
    print(struct.unpack_from('<B', buf, 5))
    return buf


print(board_info)

# left_pin = GPIO(22, GPIO.IN, GPIO.PULL_NONE)
# left_revolutions = 0


# def left_pin_callback():
#     left_revolutions += 1
#     # print("L")


# left_pin.irq(left_pin_callback, GPIO.IRQ_RISING)
# right_pin = GPIO(23, GPIO.IN, GPIO.PULL_NONE)
# right_revolutions = 0


# def right_pin_callback():
#     right_revolutions += 1
#     # print("R")


# right_pin.irq(right_pin_callback, GPIO.IRQ_RISING)
robot = car(40, 0, 0, 0, 256, path)

while True:
    inData = uart.readline()
    if inData != None:
        print(inData, inData[0])
        if inData[0] == 82:
            try:
                l = struct.unpack_from("<i", inData, 1)
                r = struct.unpack_from("<i", inData, 5)

                robot.l = 0.4036073 * l[0]
                robot.r = 0.4036073 * r[0]
                print(robot.calc_orientaion(), robot.l, robot.r)
            except:
                print("to small")

    if robot.new_cell():
        turn = robot.turn()
        log = "new_cell" + turn
        if turn != None:
            curent_buf = turn + '\n'
            reg = False
    time.sleep_ms(50)
    # if read_data:
    #print("recv:\n", read_data.decode())

    img = sensor.snapshot()
    img.rotation_corr(0, 0, 180)
    segment = []
    if reg:
        img = img.binary([(0, 100, -80, 5, -128, 48)])
        x_step = 320 // x_boxes
        y_step = 240//y_boxes
        for x in range(0, x_boxes):
            for y in range(0, y_boxes):
                segment.append(img.find_blobs([(0, 0, 0, 0, 0, 0)],
                                              pixel_threshold=600, area_threshold=500, roi=(x_step*x, y_step*y, x_step, y_step)))
        # print(segment)
        blobs = img.find_blobs([(0, 0, 0, 0, 0, 0)],
                               pixel_threshold=2000, area_threshold=2000)
        regulating_segment = img.find_blobs([(0, 0, 0, 0, 0, 0)],
                                            pixel_threshold=400, area_threshold=500, roi=(320//3, 2*240//3, 320//3, 240//3))
        big_blob = None
        if blobs or len(regulating_segment) != 0:
            if regulating_segment:
                err = center_x - regulating_segment[0][5]
            else:
                for b in blobs:
                    if big_blob:
                        if b.pixels() > big_blob.pixels():
                            big_blob = b
                    else:
                        big_blob = b
                color = (255, 0, 0)
                if big_blob[2] < 300:
                    color = (255, 0, 0)
                    err = (center_x - big_blob[5])
                else:
                    color = (0, 255, 0)
                    err = 0
                img.draw_rectangle(big_blob[0:4], color=color, thickness=6)
            curent_buf = float_to_bytes('e', [err])
        # uart.write(last_buf)
            if err > margin:

                # uart.write('r'+str(motorForce)+'\n')
                print("driving right")
                led_g.value(1)
                led_r.value(0)
                led_b.value(1)
            elif err < -margin:
                # uart.write('l'+str(abs(motorForce))+'\n')
                print("driving left")
                led_g.value(1)
                led_r.value(1)
                led_b.value(0)
            else:
                # uart.write('f\n')
                print("driving forward")
                led_g.value(0)
                led_r.value(1)
                led_b.value(1)
        else:
            curent_buf = 's\n'
            led_r.value(1)
            led_g.value(1)
            led_b.value(1)
    else:
        if car.check_rotation:
            curent_buf = "s\n"
            reg = True
    if curent_buf != last_buf:
        last_buf = curent_buf
        uart.write(last_buf)
    if reg:
        for seg in segment:
            if len(seg) != 0:
                img.draw_rectangle(seg[0][0:4], color=(0, 0, 255), thickness=3)
    piece = which_piece(segment)
    dir = True
    img.draw_string(1, 1, piece, color=(0, 0, 0), scale=2)
    img.draw_string(1, 20, log, color=(0, 0, 0), scale=2)
    # if try_to_turn(existing_segment(segment), piece, dir):
    #     uart.write("r\n" if dir else "l\n")

    lcd.display(img)
uart.deinit()
del uart
