from fpioa_manager import fm
from machine import UART
from board import board_info
import time
import lcd
import struct
import math
import sensor
import image
import json
from Maix import GPIO
from fpioa_manager import fm
print(board_info)
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
path = [[0, 0], [1, 0], [1, 1]]
path_point = 0
reg = True
go = True
log = ""

last_buf = "s\n"
curent_buf = "s\n"


class map():
    def __init__(self, map):
        self.map = map

    def get_cell(self, x, y):
        return self.map[y][x]


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


def float_to_bytes(floats):
    buf = bytearray(4 * len(floats))
    for i, f in enumerate(floats):
        struct.pack_into("<f", buf, 4*i, f)
    return buf


def send(data: bytes, command: str):
    buf = bytearray(2 + len(data))
    struct.pack_into("<sb", buf, 0, bytes(
        command, "utf-8"), len(data))
    if len(data) != 0:
        struct.pack_into("<"+str(len(data))+"s", buf, 2, data)
    return buf


class car ():
    def __init__(self, path):
        self.orientation = 0
        self.last_orientation = 0
        self.x = 0
        self.y = 0
        self.last_x = 0
        self.last_y = 0
        self.path = path
        self.path_index = 0
        self.road_type = {"strait": 0, "left": 0, "right": 0,
                          "leftT": 0, "rightT": 0, "T": 0, "4-juc": 0}

    def new_cell(self):
        if self.y > 244:
            return False
        print(self.x, self.last_x, self.y, self.last_y)
        if abs(self.x - self.last_x) + abs(self.y - self.last_y) > 2:

            self.path_index += 1
            self.last_x = self.path[self.path_index][0]
            self.last_y = self.path[self.path_index][1]
            print(self.last_x, self.last_y)
            #self.orientation = radius_mod(self.calc_orientaion())
            print(max(self.road_type))
            self.road_type = {"strait": 0, "left": 0, "right": 0,
                              "leftT": 0, "rightT": 0, "T": 0, "4-juc": 0}
            return True
        return False

    def check_rotation(self):
        logs("lll")
        print("ll")
        if abs((self.orientation - self.last_orientation)) > math.pi/2:
            self.last_orientation = self.orientation
            logs("rotad")
            print("roterad")
            return True
        return False

    def calc_path_orientation(self, index):
        print(self.path)
        print(index)
        x = self.path[index+1][0] - self.path[index][0]
        y = self.path[index+1][1] - self.path[index][1]
        return math.atan2(y, x)

    def turn(self):
        angel = self.calc_path_orientation(self.path_index)
        if angel == 0:
            return None
        if angel == math.pi/2:
            return 'r'
        if angel == -math.pi/2:
            return 'l'


def radius_mod(radius):
    a = radius % (2*math.pi)
    return a - 2 * math.pi * (a > math.pi)


print(board_info)

robot = car(path)


def logs(mes):
    uart.write(send(bytes(mes, "utf-8"), "L"))


while True:
    # the hader contains the command character and lenght
    # the while loop is for reading the whole buffer so no data get lost
    while True:
        header = uart.read(2)
        if header == None:
            # buffer is empty
            break
        mes = header[0]
        try:
            length = header[1]
        except:
            break
        data = uart.read(length)
        #print(data, length, len(data), mes)
        if mes == 82:
            print(data)
            try:
                l = struct.unpack_from("<i", data, 0)[0]
                r = struct.unpack_from("<i", data, 4)[0]
            except:
                print("to small")
            print(robot.x, robot.y)
        elif mes == 80:
            # print(data)
            try:
                robot.x = struct.unpack_from("<d", data, 0)[0]
                robot.y = struct.unpack_from("<d", data, 8)[0]
                robot.orientation = radius_mod(
                    struct.unpack_from("<d", data, 16)[0])
                #print(robot.x, robot.y, robot.orientation)
            except:
                pass
                #print("to small")

        elif mes == 112:
            # print(data)
            robot.path = json.loads(data)
            go = True
            print("go")
            print(robot.path)

    # time.sleep_ms(50)
    # if read_data:
    #print("recv:\n", read_data.decode())

    img = sensor.snapshot()
    img.rotation_corr(0, 0, 180)
    segment = []
    if go:
        if reg and False:  # robot.new_cell():
            print(robot.x, robot.y)
            print(len(robot.path), robot.path_index)
            if len(robot.path) < robot.path_index + 2:
                reg = False
                go = False
                uart.write(send(bytes(), "s"))
                print("done")
                logs("done")
            else:
                print("new cell")
                turn = robot.turn()
                log = "new_cell"
                logs("new cell")
                if turn != None:
                    log += turn
                    print("turn")
                    logs(turn)
                    curent_buf = send(bytes(), turn)
                    reg = False
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
                curent_buf = send(bytes(float_to_bytes([err])), 'e')
        # uart.write(last_buf)
                if err > margin:

                    # uart.write('r'+str(motorForce)+'\n')
                    #print("driving right")
                    led_g.value(1)
                    led_r.value(0)
                    led_b.value(1)
                elif err < -margin:
                    # uart.write('l'+str(abs(motorForce))+'\n')
                    #print("driving left")
                    led_g.value(1)
                    led_r.value(1)
                    led_b.value(0)
                else:
                    # uart.write('f\n')
                    #print("driving forward")
                    led_g.value(0)
                    led_r.value(1)
                    led_b.value(1)
            else:
                curent_buf = send(bytes(), "s")
                led_r.value(1)
                led_g.value(1)
                led_b.value(1)
        if robot.check_rotation():
            curent_buf = send(bytes(), "s")
            print("rotaad")
            reg = True
    if curent_buf != last_buf:
        last_buf = curent_buf
        uart.write(last_buf)
    if reg:
        for seg in segment:
            if len(seg) != 0:
                img.draw_rectangle(seg[0][0:4], color=(0, 0, 255), thickness=3)
    piece = which_piece(segment)
    if piece != '':
        robot.road_type[piece] += 1
    dir = True
    img.draw_string(1, 1, piece, color=(0, 0, 0), scale=2)
    img.draw_string(1, 20, log, color=(0, 0, 0), scale=2)
    # if try_to_turn(existing_segment(segment), piece, dir):
    #     uart.write("r\n" if dir else "l\n")

    lcd.display(img)
uart.deinit()
del uart
