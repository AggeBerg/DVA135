from fpioa_manager import fm
from machine import UART
from board import board_info
import time
import lcd
import struct
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

uart = UART(UART.UART1, 115200, 8, 1, 0, timeout=1000, read_buf_len=4096)

lcd.init(freq=15000000)
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.run(1)

green_threshold = (15, 73, -63, -12, 15, 45)
margin = 20
center_x = 320/2

integral = 0
derivative = 0
motorforce = 0
p_const = .02
i_const = .001
d_const = .001
last_buf = "s\n"
curent_buf = "s\n"


def float_to_bytes(command, floats):
    buf = bytearray(2 + 4 * len(floats))
    struct.pack_into("<s", buf, 0, bytes(command, "utf-8"))
    for i, f in enumerate(floats):
        struct.pack_into("<f", buf, 1+4*i, f)
    struct.pack_into("<s", buf, 1 + 4*len(floats), bytes('\n', "utf-8"))
    print(struct.unpack_from('<B', buf, 5))
    return buf


print(board_info)
while True:
    time.sleep_ms(200)
    read_data = uart.read()
    # if read_data:
    #print("recv:\n", read_data.decode())

    img = sensor.snapshot()
    # img.rotation_corr(0,0,90)
    img = img.binary([(0, 100, -80, 5, -128, 48)])
    blobs = img.find_blobs([(0, 0, 0, 0, 0, 0)],
                           pixel_threshold=10000, area_threshold=10000)

    big_blob = None
    if blobs:
        for b in blobs:
            if big_blob:
                if b.pixels() > big_blob.pixels():
                    big_blob = b
            else:
                big_blob = b

    if big_blob:
        color = (255, 0, 0)
        if big_blob[2] < 300:
            color = (255, 0, 0)
            err = (big_blob[5] - center_x)
            integral += err
            derivetive = err - derivative
            motorForce = err * p_const + integral * i_const + derivative * d_const
        else:
            color = (0, 255, 0)
            err = 0
            motorForce = 0

        img.draw_rectangle(big_blob[0:4], color=color, thickness=10)
        if False:
            right = 1 if motorForce >= 0 else motorForce + 1
            left = 1 if motorForce <= 0 else 1 - motorForce
            right = -1 if right < -1 else right
            left = -1 if left < -1 else left
            print(motorForce)
            print(right)
            print(left)
            print(float_to_bytes('a', [700.0, right, left]))
            curent_buf = float_to_bytes('a', [700.0, right, left])
        else:
            curent_buf = float_to_bytes('e', [err])
           # uart.write(last_buf)
        if motorForce > margin:

            # uart.write('r'+str(motorForce)+'\n')
            print("driving right")
            led_g.value(1)
            led_r.value(0)
            led_b.value(1)
        elif motorForce < -margin:
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
        print("stoping")
        led_r.value(1)
        led_g.value(1)
        led_b.value(1)
    if curent_buf != last_buf:
        last_buf = curent_buf
        uart.write(last_buf)
    lcd.display(img)
uart.deinit()
del uart
