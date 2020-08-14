#!/usr/bin/env python3

import rospy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from bitbots_msgs.srv import Leds, LedsRequest, LedsResponse
from std_msgs.msg import ColorRGBA

BLINK_DURATION = 0.2
ERROR_TIMEOUT = 1

rospy.init_node("error_blink")

last_hardware_error_time = None
currently_blinking = False
leds_red = False
led_set_time = None

# set up service and prepare requests
led_serv = rospy.ServiceProxy("/set_leds", Leds)
red_request = LedsRequest()
red_leds_array = []
for i in range(3):
    red_led = ColorRGBA()
    red_led.r = 1
    red_led.a = 0
    red_leds_array.append(red_led)
red_request.leds = red_leds_array

previous_req = LedsRequest()


def cb(msg: DiagnosticStatus):
    global last_hardware_error_time
    # we check if any status in the received array is not ok
    if msg.level != DiagnosticStatus.OK:
        last_hardware_error_time = rospy.Time.now().to_sec()


def set_red():
    global leds_red, red_request
    leds_red = True
    previous_req.leds = led_serv(red_request).previous_leds


def reset_leds():
    global leds_red, previous_req
    leds_red = False
    led_serv(previous_req)


# wait a moment on startup, otherwise we will think there is a problem while ros control is still booting
rospy.wait_for_service('/set_leds')
rospy.sleep(1)

rospy.Subscriber("/diagnostics_toplevel_state", DiagnosticStatus, cb, queue_size=1, tcp_nodelay=True)

rate = rospy.Rate(100)
while not rospy.is_shutdown():
    if last_hardware_error_time is not None:
        current_time = rospy.Time.now().to_sec()
        if currently_blinking:
            # we are currently blinking, check if the last hardware error is to long ago
            if current_time - last_hardware_error_time > ERROR_TIMEOUT:
                currently_blinking = False
                reset_leds()
                led_set_time = current_time
            else:
                if leds_red and current_time - led_set_time > BLINK_DURATION:
                    # reset LEDs
                    reset_leds()
                    led_set_time = current_time
                elif not leds_red and current_time - led_set_time > BLINK_DURATION:
                    # set LEDs red
                    set_red()
                    led_set_time = current_time
        else:
            # we are not blinking, check if we want to start
            if current_time - last_hardware_error_time < ERROR_TIMEOUT:
                # start blinking
                currently_blinking = True
                set_red()
                led_set_time = current_time
    rate.sleep()
