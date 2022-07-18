#!/usr/bin/env python3
from eye import *
import time
import random
import math

angular_base_speed = 45  # initial angular BASE speed in degrees per second, i.e. deg/s. For use in Proportional!
angular_speed = 0  # initial angular speed in degrees per second, i.e. deg/s.
angular_error = 0  # initial angular error. For use in Proportional!
linear_speed = 400  # initial linear speed in millimeter per second, i.e. mm/s.
max_range = 5000  # the maximum range (distance) that the robot can sense/see, e.g. 15000 mm is 15 meters
ASPEED = 100
THRES = 225

def main():
    LCDMenu("", "", "", "END")
    curr_time = 0
    tot_time = 0
    dn_0 = ""
    min_scan_0 = 0
    isScared = 0
    while True:
        temp_time = time.strftime('%S')
        if curr_time != temp_time:
            curr_time = temp_time
            LCDSetPrintf(0, 45, "Time elapsed: %d secs", tot_time)
            tot_time = tot_time + 1
            sec = tot_time % 30 #90
            min_scan = max_range  # put a very high scan value (distance) initially at each step/scan, 15000 mm is 15 meters
            min_scan_index = 0  # initial (at each step/scan) index is 0, i.e. rear of robot
            heading_to_turn = 0  # initial (at each step/scan) heading to turn is 0 degree, i.e. no need to turn!
            LCDSetPrintf(1, 45, "COCKROACH - Light off")
            scan = LIDARGet()
            for i in range(0, 360):  # read 360-degrees range around the robot
                LCDLine(i, 250 - int(scan[i] / 10), i, 250, BLUE)
                if scan[i] <= min_scan:
                    LCDSetPrintf(
                        2, 45, "NEW min detected: %d mm", scan[i]
                    )  # just to check the minimum reading
                    min_scan = scan[i]
                    min_scan_index = i
            # mark the minimum reading on the plot, using yellow line
            LCDLine(min_scan_index, 250 - int(min_scan / 10), min_scan_index, 250, YELLOW)
            LCDLine(180, 0, 180, 250, RED)  # straight, 0 degrees
            LCDLine(90, 0, 90, 250, GREEN)  # left, -90 degrees
            LCDLine(270, 0, 270, 250, GREEN)  # right, +90 degrees
            LCDSetPrintf(19, 0, "            -90             0             +90")
            if sec <= 10: #60
                dn = "Light off"
                if isScared == 0:
                    if min_scan < max_range:  # something is within detectable range
                        LCDSetPrintf(6, 45, "Something detected!")
                        if min_scan_index == 180:  # heading is exactly at front!
                            heading_to_turn = 0
                        elif min_scan_index < 180:
                            heading_to_turn = 180 - min_scan_index
                            heading_to_turn = heading_to_turn * 1  # turn left, +ve angle
                        elif min_scan_index > 180:
                            heading_to_turn = min_scan_index - 180
                            heading_to_turn = heading_to_turn * -1  # turn right, -ve angle
                        else:  # error? remain heading 0
                            heading_to_turn = 0
                        # just checking if min_scan actually has the minimum distance scanned
                        if min_scan == min(scan):
                            LCDSetPrintf(3, 45, "LIDAR min dist scanned: %d mm", min_scan)
                            LCDSetPrintf(4, 45, "LIDAR index of min dist: %d", min_scan_index)
                            LCDSetPrintf(5, 45, "Heading to turn: %d degrees", heading_to_turn)
                        # Proportional turn: the further the heading to turn,
                        # the faster the angular speed should be, vice-versa.
                        angular_error = abs(heading_to_turn) - angular_base_speed
                        angular_speed = angular_error + angular_base_speed
                        if heading_to_turn != 0:  # only turn if the heading to turn is NOT right in-front!
                            VWTurn(heading_to_turn, angular_speed)  # angular_speed should be Proportional, now!
                            VWWait()  # OPTIONAL: can remove for this scenario, for now
                        # RE-INIT angular variables
                        angular_error = 0
                        angular_speed = 0
                        # move straight forward, as far as the minimum distance scanned.
                        VWStraight(min_scan, linear_speed)
                    else:  # NOTHING is within detectable range
                        LCDSetPrintf(6, 45, "Something NOT detected!")
                        VWSetSpeed(0, 0)  # just stay put, i.e. Stop!
                else:
                    isScared = isScared - 1
                    if isScared == 0:
                        VWStraight(2000, linear_speed)
                        VWWait()
            else:
                dn = "Light on"
                LCDSetPrintf(1, 45, "COCKROACH - Light  on")
                if sec == 11: #61 # first second of daytime
                    VWTurn(180, ASPEED)
                    VWWait()
                    isScared = 0
                else:
                    if isScared != 5:
                        left = int(scan[90] > THRES)
                        front = int(scan[180] > THRES)
                        right = int(scan[270] > THRES)
                        print(left, front, right)
                        if front != 1 and right != 1:
                            VWTurn(135, ASPEED)  # turn anticlockwise
                            VWWait()
                            isScared = 5
                        elif front != 1 and left != 1:
                            VWTurn(-135, ASPEED)  # turn clockwise
                            VWWait()
                            isScared = 5
                        elif front == 1:
                            VWStraight(scan[180]-(THRES-1), linear_speed)
                            VWWait()
                        elif front != 1:
                            if left == 1 and right == 1: #left < right # turn right
                                if scan[90] < scan[270]:  #left < right #turn left
                                    angle = 90
                                else:
                                    angle = -90
                                VWTurn(angle, ASPEED)
                                VWWait()
                            elif scan[90] < scan[270]: #left < right # turn right
                                angle = int(math.degrees(math.atan(scan[180] / scan[90])) + 90)
                                VWTurn(-angle, ASPEED)
                                VWWait()
                            else:  # turn left
                                angle = int(math.degrees(math.atan(scan[180] / scan[270])) + 90)
                                print(scan[180], scan[270], angle)
                                VWTurn(angle, ASPEED)
                                VWWait()
        if dn != dn_0:
            dn_0 = dn
            print(dn_0)
            AUBeep()
        key = KEYRead()
        if key == KEY4:
            return 0

main()
