from Tetra_pybullet import TetraSim
import time
import keyboard
from Rolling_gait import Rolling
import math
import statistics

"""
str - 0-0.39, dir - 0-3.14
front-blue, right-yellow, left-green, top-red
"""

gui = True
tetra = TetraSim(gui, 190)
tetra.run()
gait = Rolling()
gait.half_circle_coordinates(0.1)
gait.red_top("blue")

# action = gait.action

# action_neutral = [[0, 0, 0, 0, 0, 0, 0, 0]]
action = [[0, 0, 0, 0, 0, 0, 0, 0]]
a = action[0]
count = 0
ind = 0
flag_stop = 0
ite = 0
f_green = False
f_blue = False
f_red = False
f_yellow = False
key_press = False

x_rot = []
y_rot = []
z_rot = []

while 1:

    if (count > 120) and (count % 24 == 0) and flag_stop == 0:
        a = action[ind]
        ind += 1
        if ind == len(action):
            ind = 0
            flag_stop = 1
    # a = action_neutral

    if count % 120 == 0:
        ite += 1

    if keyboard.is_pressed('s'):
        orientation = tetra.getRobotBaseOrientation().tolist()
        x_rot.append(orientation[0])
        y_rot.append(orientation[1])
        z_rot.append(orientation[2])

    if keyboard.is_pressed('c'):
        print("X mean: ", statistics.mean(x_rot), " | std: ", statistics.stdev(x_rot))
        print("Y mean: ", statistics.mean(y_rot), " | std: ", statistics.stdev(y_rot))
        print("Z mean: ", statistics.mean(z_rot), " | std: ", statistics.stdev(z_rot))
        x_rot = []
        y_rot = []
        z_rot = []

    if not key_press:
        if keyboard.is_pressed('b'):
            f_blue = True
            key_press = True
        elif keyboard.is_pressed('y'):
            f_yellow = True
            key_press = True

        elif keyboard.is_pressed('f'):
            f_green = True
            key_press = True

        elif keyboard.is_pressed('r'):
            f_red = True
            key_press = True

        elif keyboard.is_pressed('n'):
            flag_stop = 0
            action = [[0, 0, 0, 0, 0, 0, 0, 0]]

    if f_blue:
        if keyboard.is_pressed('f'):
            gait.blue_top("green")
            action = gait.action
            flag_stop = 0
            key_press = False
            f_blue = False
        elif keyboard.is_pressed('y'):
            gait.blue_top("yellow")
            action = gait.action
            flag_stop = 0
            key_press = False
            f_blue = False
        elif keyboard.is_pressed('r'):
            gait.blue_top("red")
            action = gait.action
            flag_stop = 0
            key_press = False
            f_blue = False
        elif keyboard.is_pressed('o'):
            flag_stop = 0
            action = [[0.2, math.pi/3, 0, 0, 0, 0, 0, 0]]
            key_press = False
    elif f_yellow:
        if keyboard.is_pressed('f'):
            gait.yellow_top("green")
            action = gait.action
            flag_stop = 0
            key_press = False
            f_yellow = False
        elif keyboard.is_pressed('b'):
            gait.yellow_top("blue")
            action = gait.action
            flag_stop = 0
            key_press = False
            f_yellow = False
        elif keyboard.is_pressed('r'):
            gait.yellow_top("red")
            action = gait.action
            flag_stop = 0
            key_press = False
            f_yellow = False
    elif f_green:
        if keyboard.is_pressed('y'):
            gait.green_top("yellow")
            action = gait.action
            flag_stop = 0
            key_press = False
            f_green = False
        elif keyboard.is_pressed('b'):
            gait.green_top("blue")
            action = gait.action
            flag_stop = 0
            key_press = False
            f_green = False
        elif keyboard.is_pressed('r'):
            gait.green_top("red")
            action = gait.action
            flag_stop = 0
            key_press = False
            f_green = False
    elif f_red:
        if keyboard.is_pressed('y'):
            gait.red_top("yellow")
            action = gait.action
            flag_stop = 0
            key_press = False
            f_red = False
        elif keyboard.is_pressed('b'):
            gait.red_top("blue")
            action = gait.action
            flag_stop = 0
            key_press = False
            f_red = False
        elif keyboard.is_pressed('f'):
            gait.red_top("green")
            action = gait.action
            flag_stop = 0
            key_press = False
            f_red = False

    if count == 240 * 600 or keyboard.is_pressed('q'):
        break

    ori = tetra.step(a)

    if gui:
        time.sleep(1. / 240)
        count += 1

tetra.endSimulation()
