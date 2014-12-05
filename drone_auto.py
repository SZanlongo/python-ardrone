#!/usr/bin/env python

# Copyright (c) 2011 Bastian Venthur
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.


"""Demo app for the AR.Drone.

This simple application allows to control the drone and see the drone's video
stream.
"""


## TODO: Filter out bad vanishing points (i.e., threshold for distance to center of image).

import cv2
import numpy as np
import math
import pygame
import libardrone


def line(p1, p2):
    A = (p1[1] - p2[1])
    B = (p2[0] - p1[0])
    C = (p1[0] * p2[1] - p2[0] * p1[1])
    return A, B, -C


def intersection(L1, L2):
    D  = L1[0] * L2[1] - L1[1] * L2[0]
    Dx = L1[2] * L2[1] - L1[1] * L2[2]
    Dy = L1[0] * L2[2] - L1[2] * L2[0]
    if D != 0:
        x = Dx / D
        y = Dy / D
        return x, y
    else:
        return False


def ip_duplicate_check(p, l):
    for x, y in l:
        if p[0] == x and p[1] == y:
            return True
    return False

    
def GetImage(surf):
    surf = pygame.transform.flip(surf, True, False)
    surf = pygame.transform.rotate(surf, 90)
    surf = pygame.surfarray.pixels3d(surf)
    res, imgb = cv2.imencode('.jpg', surf)
    img = cv2.imdecode(imgb, 1)

    return img


def HoughTransform(img):
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    edges = cv2.Canny(gray, 50, 150, apertureSize = 3)
    hls = cv2.HoughLines(edges, 1, np.pi / 180, 100)
    lines = []

    for rho,theta in hls[0]:
        a = np.cos(theta)
        b = np.sin(theta)
        x0 = a * rho
        y0 = b * rho
        x1 = int(x0 + 1000 * (-b))
        y1 = int(y0 + 1000 * (a))
        x2 = int(x0 - 1000 * (-b))
        y2 = int(y0 - 1000 * (a))
        lines.append(((x1, y1), (x2, y2)))
        
    return lines


def FindIntersections(lines):
    ip_list = []

    for i in range(len(lines)):
        for j in range(len(lines)):
            L1 = line(lines[i][0], lines[i][1])
            L2 = line(lines[j][0], lines[j][1])
            intersect_point = intersection(L1, L2)
            if intersect_point != False:
                if not ip_duplicate_check(intersect_point, ip_list):
                    ip_list.append(intersect_point)

    return ip_list


def FindVanishingPoint(img, size, ips):
    gridsize = size
    img_hgt = img.shape[0]
    img_wid = img.shape[1]
    img_col = (img_wid // gridsize) + 1
    img_row = (img_hgt // gridsize) + 1
    max_ips = 0
    grid = []

    for j in range(img_row):
        for i in range(img_col):
            grid_l = i * gridsize
            grid_r = (i + 1) * gridsize
            grid_t = j * gridsize
            grid_b = (j + 1) * gridsize
            ip_count = 0
            
            for x, y in ips:
                if x >= grid_l and x <= grid_r and y >= grid_t and y <= grid_b:
                    ip_count += 1

            if ip_count > max_ips:
                max_ips = ip_count
                grid_cen_x = (grid_l + grid_r) / 2
                grid_cen_y = (grid_t + grid_b) / 2
                grid = [grid_cen_x, grid_cen_y]
                
    return grid


def MoveDroneTo(grid, spd):
    ccenterx = 320 / 2
    ccentery = 240 / 2
    gcenterx = grid[0]
    gcentery = grid[1]

    # get x and y distance
    distx = gcenterx - ccenterx 
    disty = gcentery - ccentery

    # get x and y time
    timex = abs(distx / spd) * 0.1
    timey = abs(disty / spd) * 0.1

    dists = [distx, disty]
    times = [timex, timey]

    return dists, times


def StopwatchDisplay(timer):
    milsecs = timer % 100
    seconds = (timer // 100) % 60
    minutes = (timer // 6000) % 60
    strTime = "%d:%02d\'%02d\"" % (minutes, seconds, milsecs)
    return strTime


def sign(v):
    if (v == 0):
        return 0
    return abs(v) // v

    
def main():
    pygame.init()
    
    W, H = 320, 240
        
    screen = pygame.display.set_mode((W, H))
    drone = libardrone.ARDrone()

    # drone's velocity
    drone.speed = 0.1

    clock = pygame.time.Clock()
    timer = 0

    # Constant Variables
    TAKEOFF = 1
    HOVER = 2
    MOVE_FORWARD = 3
    MOVE_HORIZONTAL = 4
    MOVE_VERTICAL = 5
    TURN_LEFT = 6
    TURN_RIGHT = 7
    SHUTDOWN = 99

    elapsedTime = 0
    totalTime = 0
    state = 0
    result = None
    isActivated = False
    initialize = False
    canFly = False

    running = True
    while running:
        canFly = drone.navdata.get('drone_state',dict()).get('emergency_mask', 1)
        isFlying = drone.navdata.get('drone_state', dict()).get('fly_mask', 1)
        totalTime = timer - elapsedTime
        altitude = drone.navdata.get(0, dict()).get('altitude', 0)
        
        # The drone can be shutted down anytime by pressing 'Esc'.
        for event in pygame.event.get():
            if (event.type == pygame.KEYDOWN):
                if (event.key == pygame.K_RETURN):
                    if (not isActivated):
                        elapsedTime = timer
                        state = TAKEOFF
                        initialize = True
                        isActivated = True
                elif (event.key == pygame.K_BACKSPACE):
                    print(canFly)
                    if (not isActivated):
                        drone.reset()
                    elif (isActivated and state != SHUTDOWN):
                        elapsedTime = timer
                        state = SHUTDOWN
                        initialize = True
                    #else:
                        #running = False
                elif (event.key == pygame.K_ESCAPE):
                    if (isActivated and state != SHUTDOWN):
                        elapsedTime = timer
                        state = SHUTDOWN
                        initialize = True
                    else:
                        running = False


        # Takeoff state:
        if (state == TAKEOFF):
            if (initialize):
                initialize = False
            if (totalTime < 600 and totalTime % 100 == 0):
                print "Take off!"
                drone.takeoff()
            if (totalTime == 600):
                elapsedTime = timer
                state = HOVER
                initialize = True


        # Shutdown state:
        if (state == SHUTDOWN):
            if (initialize):
                print "Landing drone..."
                drone.land()
                initialize = False
                
            if (totalTime >= 250):
                drone.reset()
                running = False


        # Hovering state:
        if (state == HOVER):
            if (initialize):
                result = None
                initialize = False

            drone.hover()
            if (totalTime > 0 and totalTime % 150 == 0):
                try:
                    sfc = pygame.image.frombuffer(drone.image, (W, H), 'RGB')
                    img = GetImage(sfc)
                    result = FindVanishingPoint(img, 20, FindIntersections(HoughTransform(img)))
                    print "VP at (%d, %d). Adjust position." % (result[0], result[1])
                    elapsedTime = timer
                    state = MOVE_HORIZONTAL
                    initialize = True
                except:
                    print "No VP found."
                    elapsedTime = timer
                    state = MOVE_FORWARD
                    initialize = True


        # Moving forward state:
        if (state == MOVE_FORWARD):
            if (initialize):
                print "Move forward."
                initialize = False

            if (totalTime < 50):
                #drone.hover()
                drone.move_forward()
            else:
                elapsedTime = timer
                state = HOVER
                initialize = True
                    

        # Moving horizontal state:
        if (state == MOVE_HORIZONTAL):
            if (initialize):
                dists, times = MoveDroneTo(result, drone.speed)
                timeLength = times[0]
                direction = sign(dists[0])
                proximity = abs(dists[0])
                print 'VP is %d px. in X-dir. %d.' % (proximity, direction)
                print 'Move in X-dir. %d for %.2f sec.' % (direction, timeLength * 0.01)
                initialize = False

            # Adjustment is decided based on the promixity of the vanishing point
            # to the center of the drone's view.
            if (proximity > 20):
                offset = proximity - 20
                print 'VP off by %d px. Align vertically.' % offset
                elapsedTime = timer
                state = MOVE_VERTICAL
                initialize = True
            else:
                if (totalTime < timeLength):
                    if (direction < 0):
                        drone.rotate_left()
                    elif (direction > 0):
                        drone.rotate_right()
                else:
                    elapsedTime = timer
                    state = MOVE_VERTICAL
                    initialize = True


        # Moving vertical state:
        if (state == MOVE_VERTICAL):
            if (initialize):
                dists, times = MoveDroneTo(result, drone.speed)
                timeLength = times[1]
                direction = sign(dists[1])
                proximity = abs(dists[1])
                print 'VP is %d px. in Y-dir. %d.' % (proximity, direction)
                print 'Move in Y-dir %d for %.2f sec.' % (direction, timeLength * 0.01)
                initialize = False

            # Adjustment is decided based on the promixity of the vanishing point
            # to the center of the drone's view.
            if (proximity > 10):
                offset = proximity - 10
                print 'VP off by %d px. End adjustment.' % offset
                elapsedTime = timer
                state = MOVE_FORWARD
                initialize = True
            else:
                if (totalTime < timeLength):
                    if (direction < 0):
                        drone.move_up()
                    elif (direction > 0):
                        drone.move_down()
                else:
                    elapsedTime = timer
                    state = MOVE_FORWARD
                    initialize = True
                
        
        try:
            surface = pygame.image.frombuffer(drone.image, (W, H), 'RGB')
            # battery status
            hud_color = (255, 0, 0) if drone.navdata.get('drone_state', dict()).get('emergency_mask', 1) else (10, 10, 255)
            bat = drone.navdata.get(0, dict()).get('battery', 0)
            f = pygame.font.Font(None, 20)
            hud = f.render('Battery: %i%%' % bat, True, hud_color)
            screen.blit(surface, (0, 0))
            screen.blit(hud, (10, 10))
        except:
            pass

        pygame.display.flip()
        clock.tick(50)
        pygame.display.set_caption("FPS: %.2f, State=%d, Timer: %s" % (clock.get_fps(), state, StopwatchDisplay(totalTime)))
        timer += 2
        

    print "Shutting down...",
    drone.halt()
    pygame.display.quit()
    print "Ok."


if __name__ == '__main__':
    main()

