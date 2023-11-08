# Author - Gaurav Kumar
import pygame as py
import random
import math
import random as rnd
import numpy as np

class Landmark:
    """
        Generates a list of random landmarks within a rectangular region while ensuring a minimum distance between them.

        Args:
        - num_landmarks (int): The number of landmarks to generate.
        - rect_width (float): The width of the rectangular region.
        - rect_height (float): The height of the rectangular region.
        - min_distance (float): The minimum distance that must be maintained between landmarks.

        Returns:
        - list of tuples: A list of (x, y) coordinates representing the generated landmarks.
        """
    def generate_landmarks(self, num_landmarks, rect_width, rect_height, min_distance):
        self.landmarks = []

        for _ in range(num_landmarks):
            valid_location = False
            while not valid_location:
                x = random.uniform(0, rect_width)
                y = random.uniform(0, rect_height)

                # Check the distance to existing landmarks
                valid_location = all(
                    math.sqrt((x - lx) ** 2 + (y - ly) ** 2) >= min_distance
                    for lx, ly in self.landmarks
                )

            self.landmarks.append((x, y))

        return self.landmarks



class Robot:
    def __init__(self, x, y, theta, wheel_diameter, wheel_base, screen):
        self.x = x
        self.y = y
        self.theta = theta
        self.wheel_diameter = wheel_diameter
        self.wheel_base = wheel_base
        self.draw(screen)
        self.dis = 0
        self.ang = 0
        print("Robot")

    def odom(self, v, w, dt):
        # print("odom")
        vl = v - (w * self.wheel_diameter) / 2
        vr = v + (w * self.wheel_diameter) / 2
        
        #Itroducing error to the robot odometry data
        if self.dis > 1:
            self.dis = 0
            dx = (vl + vr) * dt * 0.5 * math.cos(self.theta) + rnd.gauss(0.05, 0.025)
            dy = (vl + vr) * dt * 0.5 * math.sin(self.theta) + rnd.gauss(0.05, 0.025)
        else:
            self.dis += dt*v
            dx = (vl + vr) * dt * 0.5 * math.cos(self.theta)
            dy = (vl + vr) * dt * 0.5 * math.sin(self.theta)

        if self.ang >= 3.14:
            self.ang = 0
            dtheta = w * dt + rnd.gauss(0.05, 0.025)
        else:
            self.ang += dt*w
            dtheta = w * dt 
        self.x += dx
        self.y += dy
        self.theta += dtheta
        Robot.draw(self, screen)    
        print(self.x, self.y, self.theta)

    def draw(self, screen):
        # print("Draw")
        # rotated_image = py.transform.rotate(py.Surface((40, 20)), -math.degrees(self.theta))
        # rect = rotated_image.get_rect(center=(self.x, self.y))
        # screen.blit(rotated_image, rect)
        screen.fill((0,0,0))
        global landmarks
        for landmark in landmarks:
            x, y = landmark
            py.draw.circle(screen, (255, 0, 0), (int(x), int(y)), 10)

        l = self.wheel_base
        b = self.wheel_diameter
        p1 = (self.x + (b/2)*math.cos(self.theta) + (l/2)*math.sin(self.theta), self.y + (b/2)*math.sin(self.theta) + (l/2)*math.cos(self.theta))
        p2 = (self.x + (b/2)*math.cos(self.theta) - (l/2)*math.sin(self.theta), self.y + (b/2)*math.sin(self.theta) - (l/2)*math.cos(self.theta))
        p3 = (self.x - (b/2)*math.cos(self.theta) - (l/2)*math.sin(self.theta), self.y - (b/2)*math.sin(self.theta) - (l/2)*math.cos(self.theta))
        p4 = (self.x - (b/2)*math.cos(self.theta) + (l/2)*math.sin(self.theta), self.y - (b/2)*math.sin(self.theta) + (l/2)*math.cos(self.theta))
        
        # py.draw.circle(screen, (0, 0, 255), (int(self.x), int(self.y)), 5)
        py.draw.line(screen, (0, 0, 255), p1, p2, 3)
        py.draw.line(screen, (0, 0, 255), p2, p3, 3)
        py.draw.line(screen, (0, 0, 255), p3, p4, 3)
        py.draw.line(screen, (0, 0, 255), p4, p1, 3)
        # py.display.flip()
        py.display.update()

    def calc_vect(self, xf, yf):
        self.theta = (self.theta + np.pi) % (2 * np.pi) - np.pi
        val = math.atan2((yf-self.y),(xf-self.x)) - self.theta
        # val = (val/(2*np.pi))*0.3
        if (val > (3.14/180.0)):
            return [0, 1] # v_t, w_t
        elif (val < -(3.14/180.0)):
            return [0, -1]
        else:
            return [1, 0]

def move_to_goal(landmarks, screen):
    m , n = landmarks[0]
    robot = Robot(m, n, 0, 10, 30, screen)
    v = 3
    w = 0.3
    dt = 0.1
    v_t = 0
    w_t = 0
    print("mg")
    def eucludian_distance(x1, y1, x2, y2):
        dis =  math.sqrt(math.pow(x1 - x2, 2) + math.pow(y1 - y2, 2))
        print(dis)
        return dis
    # odometry_data = []
    for i in range(len(landmarks) - 1):
        xi, yi = landmarks[i]
        xf, yf = landmarks[i + 1]
        distance = eucludian_distance(xi, yi, xf, yf)
        while distance > 1:
            for event in py.event.get():
                if event.type == py.QUIT:
                    py.quit()
                    exit()
            v_t, w_t = robot.calc_vect(xf, yf)
            # odometry_data.append([v * v_t, w * w_t, dt])
            robot.odom(v * v_t, w * w_t, dt)
            distance = eucludian_distance(robot.x, robot.y, xf, yf)
 

"""class Robot:
    def __init__(self, x, y, theta, wheel_diameter, wheel_base, screen):
        self.x = x
        self.y = y
        self.theta = theta
        self.wheel_diameter = wheel_diameter
        self.wheel_base = wheel_base
        self.draw(screen)
        self.dis = 0
        self.ang = 0
        print("Robot")

    def odom(self, v, w, dt):
        print("odom")
        vl = v - (w * self.wheel_base) / 2
        vr = v + (w * self.wheel_base) / 2
        
        #Itroducing error to the robot odometry data
        if self.dis > 1:
            self.dis = 0
            dx = (vl + vr) * dt * 0.5 * math.cos(self.theta) + rnd.gauss(0.05, 0.025)
            dy = (vl + vr) * dt * 0.5 * math.sin(self.theta) + rnd.gauss(0.05, 0.025)
        else:
            self.dis += dt*v
            dx = (vl + vr) * dt * 0.5 * math.cos(self.theta)
            dy = (vl + vr) * dt * 0.5 * math.sin(self.theta)

        if self.ang >= 3.14:
            self.ang = 0
            dtheta = w * dt + rnd.gauss(0.05, 0.025)
        else:
            self.ang += dt*w
            dtheta = w * dt 
        self.x += dx
        self.y += dy
        self.theta += dtheta
        Robot.draw(self, screen)    
        print(self.x, self.y, self.theta)

    def draw(self, screen):
        print("Draw")
        # rotated_image = py.transform.rotate(py.Surface((40, 20)), -math.degrees(self.theta))
        # rect = rotated_image.get_rect(center=(self.x, self.y))
        # screen.blit(rotated_image, rect)
        py.draw.circle(screen, (255, 255, 0), (int(self.x + (self.wheel_base + 10) / 2), int(self.y)), self.wheel_diameter // 2)
        py.draw.circle(screen, (255, 255, 0), (int(self.x - (self.wheel_base + 10) / 2), int(self.y)), self.wheel_diameter // 2)
        py.display.flip()
        # py.display.update()"""

"""def move_to_goal(landmarks, screen):
    m , n = landmarks[0]
    robot = Robot(m, n, 0, 10, 3, screen)
    v = 1
    w = 0.3
    dt = 0.1
    print("mg")
    def eucludian_distance(x1, y1, x2, y2):
        dis =  math.sqrt(math.pow(x1 - x2, 2) + math.pow(y1 - y2, 2))
        print(dis)
        return dis
    for i in range(len(landmarks) - 1):
        xi, yi = landmarks[i]
        xf, yf = landmarks[i + 1]

        dtheta = math.atan2(yf - robot.y, xf - robot.x)
        print("mg1")
        while not (((dtheta - robot.theta)) < 0.015 and ((dtheta - robot.theta)) > -0.015):
            if (dtheta - robot.theta) > 0:
                robot.odom(0, w, dt)
            elif (dtheta - robot.theta) < 0:
                robot.odom(0, -w, dt)
            print("mg2",dtheta - robot.theta)
            py.display.flip()
            #py.display.update()
        robot.odom(0, 0, dt)
        while not (eucludian_distance(xf, yf, robot.x, robot.y) < 30 and eucludian_distance(xf, yf, robot.x, robot.y) > -30):
            if eucludian_distance(xf, yf, robot.x, robot.y) > 0:
                robot.odom(v, 0, dt)
            elif eucludian_distance(xf, yf, robot.x, robot.y) < 0:
                robot.odom(-v, 0, dt)
            print("mg3")
            #robot.odom(v, 0, dt)
            py.display.flip()
            # py.display.update()
        robot.odom(0, 0, dt)
    #running = False
    pass"""
if __name__ == '__main__':
    py.init()
    screen = py.display.set_mode((800, 600))

    landmark = Landmark()
    num_landmarks = 5
    landmarks = landmark.generate_landmarks(num_landmarks, 800, 600, 11 )
    print("dc1")
    for landmark in landmarks:
        x, y = landmark
        py.draw.circle(screen, (255, 0, 0), (int(x), int(y)), 10)
        # print("dc")
    py.display.flip()
    #py.display.update()

    #robot = Robot(110, 110, 0, 10, 3, screen)
    move_to_goal(landmarks, screen)
    py.display.flip()
    running = False
    while running:
        for event in py.event.get():
            #move_to_goal(landmarks, screen)
            if event.type == py.QUIT:
                running = True

    py.quit()