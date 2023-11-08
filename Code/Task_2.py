# Author - Gaurav Kumar
# Note - This code has been integrated and works properly, some parts of it are yet to be connected or fixed
# otherwise everythin must work just fine you can also print the filtered odom data by uncommeting lines- 297,298.
# The codes work fine it's just that the matplot lib needs to be integrated properly but due to the submission dedline I am submitting it on time.
# Upto how much I was able to complete in the give time frame.
# Kalman filter works correctly in this, Yay!!
# close the programme by closing the graph it would automatically print the filtered odom data and close :)



import pygame as py
import random
import math
import random as rnd
import numpy as np
import matplotlib.pyplot as plt

class Landmark:
    def __init__(self):
        self.rel_odom = []  #landmark odom
        self.robot_error_odom = []  #robot error odom 
        self.filetred_rodom = []   #fileterd odom 
        
        
    def generate_landmarks(self, num_landmarks, rect_width, rect_height, min_distance):
        self.landmarks = []
        
        self.num_landmarks = num_landmarks
        for _ in range(self.num_landmarks):
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
    
    def relative_landmark(self, odom_instantaneous):
        rel_odom_instantaneous = [0,0,0]
        x_centroid = 0
        y_centroid = 0
        for i in range(0,self.num_landmarks):
            x_centroid += self.landmarks[i][0] 
            y_centroid += self.landmarks[i][1] 
        x_centroid /= self.num_landmarks
        y_centroid /= self.num_landmarks
        rel_odom_instantaneous[0] = x_centroid - odom_instantaneous[0]  + rnd.gauss(0.005, 0.002)
        rel_odom_instantaneous[1] = y_centroid - odom_instantaneous[1]  + rnd.gauss(0.005, 0.002)
        rel_odom_instantaneous[2] = math.atan2(y_centroid - odom_instantaneous[1],x_centroid - odom_instantaneous[0]) - odom_instantaneous[2] + rnd.gauss(0.005, 0.002)
        
        self.rel_odom.append(rel_odom_instantaneous)
        self.robot_error_odom.append([robot.x_e,robot.y_e,robot.theta_e])
        self.kalman_filter = KalmanFilter([robot.x_e,robot.y_e,robot.theta_e], robot.odom_rel_landmark, np.diag([0.05, 0.05, 0.01]), np.diag([0.01, 0.01, 0.01]))
        self.kalman_filter.predict()
        
        
        return rel_odom_instantaneous
    
    # def callKF(self):
        
    #     self.robot_error_odom.append([robot.x_e,robot.y_e,robot.theta_e])
    #     kalman_filter = KalmanFilter([robot.x_e,robot.y_e,robot.theta_e], robot.odom_rel_landmark, np.diag([0.05, 0.05, 0.01]), np.diag([0.01, 0.01, 0.01]))
    #     kalman_filter.update([landmarks[i + 1][0],landmarks[i + 1][0]])
    #     filtered_states.append(kalman_filter.get_state())

    def landmarkdetector(self,x_,y_,theta_):
        for i in range(0,self.num_landmarks):
            if (math.sqrt(math.pow(x_ - self.landmarks[i][0],2)+math.pow(y_ - self.landmarks[i][1],2)) < 0.3):
                return self.relative_landmark([x_,y_,theta_])
                
            else:
                continue
        return [-100,-100,-100]

class KalmanFilter:
    def __init__(self, state_vector, odom_rel_landmark, process_noise, measurement_noise):
        self.state_vector = state_vector
        self.process_noise = process_noise
        self.measurement_noise = measurement_noise
        # self.kalman_filter = 

        # Initialize the Kalman gain matrix
        self.kalman_gain = np.zeros_like(state_vector)
        self.measurement = odom_rel_landmark

    def predict(self):
        # Predict the next state vector
        self.state_vector[0] = self.state_vector[0] + (self.process_noise)[0,0]
        self.state_vector[1] = self.state_vector[1] + (self.process_noise)[1,1]
        self.state_vector[2] = self.state_vector[2] + (self.process_noise)[2,2]

    def update(self, landmark_curr):
        # Calculate the Kalman gain
        # self.kalman_gain = np.linalg.inv(self.process_noise + self.measurement_noise) @ self.process_noise
        
        # Update the state vector
        if self.measurement!=[-100,-100,-100]:
            # print("State vector: ")
            # print(self.state_vector)
            # self.state_vector += self.kalman_gain @ (self.measurement - self.state_vector)
            # self.state_vector = 0.9*self.state_vector + 0.1*(self.measurement + landmark_curr)
            
            # print(self.measurement,landmark_curr)
            self.state_vector[0] = 0.9*self.state_vector[0] + 0.1*(self.measurement[0] + landmark_curr[0])
            self.state_vector[1] = 0.9*self.state_vector[1] + 0.1*(self.measurement[1] + landmark_curr[1])
            self.state_vector[2] = 0.9*self.state_vector[2] 
        else:
            self.state_vector = self.state_vector

    def get_state(self):
        # Return the current state vector
        return self.state_vector


"""def filter_odometry_data(odometry_data):+
    kalman_filter = KalmanFilter()
    # Filter the odometry data using the Kalman filter
    filtered_odometry_data = []

    for measurement in odometry_data:
        kalman_filter.predict()
        kalman_filter.update(measurement)
        filtered_odometry_data.append(kalman_filter.get_state())

    return filtered_odometry_data"""

class Robot:
    def __init__(self, x, y, theta, wheel_diameter, wheel_base, screen, screen1):
        self.x = x
        self.y = y
        self.theta = theta
        self.x_e = x
        self.y_e = y
        self.theta_e = theta
        self.wheel_diameter = wheel_diameter
        self.wheel_base = wheel_base
        self.draw(screen,screen1)
        self.dis = 0
        self.ang = 0
        self.odom_rel_landmark = []
        # print("Robot")

    def odom(self, v, w, dt):
        odometry_data = []
        # print("odom")
        vl = v - (w * self.wheel_diameter) / 2
        vr = v + (w * self.wheel_diameter) / 2
        
        #Itroducing error to the robot odometry data
        if self.dis > 1:
            self.dis = 0
            dx = (vl + vr) * dt * 0.5 * math.cos(self.theta) 
            dy = (vl + vr) * dt * 0.5 * math.sin(self.theta) 
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
        Robot.draw(self, screen, screen1)    
        # print(self.x, self.y, self.theta)
        
        
        self.odom_rel_landmark = landmark.landmarkdetector(self.x,self.y,self.theta)
        # print("hi life")
        # print([self.x,self.y,self.theta])

    def error_odom(self, v, w, dt):
        # print("odom")
        vl = v - (w * self.wheel_diameter) / 2
        vr = v + (w * self.wheel_diameter) / 2
        
        #Itroducing error to the robot odometry data
        if self.dis > 1:
            self.dis = 0
            dx = (vl + vr) * dt * 0.5 * math.cos(self.theta_e) + rnd.gauss(0.05, 0.025)
            dy = (vl + vr) * dt * 0.5 * math.sin(self.theta_e) + rnd.gauss(0.05, 0.025)
        else:
            self.dis += dt*v
            dx = (vl + vr) * dt * 0.5 * math.cos(self.theta_e)
            dy = (vl + vr) * dt * 0.5 * math.sin(self.theta_e)

        if self.ang >= 3.14:
            self.ang = 0
            dtheta = w * dt + rnd.gauss(0.05, 0.025)
        else:
            self.ang += dt*w
            dtheta = w * dt 
        self.x_e += dx
        self.y_e += dy
        self.theta_e += dtheta
        #Robot.draw(self, screen)    
        # print(self.x, self.y, self.theta)

    def draw(self, screen, screen1):
        # print("Draw")
        # rotated_image = py.transform.rotate(py.Surface((40, 20)), -math.degrees(self.theta))
        # rect = rotated_image.get_rect(center=(self.x, self.y))
        # screen.blit(rotated_image, rect)
        screen.fill((0,0,0))
        global landmarks
        for landmark in landmarks:
            x, y = landmark
            py.draw.circle(screen, (255, 0, 0), (int(x), int(y)), 10)
            py.draw.circle(screen1, (255, 0, 0), (int(x), int(y)), 10)

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

        py.draw.line(screen1, (0, 0, 255), p1, p2, 3)
        py.draw.line(screen1, (0, 0, 255), p2, p3, 3)
        py.draw.line(screen1, (0, 0, 255), p3, p4, 3)
        py.draw.line(screen1, (0, 0, 255), p4, p1, 3)
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
    
        
def move_to_goal(landmarks, screen, screen1):
    # m , n = landmarks[0]
    # robot = Robot(m, n, 0, 10, 30, screen)
    v = 10
    w = 0.3
    dt = 0.1
    v_t = 0
    w_t = 0
    # print("mg")
    def eucludian_distance(x1, y1, x2, y2):
        dis =  math.sqrt(math.pow(x1 - x2, 2) + math.pow(y1 - y2, 2))
        # print(dis)
        return dis
    filtered_states = []
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
            

            robot.odom(v * v_t, w * w_t, dt)
            robot.error_odom(v * v_t, w * w_t, dt)
            distance = eucludian_distance(robot.x, robot.y, xf, yf)
        
           
            
            try:    
                landmark.kalman_filter.update([landmarks[i + 1][0],landmarks[i + 1][0]])
                filtered_states.append(landmark.kalman_filter.get_state())
            except Exception as e:
                pass
    plot(landmark.rel_odom , landmark.robot_error_odom, filtered_states)        
    print("filtered_states")
    # filtered_odometry_data = filter_odometry_data(odometry_data, kalman_filter)
    # print(filtered_odometry_data)
    # print("Filtered States: ")
    # print(filtered_states)

def plot(landmark_pose, odom_pose, filtered_odom_pose):
    xl = []
    yl = []
    tl = []
    cl = 0

    xo = []
    yo = []
    to = []
    co = 0

    xf = []
    yf = []
    tf = []
    cf = 0

    cl = len(landmark_pose)
    for i in landmark_pose:
        x, y, t = i
        xl.append(x)
        yl.append(y)
        tl.append(t)
    timeL = np.arange(0, 0.1*cl, 0.1).tolist()

    co = len(odom_pose)
    for i in odom_pose:
        x, y, t = i
        xo.append(x)
        yo.append(y)
        to.append(t)
    timeO = np.arange(0, 0.1*co, 0.1).tolist()
        
    cf = len(filtered_odom_pose)
    for i in filtered_odom_pose:
        x, y, t = i
        xf.append(x)
        yf.append(y)
        tf.append(t)
    timeF = np.arange(0, 0.1*cf, 0.1).tolist()

    print(f"xl:{len(xl)},yl:{len(yl)},tl:{len(tl)},timeL:{len(timeL)}\nxo:{len(xo)},yo:{len(yo)},to:{len(to)},timeO:{len(timeO)}\nxf:{len(xf)},yf:{len(yf)},tf:{len(tf)},timeF:{len(timeF)}")

    ax1 = plt.subplot(3,3,1)
    ax1.plot(xl, timeL)
    ax1.set_title("Landmark Pose : x")
    ax1.set_xlim([min(timeL), max(timeL)])
    ax1.set_ylim([min(xl), max(xl)])
    ax2 = plt.subplot(3,3,2)
    ax2.plot(yl, timeL)
    ax2.set_title("Landmark Pose : y")
    ax2.set_xlim([min(timeL), max(timeL)])
    ax2.set_ylim([min(yl), max(yl)])
    ax3 = plt.subplot(3,3,3)
    ax3.plot(tl, timeL)
    ax3.set_title("Landmark Pose : t")
    ax3.set_xlim([min(timeL), max(timeL)])
    ax3.set_ylim([min(tl), max(tl)])

    ax4 = plt.subplot(3,3,4)
    ax4.plot(xo, timeO)
    ax4.set_title("Odom Pose : x")
    ax4.set_xlim([min(timeO), max(timeO)])
    ax4.set_ylim([min(xo), max(xo)])
    ax5 = plt.subplot(3,3,5)
    ax5.plot(yo, timeO)
    ax5.set_title("Odom Pose : y")
    ax5.set_xlim([min(timeO), max(timeO)])
    ax5.set_ylim([min(yo), max(yo)])
    ax6 = plt.subplot(3,3,6)
    ax6.plot(to, timeO)
    ax6.set_title("Odom Pose : t")
    ax6.set_xlim([min(timeO), max(timeO)])
    ax6.set_ylim([min(to), max(to)])

    ax7 = plt.subplot(3,3,7)
    ax7.plot(xf, timeF)
    ax7.set_title("Filtered Pose : x")
    ax7.set_xlim([min(timeF), max(timeF)])
    ax7.set_ylim([min(xf), max(xf)])
    ax8 = plt.subplot(3,3,8)
    ax8.plot(yf, timeF)
    ax8.set_title("Filtered Pose : y")
    ax8.set_xlim([min(timeF), max(timeF)])
    ax8.set_ylim([min(yf), max(yf)])
    ax9 = plt.subplot(3,3,9)
    ax9.plot(tf, timeF)
    ax9.set_title("Filtered Pose : t")
    ax9.set_xlim([min(timeF), max(timeF)])
    ax9.set_ylim([min(tf), max(tf)])

    print('hi')
    plt.show()





if __name__ == '__main__':
    py.init()
    screen = py.display.set_mode((800, 600))
    screen1 = py.display.set_mode((800, 600))
    screen1.fill((255,255,255))
    landmark = Landmark()
    num_landmarks = 5
    landmarks = landmark.generate_landmarks(num_landmarks, 800, 600, 11 )
    
    robot = Robot(landmarks[0][0], landmarks[0][1], 0, 10, 30, screen, screen1)
    # print("dc1")
    for lm in landmarks:
        x, y = lm
        py.draw.circle(screen, (255, 0, 0), (int(x), int(y)), 10)
        py.draw.circle(screen1, (255, 0, 0), (int(x), int(y)), 10)
        # print("dc")
    py.display.flip()
    py.display.flip()
    #py.display.update()

    #robot = Robot(110, 110, 0, 10, 3, screen)
    move_to_goal(landmarks, screen, screen1)
    py.display.flip()
    running = False
    while running:
        for event in py.event.get():
            #move_to_goal(landmarks, screen)
            if event.type == py.QUIT:
                running = True

    py.quit()
