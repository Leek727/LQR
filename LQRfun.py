import numpy as np
import pygame
import control

pygame.init()

screen = pygame.display.set_mode([500, 500])


class Kraken():
    def __init__(self):
        # FOC kraken specs stolen from 971 code
        # Stall Torque in N m
        self.stall_torque = .4982
        # Stall Current in Amps
        self.stall_current = 85
        # Free Speed in RPM
        self.free_speed = 19300.0
        # Free Current in Amps
        self.free_current = 1.2
        # Effective mass of the shooter in kg.
        # This rough estimate should about include the effect of the masses
        # of the gears. If this number is too low, the eigen values of self.A
        # will start to become extremely small.
        self.J = 200
        # Resistance of the motor, divided by the number of motors.
        self.R = 12.0 / self.stall_current / 2.0
        # Motor velocity constant
        self.Kv = ((self.free_speed / 60.0 * 2.0 * np.pi) /
                   (12.0 - self.R * self.free_current))
        # Torque constant
        self.Kt = self.stall_torque / self.stall_current

class AngularSystem():
    def __init__(self, motor, G, J):
        # Arm state space model
        # G > 1 is gear increase
        self.motor = motor
        self.G = G
        self.J = J
        self.dt = .005


        C1 = -(self.motor.Kt * self.G**2) / (self.motor.R *
                              self.J * self.motor.Kv)
        C2 = (self.motor.Kt * self.G) / (self.J * self.motor.R)

        self.A_continuous = np.matrix([[0, 1], [0, C1]])
        self.B_continuous = np.matrix([[0], [C2]])


        self.A_discrete = np.eye(2) + self.A_continuous * self.dt
        self.B_discrete = self.B_continuous * self.dt

        self.K = self.getK()
        self.setpoint = np.matrix([0,0]).T


    def calcNextState(self, state, u):
        return self.A_discrete * state + self.B_discrete * u


    def getK(self):
        # diagonal matrix - qi is the weight of the ith state variable
        Q = np.matrix([[1500, 0], [0, 1]])
        R = np.matrix([[.001]]) # penalize control effort

        return np.matrix(control.lqr(self.A_continuous, self.B_continuous, Q, R)[0])

    def getLQROutput(self, state):
        return -self.K * (state - self.setpoint)

turret = AngularSystem(Kraken(),200,3)
state = np.matrix([1.5,0]).T
KGain = turret.getK()



print(KGain)

counter = 0
clock = pygame.time.Clock()
running = True
while running:
    counter += 1
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    # set state angle to mouse click
    # check if mouse click
    if pygame.mouse.get_pressed()[0]:
        turret.setpoint = np.arctan2(pygame.mouse.get_pos()[1] - 250, pygame.mouse.get_pos()[0] - 250)
        #state[0] = np.arctan2(pygame.mouse.get_pos()[1] - 250, pygame.mouse.get_pos()[0] - 250)
            

    # clear screen
    screen.fill((0,0,0))

    
    u = turret.getLQROutput(state)
    battery_voltage = 13
    u = min(battery_voltage, max(-battery_voltage, u))
    state = turret.calcNextState(state, u)

    # the state is [angle, angular velocity]
    # we want to draw a line from the center of the screen to the end of the arm

    # draw line
    x = 250 + 100 * np.cos(state[0])
    y = 250 + 100 * np.sin(state[0])
    x = int(x)
    y = int(y)

    pygame.draw.line(screen, (0, 0, 255), (250, 250), (x, y), 3)


    # make sure sim is in real time using turret.dt and clock tick
    dt = clock.tick(1/turret.dt)

    
    if counter % 100 == 0:
        print(f"Error: {turret.setpoint - state[0]}")
        print(f"U: {u}")
        print(f"A: {u / turret.motor.R}")

    pygame.display.flip()

pygame.quit()
