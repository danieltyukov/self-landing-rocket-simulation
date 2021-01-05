import matplotlib.pyplot as plt
import numpy as np

class Rocket():
    def __init__(self, rocketMass, fuelMass, rateOfConsumption, thrust, secondIncrement=0.01, gravity=-9.8, time=0.0, height=0.0):
        self.gravity = gravity
        self.rocketMass = rocketMass # grams
        self.fuelMass = fuelMass # grams
        self.rateOfConsumption = rateOfConsumption # grams/second
        self.thrust = thrust # Newtons
        self.currentMass = rocketMass + fuelMass # grams
        self.secondIncrement = secondIncrement # seconds
        self.time = time # seconds
        self.height = height # meters
        self.counter = 0
        self.velocity = 0
        self.time2 = time
        self.acceleration = 0

    @staticmethod
    def inKg(grams):
        return grams / 1000

    @staticmethod
    def inNewtons(grams):
        kg = Rocket.inKg(grams)
        return kg * 9.8

    def calcAcceleration(self):
        if self.currentMass >= self.rocketMass:
            self.acceleration = (self.thrust - Rocket.inNewtons(self.currentMass)) / Rocket.inKg(self.currentMass)
            return self.acceleration
        else:
            self.acceleration = -9.8
            return self.acceleration

    def calcCurrentMass(self):
        if self.currentMass >= self.rocketMass:
            self.currentMass = self.rocketMass + (self.fuelMass - (self.rateOfConsumption * self.time))
        return self.currentMass

    def incrementTime(self):
        self.time += self.secondIncrement

class Simulation():
    def __init__(self, rocket):
        self.rocket = rocket

    def runSim(self):
        times = []
        heights = []
        velocities = []
        accelerations = []
        masses = []
        while(self.rocket.height >= 0):
            masses.append(self.rocket.calcCurrentMass())
            times.append(self.rocket.time)
            accelerations.append(self.rocket.calcAcceleration())
            if len(velocities) > 0:
                self.rocket.velocity = velocities[-1] + (accelerations[-1] * self.rocket.secondIncrement)
            else:
                self.rocket.velocity = 0
            velocities.append(self.rocket.velocity)
            if len(heights) > 0:
                self.rocket.height = heights[-1] + (velocities[-1] * self.rocket.secondIncrement) + (.5 * accelerations[-1] * (self.rocket.secondIncrement**2))
            else:
                self.rocket.height = 0
            heights.append(self.rocket.height)
            self.rocket.incrementTime()

        fig, (pt, vt, at) = plt.subplots(1, 3, sharex=True)
        xmin = 0
        # pt.set(xlim=(xmin, xmax))
        pt.set_title("Position v. Time")
        pt.set_xlabel("Time (s)")
        pt.set_ylabel("Meters (m)")
        pt.grid()

        vt.set_title("Velocity v. Time")
        vt.axhline(color='grey', linestyle="--")
        vt.set_xlabel("Time (s)")
        vt.set_ylabel("Velocity (m/s)")
        vt.grid()

        at.set_title("Acceleration v. Time")
        at.axhline(color='grey', linestyle="--")
        at.set_xlabel("Time (s)")
        at.set_ylabel("Acceleration (m/s^2)")
        at.grid()

        fig.suptitle("Projectile Motion Graphs")
        
        pt.plot(times, heights)
        vt.plot(times, velocities)
        at.plot(times, accelerations)

        fig.set_size_inches(18, 6)
        fig.set_dpi(100)
        plt.savefig("RocketGraphs_Other")
        plt.show()
        plt.close()

# https://en.wikipedia.org/wiki/Falcon_9#:~:text=Falcon%209%20is%20a%20partially,RP%2D1)%20as%20propellants.
# (rocketMass (g), fuelMass (g), rateOfConsumption (g/s), thrust (N), secondIncrement=0.01):
falcon = Rocket(98286580, 450767420, 1451496, 7607000)
rocket = Rocket(800, 100, 50, 9)
sim = Simulation(falcon)
sim.runSim()