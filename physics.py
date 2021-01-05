import time

import graphics as g
import numpy as np

GRAVITY = np.array((0, 9.81))

class RigidBody(object):
    

    # TODO figure out center_of_mass and moment_of_inertia
    # programatically.
    def __init__(self, 
                 mass, 
                 moment_of_inertia, 
                 position=(0, 0), 
                 rotation=0):
        
        self._m = mass
        self._mi = moment_of_inertia
        self._p = np.array(position, dtype=np.float64) 
        self._v = np.array((0, 0), dtype=np.float64)
        self._r = rotation
        self._av = 0

        # Reset every tick.
        self._f = np.array((0, 0), dtype=np.float64)
        self._t = 0

    @property
    def position(self):
        return self._p

    @property
    def rotation(self):
        return self._r

    def apply_force(self, force, contact_point=None):
        
        cp = contact_point or self._p
        self._f += force
        self._t += np.cross(self._p - cp, force)

    def update(self, dt):
        
        # Linear component.
        a = GRAVITY
        a += self._f / self._m
        self._v += a * dt
        self._p += self._v * dt

        # Angular component.
        aa = self._t / self._mi
        self._av += aa  * dt
        self._r += self._av * dt

        # Clear forces.
        self._f = np.array((0, 0), dtype=np.float64)
        self._t = 0


class Simulation(object):

    def __init__(self):
        self._window = g.GraphWin('test', 400, 400, autoflush=False)       
        self._body = RigidBody(
                mass=100, 
                moment_of_inertia=(100*10**4)/12, 
                position=(100, 100),
                rotation=np.pi/4)

    def _draw_body(self):
        x, y = self._body.position
        w, h = 5, 5
        r = self._body.rotation
        R = np.array([[np.cos(r), -np.sin(r)], [np.sin(r), np.cos(r)]])
        points = [np.array((x - w, y - h)), np.array((x + w, y - h)),
                  np.array((x + w, y + h)), np.array((x - w, y + h))]
        gPoints = []
        for point in points:
            rpoint= np.dot(R, point - self._body.position) + self._body.position
            print(point, rpoint, r)
            gPoints.append(g.Point(*rpoint))
        return g.Polygon(*gPoints)


    def _draw(self, drawables):
        for drawable in drawables:
            drawable.draw(self._window)

    def _undraw(self, drawables):
        for drawable in drawables:
            drawable.undraw()

    def run(self):
        """Runs the simulation until the user closes out."""
        drawables = []
        t0 = time.time()
        while self._window.isOpen():
            # Resolve time since last tick.
            t = time.time()
            dt = t - t0
            t0 = t

            # Run simulation for 1 tick.
            self._undraw(drawables)
            drawables = []
            self._body.update(dt)
            drawables.extend([self._draw_body()])
            self._draw(drawables)
            g.update(60)

if __name__ == '__main__':
    Simulation().run()

if __name__ == '__main__':
    size = 10
    mass = 100
    pos = (100, 100)