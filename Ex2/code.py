#Kacper Marzol

import pygame
import math
import numpy as np
import random

class Bead:
    def __init__(self, radius, mass, pos):
        self.radius = radius
        self.mass = mass
        self.pos = np.array(pos, dtype=float)
        self.prevPos = np.array(pos, dtype=float)
        self.vel = np.array([0.0, 0.0], dtype=float)

    def startStep(self, dt, gravity):
        self.vel += gravity * dt
        self.prevPos[:] = self.pos
        self.pos += self.vel * dt

    def keepOnWire(self, center, radius):
        dir = self.pos - center
        len_dir = np.linalg.norm(dir)
        if len_dir == 0.0:
            return 0.0
        dir /= len_dir
        lambda_val = radius - len_dir
        self.pos += dir * lambda_val
        return lambda_val

    def endStep(self, dt):
        self.vel = (self.pos - self.prevPos) / dt

def handleBeadBeadCollision(bead1, bead2, restitution=1.0):
    dir = bead2.pos - bead1.pos
    d = np.linalg.norm(dir)
    if d == 0 or d > bead1.radius + bead2.radius:
        return
    dir /= d
    corr = (bead1.radius + bead2.radius - d) / 2
    bead1.pos -= dir * corr
    bead2.pos += dir * corr

    v1 = np.dot(bead1.vel, dir)
    v2 = np.dot(bead2.vel, dir)
    m1, m2 = bead1.mass, bead2.mass

    newV1 = (m1 * v1 + m2 * v2 - m2 * (v1 - v2) * restitution) / (m1 + m2)
    newV2 = (m1 * v1 + m2 * v2 - m1 * (v2 - v1) * restitution) / (m1 + m2)

    bead1.vel += dir * (newV1 - v1)
    bead2.vel += dir * (newV2 - v2)

pygame.init()
screen_width = 800
screen_height = 600
screen = pygame.display.set_mode((screen_width, screen_height))
clock = pygame.time.Clock()

gravity = np.array([0.0, -10.0])
dt = 1/60
numSteps = 100

simMinWidth = 2.0
cScale = min(screen_width, screen_height) / simMinWidth
simWidth = screen_width / cScale
simHeight = screen_height / cScale

wireCenter = np.array([simWidth/2, simHeight/2])
wireRadius = simMinWidth * 0.4

num_beads = 5
beads = []

angle = 0.0
for i in range(num_beads):
    r = 0.05 + random.random() * 0.1
    mass = math.pi * r * r
    pos = wireCenter + wireRadius * np.array([math.cos(angle), math.sin(angle)])
    beads.append(Bead(r, mass, pos))
    angle += (math.pi / num_beads) * (0.8 + 0.4 * random.random())

def cX(pos):
    return int(pos[0] * cScale)

def cY(pos):
    return int(screen_height - pos[1] * cScale)

def simulate():
    sdt = dt / numSteps
    for _ in range(numSteps):
        for bead in beads:
            bead.startStep(sdt, gravity)
        for bead in beads:
            bead.keepOnWire(wireCenter, wireRadius)
        for bead in beads:
            bead.endStep(sdt)
        for i in range(len(beads)):
            for j in range(i):
                handleBeadBeadCollision(beads[i], beads[j])

def draw():
    screen.fill((255,255,255))
    pygame.draw.circle(screen, (0,0,0), (cX(wireCenter), cY(wireCenter)), int(wireRadius*cScale), 2)
    for bead in beads:
        pygame.draw.circle(screen, (255,0,0), (cX(bead.pos), cY(bead.pos)), int(bead.radius*cScale))

running = True
while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
    simulate()
    draw()
    pygame.display.flip()
    clock.tick(60)
pygame.quit()
