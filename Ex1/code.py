#Kacper Marzol

import pygame
import sys
import random
import math

# pygame for the visualization environment, as suggested by llm
pygame.init()
WIDTH, HEIGHT = 800, 600
screen = pygame.display.set_mode((WIDTH, HEIGHT))
clock = pygame.time.Clock()

# parameters
# define how big the simulation world is in simulation units
sim_min_width = 20.0
c_scale = min(WIDTH, HEIGHT) / sim_min_width
sim_width = WIDTH / c_scale
sim_height = HEIGHT / c_scale


# coordinate converters simulation space to screen space
def cX(pos):
    return pos["x"] * c_scale

def cY(pos):
    return HEIGHT - pos["y"] * c_scale

gravity = {"x": 0.0, "y": -10.0} # no horizontal gravity, gravity pulls downwards
time_step = 1.0 / 60.0
air_resistance = 0.05
restitution = 0.7  # balls lose energy with each bounce

# creating balls
# each balls has random size, position, speed, color
balls = []
num_balls = 10
for _ in range(num_balls):
    balls.append({
        "radius": 0.3,
        "pos": {"x": random.uniform(1, sim_width - 1),
                "y": random.uniform(1, sim_height / 2)},
        "vel": {"x": random.uniform(-5, 5),
                "y": random.uniform(5, 15)},
        "color": (
            random.randint(0, 255),
            random.randint(0, 255),
            random.randint(0, 255)
        )
    })

# fucntion for drawing balls
def draw():
    screen.fill((255, 255, 255))
    for ball in balls:
        pygame.draw.circle(
            screen,
            ball["color"],
            (int(cX(ball["pos"])), int(cY(ball["pos"]))),
            int(c_scale * ball["radius"])
        )
    pygame.display.flip()

# ball collisions
def collide_balls(b1, b2):
    dx = b2["pos"]["x"] - b1["pos"]["x"]
    dy = b2["pos"]["y"] - b1["pos"]["y"]
    dist = math.sqrt(dx**2 + dy**2)
    min_dist = b1["radius"] + b2["radius"]
    if dist < min_dist: #it means they overlap so collision happened.

        # create unit vector (nx, ny) pointing from ball 1 to 2. direction of the collision
        nx = dx / dist
        ny = dy / dist

        # Separate overlapping balls
        overlap = min_dist - dist
        b1["pos"]["x"] -= nx * overlap /2
        b1["pos"]["y"] -= ny * overlap/2
        b2["pos"]["x"] += nx * overlap/2
        b2["pos"]["y"] += ny * overlap/2

        #how fast the balls are moving  from each other along the collision direction
        vx_rel = b1["vel"]["x"] - b2["vel"]["x"]
        vy_rel = b1["vel"]["y"] - b2["vel"]["y"]
        vel_along_normal = vx_rel * nx + vy_rel * ny

        e = restitution
        j = (-(1 + e) * vel_along_normal)/2 #impulse formula simplified so that mass is always one for all balls

        impulse_x = j * nx
        impulse_y = j * ny
        b1["vel"]["x"] += impulse_x
        b1["vel"]["y"] += impulse_y
        b2["vel"]["x"] -= impulse_x
        b2["vel"]["y"] -= impulse_y

def simulate():
    for ball in balls:
        # Air resistance
        ball["vel"]["x"] *= (1 - air_resistance * time_step)
        ball["vel"]["y"] *= (1 - air_resistance * time_step)

        # Gravity
        ball["vel"]["x"] += gravity["x"] * time_step
        ball["vel"]["y"] += gravity["y"] * time_step

        # Update position
        ball["pos"]["x"] += ball["vel"]["x"] * time_step
        ball["pos"]["y"] += ball["vel"]["y"] * time_step

        # Wall collisions
        if ball["pos"]["x"] - ball["radius"] < 0.0:
            ball["pos"]["x"] = ball["radius"]
            ball["vel"]["x"] = -ball["vel"]["x"] * restitution

        if ball["pos"]["x"] + ball["radius"] > sim_width:
            ball["pos"]["x"] = sim_width - ball["radius"]
            ball["vel"]["x"] = -ball["vel"]["x"] * restitution

        if ball["pos"]["y"] - ball["radius"] < 0.0:
            ball["pos"]["y"] = ball["radius"]
            ball["vel"]["y"] = -ball["vel"]["y"] * restitution

        if ball["pos"]["y"] + ball["radius"] > sim_height:
            ball["pos"]["y"] = sim_height - ball["radius"]
            ball["vel"]["y"] = -ball["vel"]["y"] * restitution

    # Ball collisions
    for i in range(len(balls)):
        for j in range(i + 1, len(balls)):
            collide_balls(balls[i], balls[j])

def bounce_near_mouse(mx, my):
    # screen pixels to simulation units
    sim_x = mx / c_scale
    sim_y = (HEIGHT - my) / c_scale

    # check each ball for being close to the click
    for ball in balls:
        dx = ball["pos"]["x"] - sim_x
        dy = ball["pos"]["y"] - sim_y
        dist = math.sqrt(dx**2 + dy**2)
        if dist < 1.0:
            ball["vel"]["y"] += 10 #bounce up
            ball["vel"]["x"] += random.uniform(-2, 2) #go left or right

while True:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            pygame.quit()
            sys.exit()
        elif event.type == pygame.MOUSEBUTTONDOWN:
            bounce_near_mouse(*event.pos)
    simulate()
    draw()
    clock.tick(60)
