import pygame
import requests
import sys
import random
from pygame.locals import *


# === Setup Pygame and Params ===
pygame.init()
vec = pygame.math.Vector2
HEIGHT = 500
WIDTH = 500
FPS = 2
CELL_COLOR_MULTIPLIER = 255
ROVER_URL = "http://192.168.1.79:5000/heatmap"
DEBUG = False

# === Window Inint ===
display = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("Obstacle + Thermal: Heatmap Viewer")
clock = pygame.time.Clock()


# === Get Grid from Rover ===
def fetch_grid():
    try:
        res = requests.get(ROVER_URL)
        data = res.json()
        return data["grid"]
    except Exception as e:
        print("Error getting grid:", e)
        return None

def value_to_color(value):
    if value <= 0.0:
        return (30, 30, 30)       # dark gray for unexplored
    elif value < 0.33:
        return (0, 200, 0)        # green = safe
    elif value < 0.66:
        return (255, 165, 0)      # orange = warning
    else:
        return (255, 0, 0)        # red = danger

# === Draw Grid ===
def draw_grid(grid):
    if grid is None:
        return
    rows = len(grid)
    cols = len(grid[0]) if rows > 0 else 0
    cell_width = WIDTH // cols
    cell_height = HEIGHT // rows
    for y, row in enumerate(grid):
        for x, value in enumerate(row):
            color = value_to_color(value)
            rect = pygame.Rect(x * cell_width, y * cell_height, cell_width, cell_height)
            pygame.draw.rect(display, color, rect)

def generate_fake_grid(rows=50, cols=50):
    return [[random.uniform(0.0, 1.0) for _ in range(cols)] for _ in range(rows)]

while True:
    for event in pygame.event.get():
        if event.type == QUIT:
            pygame.quit()
            sys.exit()

    display.fill((0, 0, 0))
    
    grid = fetch_grid() if not DEBUG else generate_fake_grid()
    draw_grid(grid)

    pygame.display.update()
    clock.tick(FPS)
