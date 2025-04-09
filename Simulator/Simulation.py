import pygame
import math

# Initialize Pygame
pygame.init()

# Screen settings
WIDTH, HEIGHT = 800, 600
screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("Autonomous Car Simulator")

# Clock
clock = pygame.time.Clock()
FPS = 60

# Colors
WHITE = (255, 255, 255)
RED = (200, 0, 0)
GRAY = (50, 50, 50)

# Car settings
CAR_WIDTH, CAR_HEIGHT = 30, 50

class Car:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.angle = 0  # degrees
        self.speed = 0
        self.max_speed = 4
        self.acceleration = 0.1
        self.friction = 0.05
        self.rotation_speed = 2
        self.width = CAR_WIDTH
        self.height = CAR_HEIGHT

        # Create a base surface for the car
        self.original_image = pygame.Surface((self.width, self.height), pygame.SRCALPHA)
        self.original_image.fill(RED)
        self.image = self.original_image
        self.rect = self.image.get_rect(center=(self.x, self.y))

    def update(self, keys):
        if keys[pygame.K_UP]:
            self.speed = min(self.speed + self.acceleration, self.max_speed)
        elif keys[pygame.K_DOWN]:
            self.speed = max(self.speed - self.acceleration, -self.max_speed / 2)
        else:
            if self.speed > 0:
                self.speed = max(self.speed - self.friction, 0)
            elif self.speed < 0:
                self.speed = min(self.speed + self.friction, 0)

        if keys[pygame.K_RIGHT]:
            self.angle += self.rotation_speed * (self.speed / self.max_speed)
        if keys[pygame.K_LEFT]:
            self.angle -= self.rotation_speed * (self.speed / self.max_speed)

        # Update position
        rad = math.radians(self.angle)
        self.x += math.sin(rad) * self.speed
        self.y -= math.cos(rad) * self.speed

    def draw(self, surface):
        self.image = pygame.transform.rotate(self.original_image, -self.angle)
        self.rect = self.image.get_rect(center=(self.x, self.y))
        self.mask = pygame.mask.from_surface(self.image)
        surface.blit(self.image, self.rect.topleft)

    def save_position(self):
        self.prev_x = self.x
        self.prev_y = self.y
        self.prev_angle = self.angle

    def rewind_position(self):
        self.x = self.prev_x
        self.y = self.prev_y
        self.speed = 0  # Stop the car to prevent jitter


class ObstacleManager:
    def __init__(self):
        self.obstacles = []

    def add_obstacle(self, x, y):
        size = 20
        surface = pygame.Surface((size, size), pygame.SRCALPHA)
        surface.fill(GRAY)
        rect = surface.get_rect(center=(x, y))
        mask = pygame.mask.from_surface(surface)
        self.obstacles.append({"surface": surface, "rect": rect, "mask": mask})

    def draw(self, surface):
        for obs in self.obstacles:
            surface.blit(obs["surface"], obs["rect"].topleft)


# Initialize car and obstacle manager
car = Car(WIDTH // 2, HEIGHT // 2)
obstacles = ObstacleManager()

# Main loop
running = True
while running:
    screen.fill(WHITE)
    keys = pygame.key.get_pressed()

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

        # Place obstacle with left click
        if event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
            mx, my = pygame.mouse.get_pos()
            obstacles.add_obstacle(mx, my)

    car.save_position()
    car.update(keys)
    car.draw(screen)
    obstacles.draw(screen)



    for obs in obstacles.obstacles:
        offset = (int(obs["rect"].x - car.rect.x), int(obs["rect"].y - car.rect.y))
        if car.mask.overlap(obs["mask"], offset):
            # Try moving only X, then only Y — keep the one that doesn't collide
            car_x_before = car.x
            car_y_before = car.y

            # First try X only
            car.x = car.prev_x
            car.y = car_y_before
            car.draw(screen)  # Need to update mask/rect
            for obs in obstacles.obstacles:
                offset = (int(obs["rect"].x - car.rect.x), int(obs["rect"].y - car.rect.y))
                if car.mask.overlap(obs["mask"], offset):
                    car.x = car_x_before  # X is bad

            # Then try Y only
            car.y = car.prev_y
            car.draw(screen)
            for obs in obstacles.obstacles:
                offset = (int(obs["rect"].x - car.rect.x), int(obs["rect"].y - car.rect.y))
                if car.mask.overlap(obs["mask"], offset):
                    car.y = car_y_before  # Y is bad

            car.speed = 0  # Still stop the car from pushing in

            break




    pygame.display.flip()
    clock.tick(FPS)

pygame.quit()
