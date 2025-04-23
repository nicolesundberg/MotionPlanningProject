import random
import numpy as np
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

GOAL_RADIUS = 10

class Car:
    def __init__(self, x, y, angle):
        self.x = x
        self.y = y
        self.angle = angle # degrees
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

    def update(self):
        rad = math.radians(self.angle)
        self.x += math.sin(rad) * self.speed
        self.y -= math.cos(rad) * self.speed

    def draw(self, surface):
        # Rotate the image
        self.image = pygame.transform.rotate(self.original_image, -self.angle)
        self.rect = self.image.get_rect(center=(self.x, self.y))
        self.mask = pygame.mask.from_surface(self.image)

        # Blit the car
        surface.blit(self.image, self.rect.topleft)

        # --- Draw two circles at the front ---
        # Compute angle in radians
        rad = math.radians(self.angle)

        # Compute front center of car
        front_x = self.x + (self.height / 2) * math.sin(rad)
        front_y = self.y - (self.height / 2) * math.cos(rad)

        # Compute lateral (sideways) offset
        side_dx = (self.width / 4) * math.cos(rad)
        side_dy = (self.width / 4) * math.sin(rad)

        # Left and right circle positions
        left_x = front_x - side_dx
        left_y = front_y - side_dy
        right_x = front_x + side_dx
        right_y = front_y + side_dy

        # Draw the circles
        pygame.draw.circle(surface, (0, 255, 0), (int(left_x), int(left_y)), 4)   # Green left
        pygame.draw.circle(surface, (0, 255, 0), (int(right_x), int(right_y)), 4) # Green right

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

    def add_obstacle(self, x, y, size=20):
        surface = pygame.Surface((size, size), pygame.SRCALPHA)
        surface.fill(GRAY)
        rect = surface.get_rect(center=(x, y))
        mask = pygame.mask.from_surface(surface)
        self.obstacles.append({"surface": surface, "rect": rect, "mask": mask})

    def draw(self, surface):
        for obs in self.obstacles:
            surface.blit(obs["surface"], obs["rect"].topleft)


def steer_car(car):
    if car.path_index >= len(car.path):
        car.speed = 0
        return

    # Get current waypoint
    target = car.path[car.path_index]
    print("TARGET", target, " Index:", car.path_index)

    target_x, target_y, target_theta = target
    dx = target_x - car.x
    dy = target_y - car.y
    distance = math.hypot(dx, dy)

    # Compute the desired angle (angle to next point)
    desired_angle = math.degrees(target_theta)
    angle_diff = (desired_angle - car.angle + 180) % 360 - 180

    # Turn toward target heading (not just position)
    if abs(angle_diff) > 2:
        if angle_diff > 0:   
            car.angle += car.rotation_speed
        else:
            car.angle -= car.rotation_speed

    # Match final orientation (if near goal and last point)
    if car.path_index == len(car.path) - 1:
        goal_angle_diff = (math.degrees(target_theta) - car.angle + 180) % 360 - 180
        if abs(goal_angle_diff) > 2:
            if goal_angle_diff > 0:
                car.angle += car.rotation_speed
            else:
                car.angle -= car.rotation_speed

    # Speed control
    stopping_distance = (car.speed ** 2) / (2 * car.friction + 1e-5)
    if distance <= stopping_distance:
        car.speed = max(car.speed - car.friction, 0)
    else:
        car.speed = min(car.speed + car.acceleration, car.max_speed)

    # Advance to next waypoint
    LOOKAHEAD = 20
    if distance < LOOKAHEAD and car.path_index < len(car.path) - 1:
        car.path_index += 1

    car.update()


def main():
    # Init car random start
    start_x = 100 # random.randint(0, WIDTH - CAR_HEIGHT) + CAR_HEIGHT
    start_y = 100 # random.randint(0, HEIGHT - CAR_HEIGHT) + CAR_HEIGHT
    start_angle = 180
    car = Car(start_x, start_y, start_angle)

    #Init obstacles
    obstacles = ObstacleManager()
    obstacles.add_obstacle(WIDTH//2, HEIGHT//2, 100)

    # Init goal
    goal_pos = (WIDTH - 100, HEIGHT - 100)
    # Init planner
    planner = RRT(1000, obstacles, lims=np.array([[0, WIDTH], [0, HEIGHT]]), collision_func=is_in_collision)
    # Create plan
    start = (int(car.x), int(car.y), car.angle)
    plan, root = planner.plan(start, goal_pos)
    if len(plan) > 0:
        nodes_to_draw = dfs_collect_all_nodes(root)
        car.path = plan
    else:
        nodes_to_draw = dfs_collect_all_nodes(root)
        car.path = []


    car.path_index = 0
    # print("PATH", car.path)
    # print("TO DRAW:", nodes_to_draw)
    # Main loop
    running = True
    while running:
        screen.fill(WHITE)
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

            # Place obstacle with left click
            if event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
                mx, my = pygame.mouse.get_pos()
                obstacles.add_obstacle(mx, my)

        car.save_position()
    
        steer_car(car)
        # Apply movement update
        car.update()
        car.draw(screen)
        pygame.draw.circle(screen, (0, 200, 0), goal_pos, GOAL_RADIUS)

        for i in range(len(nodes_to_draw) - 1):
            pygame.draw.line(
                screen,
                (0, 0, 255),  # Blue path
                nodes_to_draw[i][:2],
                nodes_to_draw[i + 1][:2],
                2  # Thickness
            )

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


_TRAPPED = 'trapped'
_ADVANCED = 'advanced'
_REACHED = 'reached'

def dist(p1, p2):
    return math.hypot(p1[0] - p2[0], p1[1] - p2[1])

def dfs_collect_all_nodes(root):
    """
    Performs a depth-first search starting from the root node and 
    returns a list of all TreeNodes in the connected tree.
    """
    visited = []

    def dfs(node):
        visited.append(node.state)
        for child in node.children:
            dfs(child)

    dfs(root)
    return visited


def is_in_collision(state, obstacles):
    """
    Returns True if the car at [x, y, theta] would collide with any obstacle or go out of bounds.
    Otherwise, returns False.
    """
    px, py, theta = state

    # Check screen bounds first
    if not (0 <= px < WIDTH and 0 <= py < HEIGHT):
        return True

    # Create a temporary car surface
    car_surface = pygame.Surface((CAR_WIDTH, CAR_HEIGHT), pygame.SRCALPHA)
    car_surface.fill(RED)

    # Rotate the car
    rotated_car = pygame.transform.rotate(car_surface, -math.degrees(theta))
    car_rect = rotated_car.get_rect(center=(px, py))
    car_mask = pygame.mask.from_surface(rotated_car)

    # Check overlap with obstacles
    for obs in obstacles.obstacles:
        offset = (int(obs["rect"].x - car_rect.x), int(obs["rect"].y - car_rect.y))
        if car_mask.overlap(obs["mask"], offset):
            return True

    return False


def car_step_toward(q_start, q_target, step_size, max_steer_deg=30):
    x, y, theta = q_start
    goal_dx = q_target[0] - x
    goal_dy = q_target[1] - y
    goal_angle = math.atan2(goal_dx, -goal_dy)  # match Car.update() math
    angle_diff = (goal_angle - theta + np.pi) % (2 * np.pi) - np.pi

    # Clamp steering angle
    max_steer_rad = math.radians(max_steer_deg)
    steer = np.clip(angle_diff, -max_steer_rad, max_steer_rad)

    new_theta = theta + steer

    # Now match car movement: sin and cos flipped
    new_x = x + step_size * math.sin(new_theta)
    new_y = y - step_size * math.cos(new_theta)

    return (float(new_x), float(new_y), float(new_theta))


class TreeNode:
    '''
    Class to hold node state and connectivity for building an RRT
    '''
    def __init__(self, state, parent=None):
        self.state = state
        self.children = []
        self.parent = parent


    def add_child(self, child):
        '''
        Add a child node
        '''
        self.children.append(child)


class RRTSearchTree:
    '''
    Searh tree used for building an RRT
    '''
    def __init__(self, init):
        '''
        init - initial tree configuration
        '''
        self.root = TreeNode(init)
        self.nodes = [self.root]
        self.edges = []


    def find_nearest(self, s_query):
        '''
        Find node in tree closets to s_query
        returns - (nearest node, dist to nearest node)
        '''
        min_d = 1000000
        nn = self.root
        for n_i in self.nodes:
            d = np.linalg.norm(s_query - n_i.state[:2])
            if d < min_d:
                nn = n_i
                min_d = d
        return (nn, min_d)


    def add_node(self, node, parent):
        '''
        Add a node to the tree
        node - new node to add
        parent - nodes parent, already in the tree
        '''
        self.nodes.append(node)
        self.edges.append((parent.state, node.state))
        node.parent = parent
        parent.add_child(node)


    def get_states_and_edges(self):
        '''
        Return a list of states and edgs in the tree
        '''
        states = np.array([n.state for n in self.nodes])
        return (states, self.edges)


    def get_back_path(self, n):
        '''
        Get the path from the root to a specific node in the tree
        n - node in tree to get path to
        '''
        path = []
        while n.parent is not None:
            path.append(n.state)
            n = n.parent
        path.append(n.state)
        path.reverse()
        return path


class RRT(object):
    '''
    Rapidly-Exploring Random Tree Planner
    '''
    def __init__(self, num_samples, obstacles, num_dimensions=2, step_length = 10, lims = None,
                 connect_prob = 0.05, collision_func=None):
        '''
        Initialize an RRT planning instance
        '''
        self.K = num_samples
        self.n = num_dimensions
        self.epsilon = step_length
        self.connect_prob = connect_prob
        self.obstacles = obstacles

        self.in_collision = collision_func
        if collision_func is None:
            self.in_collision = self.fake_in_collision

        # Setup range limits
        self.limits = lims
        if self.limits is None:
            self.limits = []
            for n in range(num_dimensions):
                self.limits.append([0,100])
            self.limits = np.array(self.limits)

        self.ranges = self.limits[:,1] - self.limits[:,0]
        self.found_path = False
    

    def plan(self, init, goal):
        return self.build_rrt(init, goal)


    def build_rrt(self, init, goal):
        '''
        Build the rrt from init to goal
        Returns path to goal or None
        '''
        self.goal = np.array(goal)
        self.init = np.array(init)
        self.found_path = False

        # Build tree and search
        self.T = RRTSearchTree(init)

        # Sample and extend
        state = (None, self.T.root)
        for _ in range(self.K):
            new_sample = self.sample(self.goal, self.connect_prob)
            state, node = self.extend(self.T, new_sample)

            if state == _REACHED:
                distance = np.linalg.norm(self.goal - node.state[:2])
                if distance <= self.epsilon:
                    # print("FOUND GOAL")
                    self.found_path = True
                    return self.T.get_back_path(node), self.T.root 
        print("DID NOT FIND GOAL", type(node))
        return [], self.T.root


    def sample(self, target, probability):
        '''
        Sample a new configuration
        Returns a configuration of size self.n bounded in self.limits
        '''
        # Return goal with connect_prob probability
        if random.random() < probability:
            return target
        else:
            randoms = np.random.rand(self.n)
            for i in range(len(randoms)):
                randoms[i] = randoms[i] * self.ranges[i] + self.limits[i][0]

            return randoms


    def extend(self, T, q):
        '''
        Perform rrt extend operation.
        q - new configuration to extend towards
        returns - tuple of (status, TreeNode)
           status can be: _TRAPPED, _ADVANCED or _REACHED
        '''
        nearest_node, nearest_node_dist = T.find_nearest(q)
        
        # Create step
        # q and nearest_node.state are now [x, y, theta]
        new_state = car_step_toward(nearest_node.state, q, self.epsilon)
        # print("NEAREST", nearest_node.state, " NEW STATE", new_state)
        # Check collision
        if self.in_collision(new_state, self.obstacles):
            return (_TRAPPED, T)

        new_node = TreeNode(new_state)
        T.add_node(new_node, nearest_node)

        # Check collision
        if self.in_collision(new_state, self.obstacles):
            return (_TRAPPED, T)

        # Add new step node
        new_node = TreeNode(new_state)
        T.add_node(new_node, nearest_node)

        if nearest_node_dist < self.epsilon:
            return (_REACHED, new_node)
        else:
            return (_ADVANCED, new_node)


if __name__ == "__main__":
    main()