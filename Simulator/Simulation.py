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
FPS = 90

# Colors
WHITE = (255, 255, 255)
RED = (200, 0, 0)
GREEN = (0, 255, 0)
GRAY = (50, 50, 50)

# Car settings
CAR_WIDTH, CAR_HEIGHT = 30, 50

GOAL_RADIUS = 10

MAP_0 = [(WIDTH//2, HEIGHT//2, 100)]
MAP_1 = [(WIDTH//2 + 75, HEIGHT//2 - 75, 100), (WIDTH//2 - 75, HEIGHT//2 + 75, 100)]
MAP_0 = [(WIDTH//2, HEIGHT//2, 100)]

############### MAIN LOOP ######################

def main():
    # Init car random start
    start_x = 100 # random.randint(0, WIDTH - CAR_HEIGHT) + CAR_HEIGHT
    start_y = 100 # random.randint(0, HEIGHT - CAR_HEIGHT) + CAR_HEIGHT
    start_angle = 180
    car: Car = Car(start_x, start_y, start_angle)

    #Init obstacles
    obstacles = ObstacleManager()
    for x, y, size in MAP_1:
        obstacles.add_obstacle(x, y, size)

    # Init goal
    goal_pos = (WIDTH - 100, HEIGHT - 100, 180)
    # Init planner
    planner = RRT(1000, obstacles, car, lims=np.array([[0, WIDTH], [0, HEIGHT], [0, 360]]), collision_func=is_in_collision, num_dimensions=3, connect_prob=.2)

    # Create plan
    start = (int(car.x), int(car.y), car.angle)
    plan, root = planner.plan(start, goal_pos)
    if len(plan) > 0:
        nodes_to_draw = dfs_collect_all_nodes(root)
        car.path = plan
    else:
        nodes_to_draw = dfs_collect_all_nodes(root)
        car.path = []

    #
    compute_trajectory(car)
    trajectory_xy = [(pt[0], pt[1]) for pt in car.traj_pts]

    #### option to draw only path
    # nodes_to_draw = trajectory_xy

    # Main loop
    car.path_index = 0
    running = True
    screen_fill = WHITE
    started = False
    target_point_to_draw = None
    while running:
        dt = clock.tick(FPS) / 1000.0
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_SPACE:
                    started = True
        

        screen.fill(screen_fill)
        if started:
            # Save previous car position
            car.save_position()


            #### Compute new car position and follow path

            # target_point_to_draw = steer_car(car)
            # target_point_to_draw = None
            # follow_path_exact(car)
            
            target_point_to_draw = follow_trajectory(car, dt)

            # Check if goal was reached
            gx, gy, _ = goal_pos
            dist_to_goal = math.hypot(car.x - gx, car.y - gy)
            if dist_to_goal <= GOAL_RADIUS + 20:
                car.speed = 0
                screen_fill = GREEN

            # Apply movement update
            car.update(dt)

            # Collision correction before drawing
            for obs in obstacles.obstacles:
                offset = (obs["rect"].x - car.rect.x,
                        obs["rect"].y - car.rect.y)
                if car.mask.overlap(obs["mask"], offset):
                    car.rewind_position()
                    break

        # Draw everything
        car.draw(screen)
        pygame.draw.circle(screen, (0, 200, 0), goal_pos[:2], GOAL_RADIUS)

        for i in range(len(nodes_to_draw) - 1):
            pygame.draw.line(
                screen, (0, 0, 255), nodes_to_draw[i][:2], nodes_to_draw[i + 1][:2], 2
            )

        if target_point_to_draw:
            pygame.draw.circle(screen, GREEN, (int(target_point_to_draw[0]), int(target_point_to_draw[1])), 5)
        
        obstacles.draw(screen)

        pygame.display.flip()

    pygame.quit()


################# PATH FOLLOWING ##########################

class Car:
    def __init__(self, x, y, angle):
        self.x = x
        self.y = y
        self.angle = angle # degrees
        self.speed = 0
        self.max_speed = 40
        self.acceleration = 0.1
        self.friction = 0.05
        self.rotation_speed = 2
        self.width = CAR_WIDTH
        self.height = CAR_HEIGHT
        self.collider_buffer = 20

        # Create a base surface for the car
        self.original_image = pygame.Surface((self.width, self.height), pygame.SRCALPHA)
        self.original_image.fill(RED)
        self.image = self.original_image
        self.rect = self.original_image.get_rect(center=(self.x, self.y))
        self.mask = pygame.mask.from_surface(self.original_image)

    def update(self, dt):
        """
        Move the car by speed (px/sec) over dt (sec), then
        update its collision mask immediately.
        """
        # move
        rad = math.radians(self.angle)
        self.x += math.sin(rad) * self.speed * dt
        self.y -= math.cos(rad) * self.speed * dt

        # now rebuild image, rect, mask so collision sees the new pose
        self.image = pygame.transform.rotate(self.original_image, -self.angle)
        self.rect  = self.image.get_rect(center=(self.x, self.y))
        self.mask  = pygame.mask.from_surface(self.image)


    def draw(self, surface):
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
        self.speed = 0 


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


def follow_trajectory(car: Car, dt,
                      k_slow=0.3,
                      lookahead_slow=40,
                      min_speed_factor=1):
    """
    Trajectory follower with gentle CTE-based speed scaling.
      - k_slow:   how aggressively to slow when off-track (try 0.1–0.5)
      - lookahead_slow: CTE at which you’d start slowing significantly
      - min_speed_factor: never go below this fraction of v_ref
    """
    # 1) advance your time-clock
    car.t_curr = min(car.t_curr + dt, car.traj_ts[-1])

    # 2) bracket t_curr
    i = max(j for j, t in enumerate(car.traj_ts) if t <= car.t_curr)
    i = min(i, len(car.traj_ts) - 2)
    t0, t1 = car.traj_ts[i], car.traj_ts[i+1]
    frac    = (car.t_curr - t0) / (t1 - t0 + 1e-9)

    # 3) interp pose & speed
    x0, y0, th0 = car.traj_pts[i]
    x1, y1, th1 = car.traj_pts[i+1]
    v0          = car.traj_vels[i]
    v1          = car.traj_vels[i+1]

    # unwrap heading
    dth    = ((th1 - th0 + math.pi) % (2*math.pi)) - math.pi
    x_ref  = x0 + frac * (x1 - x0)
    y_ref  = y0 + frac * (y1 - y0)
    th_ref = th0 + frac * dth
    v_ref  = v0 + frac * (v1 - v0)

    # 4) compute CTE
    proj_x, proj_y = project_point_to_segment(car.x, car.y,
                                              x0, y0, x1, y1)
    cte = math.hypot(car.x - proj_x, car.y - proj_y)

    # 5) compute speed factor, then clamp it
    raw_factor = 1 - k_slow * (cte / lookahead_slow)
    speed_factor = max(min_speed_factor, min(1.0, raw_factor))
    v_ref_adj    = v_ref * speed_factor

    # 6) directly match speed
    car.speed = v_ref_adj

    # 7) pure-pursuit steering on th_ref
    desired = math.degrees(th_ref)
    err     = (desired - car.angle + 180) % 360 - 180
    turn    = max(-car.rotation_speed,
                  min(car.rotation_speed, err))
    car.angle += turn

    # return the ref-point for your debug dot
    return (x_ref, y_ref)


def compute_trajectory(car):
    # --- Trajectory time‐parameterization ---
    # 1a) compute distances between consecutive waypoints
    arc_d = [
        math.hypot(x2-x1, y2-y1)
        for (x1,y1,_),(x2,y2,_) in zip(car.path, car.path[1:])
    ]

    # 1b) cumulative arc‐length s_i
    s_list = [0]
    for d in arc_d:
        s_list.append(s_list[-1] + d)

    # 1c) choose a simple constant speed profile
    v_max = car.max_speed
    v_list = [v_max] * len(car.path)

    # 1d) time stamps t_i from dt = ds / v_avg
    t_list = [0]
    for i in range(1, len(car.path)):
        v_avg = 0.5 * (v_list[i-1] + v_list[i])
        dt = arc_d[i-1] / (v_avg + 1e-6)
        t_list.append(t_list[-1] + dt)

    # stash on the car for your main loop
    car.traj_pts = car.path       # [(x,y,theta),...]
    car.traj_vels = v_list         # [v(s0), v(s1),...]
    car.traj_ts = t_list         # [t0=0, t1, t2,...]
    car.t_curr = 0.0


def follow_path_exact(car):
    """
    Move the car instantly to the next waypoint on car.path.
    Sets car.x, car.y, and car.angle to match the path exactly.
    """
    if car.path_index < len(car.path):
        px, py, theta = car.path[car.path_index]
        car.x = px
        car.y = py
        car.angle = math.degrees(theta)
        car.path_index += 1
    else:
        # once we hit the end, stop or loop
        car.path_index = len(car.path)  # clamp
    return (car.x, car.y)


import math

def project_point_to_segment(px, py, x1, y1, x2, y2):
    dx, dy = x2 - x1, y2 - y1
    if dx == 0 and dy == 0:
        return x1, y1
    t = ((px - x1)*dx + (py - y1)*dy) / (dx*dx + dy*dy)
    t = max(0, min(1, t))
    return x1 + t*dx, y1 + t*dy


def steer_car(car, lookahead=25, off_path_tol=10):
    path = car.path
    if not path:
        car.speed = 0
        return None

    # 1) find the nearest projection on the polyline
    best_d, best_proj, best_i = float('inf'), None, 0
    for i in range(len(path)-1):
        x1,y1,_ = path[i]
        x2,y2,_ = path[i+1]
        proj = project_point_to_segment(car.x, car.y, x1, y1, x2, y2)
        d = math.hypot(car.x - proj[0], car.y - proj[1])
        if d < best_d:
            best_d, best_proj, best_i = d, proj, i

    # snap path_index so lookahead always marches forward
    car.path_index = best_i

    # 2) if you’re off-path, *immediately* head for the projection point
    if best_d > off_path_tol:
        target = best_proj

    else:
        # 3) otherwise pure-pursuit: march lookahead distance from best_proj
        remaining = lookahead
        last_x, last_y = best_proj
        target = None

        for i in range(best_i, len(path)-1):
            x1,y1,_ = path[i]
            x2,y2,_ = path[i+1]

            # start this segment at the projection for the first
            sx, sy = (last_x, last_y) if i == best_i else (x1, y1)
            seg_len = math.hypot(x2 - sx, y2 - sy)

            if seg_len >= remaining:
                frac = remaining / seg_len
                target = (sx + frac * (x2 - sx),
                          sy + frac * (y2 - sy))
                break

            remaining -= seg_len
            last_x, last_y = x2, y2
            car.path_index = i+1

        # if we ran out of path, aim at the final point
        if target is None:
            px, py, _ = path[-1]
            target = (px, py)
            car.path_index = len(path) - 1

    # 4) compute steering angle to that target
    dx, dy = target[0] - car.x, target[1] - car.y
    desired = math.degrees(math.atan2(dy, dx))
    delta = (desired - car.angle + 180) % 360 - 180
    turn = max(-car.rotation_speed, min(car.rotation_speed, delta))
    car.angle += turn

    # 5) simple speed control (optional)
    gx, gy, _ = path[-1]
    dist_goal = math.hypot(gx - car.x, gy - car.y)
    stopping = (car.speed**2) / (2*car.friction + 1e-5)
    if dist_goal < stopping:
        car.speed = max(car.speed - car.friction, 0)
    else:
        car.speed = min(car.speed + car.acceleration, car.max_speed)

    return target



################# RRT PLANNING ##########################

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


def is_in_collision(car: Car, state, obstacles):
    """
    Returns True if the car at [x, y, theta] would collide with any obstacle or go out of bounds.
    Otherwise, returns False.
    """
    px, py, theta = state

    # Check screen bounds first
    if not (0 <= px < WIDTH and 0 <= py < HEIGHT):
        return True

    # Create a temporary car surface
    car_surface = pygame.Surface((car.width + car.collider_buffer, car.height + car.collider_buffer), pygame.SRCALPHA)
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
            d = np.linalg.norm(s_query - n_i.state)
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
    def __init__(self, num_samples, obstacles, robot, num_dimensions=2, step_length = 10, lims = None,
                 connect_prob = 0.05, collision_func=None):
        '''
        Initialize an RRT planning instance
        '''
        self.K = num_samples
        self.n = num_dimensions
        self.epsilon = step_length
        self.connect_prob = connect_prob
        self.obstacles = obstacles
        self.robot = robot

        self.in_collision = collision_func
        if collision_func is None:
            self.in_collision = self.fake_in_collision

        # Setup range limits
        self.limits = lims
        if self.limits is None:
            self.limits = []
            for n in range(num_dimensions-1):
                self.limits.append([0,100])

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
                distance = np.linalg.norm(self.goal - node.state)
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


    def extend(self, T: RRTSearchTree, q):
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
        if self.in_collision(self.robot, new_state, self.obstacles):
            return (_TRAPPED, T)

        new_node = TreeNode(new_state)
        T.add_node(new_node, nearest_node)

        # Check collision
        if self.in_collision(self.robot, new_state, self.obstacles):
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