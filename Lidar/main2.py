from Lidar import env, sensors, features
import random
import pygame

def random_color():
    levels = range(32, 256, 32)
    return tuple(random.choice(levels) for _ in range(3))

FeatureMAP = features.featureDetection()
environment = env.buildEnvironment((600,1200))
originalMap = environment.map.copy()
laser=sensors.LaserSensor(200,originalMap, uncertainty=(0.5,0.01))
environment.map.fill((255,255,255))
environment.infomap = environment.map.copy()
#??
environment.map.fill((0,0,0))

pygame.display.set_caption('Feature detection using lidar')

running = True
FEATURE_DETECTION = True
BREAK_POINT_IND = 0

all_line_segments = []
all_lines = []

while running:
    environment.infomap = originalMap.copy()
    FEATURE_DETECTION = True
    BREAK_POINT_IND = 0
    ENDPOINTS = [0,0]
    sensorON=False
    PREDICTED_POINTS_TODRAW = []
    environment.map.fill((0, 0, 0))

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
        if pygame.mouse.get_focused():
            sensorON=True

        elif not pygame.mouse.get_focused():
            sensorON=False

    for point in all_line_segments:
        environment.infomap.set_at((int(point[0][0]), int(point[0][1])), (0, 255, 0))
        pygame.draw.circle(environment.infomap, COLOR, (int(point[0][0]), int(point[0][1])), 2, 0)
    # all_lines.append(ENDPOINTS)
    for line in all_lines:
        pygame.draw.line(environment.infomap, (255, 0, 0), line[0], line[1], 2)

    if sensorON:
        position = pygame.mouse.get_pos()
        laser.position=position
        sensor_data = laser.sense_obstacles()

        FeatureMAP.laser_points_set(sensor_data)

        while BREAK_POINT_IND < (FeatureMAP.NP - FeatureMAP.PMIN):
            seedSeg = FeatureMAP.seed_segment_detection(laser.position,BREAK_POINT_IND)
            if seedSeg == False:

                break
            else:
                seedSegment = seedSeg[0]
                PREDICTED_POINTS_TODRAW = seedSeg[1]
                INDICES = seedSeg[2]
                results = FeatureMAP.seed_segment_growing(INDICES, BREAK_POINT_IND)
                if results == False:
                    BREAK_POINT_IND = INDICES[1]
                    continue
                else:
                    line_eq=results[1]
                    m,c = results[5]
                    line_seg=results[0]
                    OUTERMOST = results[2]
                    BREAK_POINT_IND = results[3]

                    ENDPOINTS[0] = FeatureMAP.projection_point2line(OUTERMOST[0], m, c)
                    ENDPOINTS[1] = FeatureMAP.projection_point2line(OUTERMOST[1], m, c)

                    COLOR = random_color()
                    all_line_segments.extend(line_seg)
                    for point in all_line_segments:
                        environment.infomap.set_at((int(point[0][0]), int(point[0][1])), (0,255,0))
                        pygame.draw.circle(environment.infomap,COLOR,(int(point[0][0]), int(point[0][1])), 2, 0)
                    all_lines.append(ENDPOINTS)
                    for line in all_lines:
                        pygame.draw.line(environment.infomap, (255,0,0), line[0], line[1], 2)

                    environment.dataStorage(sensor_data)
                    # environment.show_sensorData()

    environment.map.blit(environment.infomap, (0,0))

    pygame.display.update()

