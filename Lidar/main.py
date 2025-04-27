from Lidar import env, sensors

# from Simulator import Simulation

import pygame
import math

# import numpy as np
# import matplotlib.pyplot as plt
# from scipy.spatial import ConvexHull
# from sklearn.cluster import DBSCAN
#
# # import pip install opencv-pythoncv2
# import cv2

environment = env.buildEnvironment((600, 1200))

environment.originalMap = environment.map.copy()
laser=sensors.LaserSensor(200,environment.originalMap, uncertainty=(0.5,0.01))
environment.map.fill((0,0,0))
environment.infomap = environment.map.copy()

running = True

while running:
    sensorON=False

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
        if pygame.mouse.get_focused():
            sensorON=True
        # if event.type == pygame.KEYDOWN:
        #     if event.key == pygame.K_SPACE:

                # tuple_points = environment.pointCloud
                # points = np.array(tuple_points)
                #
                # clustering = DBSCAN(eps=5, min_samples=5).fit(points)
                # labels = clustering.labels_
                #
                # for label in set(labels):
                #     if label == -1: continue  # skip noise
                #     cluster = points[labels == label]
                #     if len(cluster) >= 3:
                #         hull = ConvexHull(cluster)
                #         for simplex in hull.simplices:
                #             plt.plot(cluster[simplex, 0], cluster[simplex, 1], 'k-')
                #
                # min_x, min_y = cluster.min(axis=0)
                # max_x, max_y = cluster.max(axis=0)
                # box = np.array([
                #     [min_x, min_y],
                #     [min_x, max_y],
                #     [max_x, max_y],
                #     [max_x, min_y],
                #     [min_x, min_y]
                # ])
                # plt.plot(box[:, 0], box[:, 1], 'r--')
                #
                # rect = cv2.minAreaRect(np.array(cluster, dtype=np.float32))
                # box = cv2.boxPoints(rect)
                # box = np.intp(box)
                # box = np.vstack([box, box[0]])  # close the loop
                # plt.plot(box[:, 0], box[:, 1], 'g--')
                #
                #
                # plt.scatter(points[:, 0], points[:, 1], c=labels, cmap='tab10', s=10)
                # plt.gca().set_aspect('equal')
                # plt.title("2D Environment Primitives from Point Cloud")
                # plt.show()
                # # Compute convex hull
                # hull = ConvexHull(points)
                #
                # # Plot
                # plt.plot(points[:, 0], points[:, 1], 'o')
                # for simplex in hull.simplices:
                #     plt.plot(points[simplex, 0], points[simplex, 1], 'k-')
                # plt.show()
        elif not pygame.mouse.get_focused():
            sensorON=False

    if sensorON:
        position = pygame.mouse.get_pos()
        laser.position=position
        sensor_data = laser.sense_obstacles()
        environment.dataStorage(sensor_data)
        environment.show_sensorData()

    environment.map.blit(environment.infomap, (0,0))

    pygame.display.update()





