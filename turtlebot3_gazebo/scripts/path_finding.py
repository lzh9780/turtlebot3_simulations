import os
import rospkg
import numpy as np
import matplotlib.pyplot as plt
from PIL import Image
from Astar import AStar
import plotting
import math

def load_cost_map():
    rospack = rospkg.RosPack()
    map_path = os.path.join(rospack.get_path('turtlebot3_gazebo'), "scripts", "map.png")
    image = Image.open(map_path).convert("L")
    bw = image.point(lambda x: 0 if x<128 else 255, '1')
    arr = np.asarray(bw)
    arr = np.delete(arr, [4000, 4001], axis=0)
    arr = np.delete(arr, [4000, 4001], axis=1)
    arr = arr.reshape(200, 20, 200, 20).mean(axis=(1, 3))
    arr = (arr > 0.7).astype(int)
    
    # plt.imshow(arr, cmap='gray', vmin=0, vmax=1)
    # plt.show()
    
    cost = arr.copy()
    for i in range(len(arr)):
        for j in range(len(arr[i])):
            if arr[i][j] == 1:
                pass
            else:
                for n in range(-15, 15):
                    for m in range(-15, 15):
                        if i + n < 0 or i + n >= 200 or j + m < 0 or j + m >= 200:
                            pass
                        elif n ** 2 + m ** 2 > 15 ** 2: 
                            pass
                        else:
                            cost[i+n][j+m] = 0
    
    # cost = cost.reshape(100, 2, 100, 2).mean(axis=(1, 3))
    # cost = (cost > 0.7).astype(int)
    # plt.imshow(cost, cmap='gray', vmin=0, vmax=1)
    # plt.show()
    
    return cost

def add_robot(cost_map, x, y):
    pos_x, pos_y = coord_trans(x, y)
    map = cost_map.copy()
    
    for n in range(-25, 25):
        for m in range(-25, 25):
            if pos_x + n < 0 or pos_x + n >= 200 or pos_y + m < 0 or pos_y + m >= 200:
                pass
            elif n ** 2 + m ** 2 > 25 ** 2: 
                pass
            else:
                map[pos_x+n][pos_y+m] = 0
    
    return map

def get_obs(cost_map, multi_robot, robot_x, robot_y):
    if multi_robot:
        map = add_robot(cost_map, robot_x, robot_y)
    else: 
        map = cost_map
    
    obs = set()
    for i in range(len(map)):
        for j in range(len(map[i])):
            if map[i][j] == 1:
                pass
            else:
                obs.add((j, 200 - i))
    
    return obs

def coord_trans(x, y):
        return (round((y + 2) * 50), round((x + 2) * 50))

def path_generate(cost_map, s_start, s_goal, multi_robot, robot2_x, robot2_y):
    # plot = plotting.Plotting(s_start, s_goal)
    # plot.obs = obs
    if s_start == s_goal:
        return []
    
    astar = AStar(s_start, s_goal, "euclidean")
    astar.obs = get_obs(cost_map, multi_robot, robot2_x, robot2_y)
    try:
        path, visited = astar.searching()
    except KeyError:
        return []
    
    # print(path)
    # plot.animation(path, visited, "A*")
    
    path = path[::-1]
    action = [] # store the distance and yaw
    direction = [0, 0]
    start_pont = [path[0][0], path[0][1]]
    for c in range(len(path)-1):
        new_direction = [path[c+1][0] - path[c][0], path[c+1][1] - path[c][1]]
        if new_direction == direction:
            pass
        else: 
            if c != 0:
                distance = math.sqrt((path[c][0] - start_pont[0]) ** 2 + (path[c][1] - start_pont[1]) ** 2)
                yaw = math.atan2(path[c][0] - start_pont[0], path[c][1] - start_pont[1])
                action.append((round(distance / 50, 2), round(yaw, 2)))
            direction = new_direction
            start_pont = path[c]
        
    # print(action)
    # plot.animation(path, visited, "A*")
    
    # while len(action) != 0:
    #     a = action.pop(0)
    #     print(a)
    
    return action


if __name__ == '__main__':
    start = coord_trans(-1.6, 0)
    goal = coord_trans(-1.5, -1.5)
    path = path_generate(start, goal)
    print(path)