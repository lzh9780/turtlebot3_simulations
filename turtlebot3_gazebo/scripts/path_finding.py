import os
import rospy
import rospkg
import numpy as np
import matplotlib.pyplot as plt
from PIL import Image
from Astar import AStar
import plotting
import time

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
    
    # cost = cost.reshape(40, 5, 40, 5).mean(axis=(1, 3))
    # cost = (cost > 0.7).astype(int)
    # plt.imshow(cost, cmap='gray', vmin=0, vmax=1)
    # plt.show()
    
    return cost

def get_obs():
    cost_map = load_cost_map()
    
    obs = set()
    for i in range(len(cost_map)):
        for j in range(len(cost_map[i])):
            if cost_map[i][j] == 1:
                pass
            else:
                obs.add((j, 200 - i))
    
    return obs

def main():
    obs = get_obs()
    
    def coord_trans(x, y):
        return (round((y + 2) * 50), round((x + 2) * 50))
    
    s_start = coord_trans(-1.6, 0)
    s_goal = coord_trans(1.5, 1.5)
    plot = plotting.Plotting(s_start, s_goal)
    plot.obs = obs
    
    astar = AStar(s_start, s_goal, "euclidean")
    astar.obs = obs
    
    path, visited = astar.searching()
    print(path)
    plot.animation(path, visited, "A*")


if __name__ == '__main__':
    main()