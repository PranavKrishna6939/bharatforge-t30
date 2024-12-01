import numpy as np
import matplotlib.pyplot as plt
import matplotlib.colors as colors  
import time

# Load updated map from file
value_to_index = {-1: 0, 0: 1, 100: 2, 99: 2, 98: 2}
num_bots = 2
cmap = colors.ListedColormap(['gray', 'white', 'black'])

while True:
    updated_map = np.loadtxt('updated_map.txt')
    display_map = np.copy(updated_map)
    index_map = np.vectorize(value_to_index.get)(display_map)
    
    plt.clf()
    plt.imshow(index_map, cmap=cmap, norm=colors.NoNorm())
    
    bot_positions = np.argwhere((display_map >= 99 - num_bots) & (display_map <= 99))
    plt.scatter(bot_positions[:,1], bot_positions[:,0], marker='o', color='red')  # Adjust coordinates if needed
    
    plt.pause(1)