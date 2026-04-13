import numpy as np
import yaml
from PIL import Image
import os

def load_map(map_yaml_path):

    with open(map_yaml_path, 'r') as f:
        map_info = yaml.safe_load(f)

    map_dir = os.path.dirname(map_yaml_path)
    image_path = os.path.join(map_dir, map_info['image'])

    img = Image.open(image_path)
    map_array = np.array(img)
    threshold = map_info.get('occupied_thresh', 0.65) * 255
    free_thresh = map_info.get('free_thresh', 0.196) * 255

    grid = np.zeros_like(map_array, dtype=np.uint8)
    grid[map_array < free_thresh] = 1
    grid[map_array > threshold] = 0
    grid[(map_array >= free_thresh) & (map_array <= threshold)] = 1
    grid = np.flipud(grid)

    resolution = map_info['resolution']
    origin = map_info['origin']

    return grid, resolution, origin

def world_to_grid(x, y, resolution, origin):

    gx = int((x - origin[0]) / resolution)
    gy = int((y - origin[1]) / resolution)
    return gx, gy

def grid_to_world(gx, gy, resolution, origin):

    x = gx * resolution + origin[0]
    y = gy * resolution + origin[1]
    return x, y

def create_sample_map(width=200, height=200, num_obstacles=15, seed=42):

    np.random.seed(seed)
    grid = np.zeros((height, width), dtype=np.uint8)
    grid[0:2, :] = 1
    grid[-2:, :] = 1
    grid[:, 0:2] = 1
    grid[:, -2:] = 1
    obstacles = [
        (40, 40, 20, 20),
        (100, 30, 15, 30),
        (150, 60, 25, 15),
        (60, 100, 30, 10),
        (120, 110, 10, 25),
        (30, 150, 20, 15),
        (80, 70, 15, 15),
        (160, 140, 20, 20),
        (50, 50, 8, 40),
        (130, 70, 12, 18),
        (90, 150, 25, 10),
        (170, 100, 15, 25),
        (20, 80, 18, 12),
        (110, 160, 20, 15),
        (70, 130, 10, 20),
    ]

    for ox, oy, ow, oh in obstacles[:num_obstacles]:
        x1, y1 = max(0, oy), max(0, ox)
        x2, y2 = min(height, oy + oh), min(width, ox + ow)
        grid[x1:x2, y1:y2] = 1
    for _ in range(5):
        cx = np.random.randint(30, width - 30)
        cy = np.random.randint(30, height - 30)
        r = np.random.randint(5, 12)
        for i in range(max(0, cy - r), min(height, cy + r)):
            for j in range(max(0, cx - r), min(width, cx + r)):
                if (j - cx)**2 + (i - cy)**2 <= r**2:
                    grid[i, j] = 1

    resolution = 0.05
    origin = [0.0, 0.0, 0.0]

    return grid, resolution, origin

if __name__ == "__main__":
    grid, res, origin = create_sample_map()
    print(f"Harita boyutu: {grid.shape}")
    print(f"Cozunurluk: {res} m/piksel")
    print(f"Serbest alan orani: {np.sum(grid == 0) / grid.size:.2%}")
    print(f"Engel orani: {np.sum(grid == 1) / grid.size:.2%}")
