import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import json
import os

from map_loader import create_sample_map
from astar import AStar
from rrt import RRT

def plot_map_with_paths(grid, paths_dict, resolution, title="Yol Planlama Karsilastirmasi",
                        save_path=None):

    fig, ax = plt.subplots(1, 1, figsize=(10, 10))
    cmap = plt.cm.colors.ListedColormap(['white', 'black'])
    ax.imshow(grid, cmap=cmap, origin='lower')

    colors = {'A*': '#2196F3', 'RRT': '#F44336', 'RRT*': '#4CAF50', 'PRM': '#FF9800'}
    legend_handles = []

    for algo_name, path in paths_dict.items():
        if path is None:
            continue
        path_array = np.array(path)
        color = colors.get(algo_name, '#9C27B0')
        ax.plot(path_array[:, 1], path_array[:, 0], '-', color=color,
                linewidth=2, alpha=0.8, label=algo_name)
        ax.plot(path_array[0, 1], path_array[0, 0], 'o', color=color,
                markersize=10, markeredgecolor='black', markeredgewidth=1.5)
        ax.plot(path_array[-1, 1], path_array[-1, 0], 's', color=color,
                markersize=10, markeredgecolor='black', markeredgewidth=1.5)

        length_m = sum(
            np.sqrt((path[i][0]-path[i-1][0])**2 + (path[i][1]-path[i-1][1])**2)
            for i in range(1, len(path))
        ) * resolution
        legend_handles.append(
            mpatches.Patch(color=color, label=f'{algo_name} ({length_m:.2f} m)')
        )
    legend_handles.append(plt.Line2D([0], [0], marker='o', color='w',
                          markerfacecolor='gray', markersize=10, label='Baslangic'))
    legend_handles.append(plt.Line2D([0], [0], marker='s', color='w',
                          markerfacecolor='gray', markersize=10, label='Hedef'))

    ax.legend(handles=legend_handles, loc='upper left', fontsize=10)
    ax.set_title(title, fontsize=14, fontweight='bold')
    ax.set_xlabel('X (piksel)', fontsize=11)
    ax.set_ylabel('Y (piksel)', fontsize=11)
    ax.grid(True, alpha=0.3)

    plt.tight_layout()
    if save_path:
        plt.savefig(save_path, dpi=150, bbox_inches='tight')
        print(f"Grafik kaydedildi: {save_path}")
    plt.close()

def plot_rrt_tree(grid, tree_nodes, path, title="RRT Agac Yapisi", save_path=None):

    fig, ax = plt.subplots(1, 1, figsize=(10, 10))

    cmap = plt.cm.colors.ListedColormap(['white', 'black'])
    ax.imshow(grid, cmap=cmap, origin='lower')
    for node in tree_nodes:
        if node.parent is not None:
            ax.plot([node.y, node.parent.y], [node.x, node.parent.x],
                    '-', color='#90CAF9', linewidth=0.3, alpha=0.5)
    if path:
        path_array = np.array(path)
        ax.plot(path_array[:, 1], path_array[:, 0], '-', color='#F44336',
                linewidth=2.5, label='Bulunan Yol')
        ax.plot(path_array[0, 1], path_array[0, 0], 'go', markersize=12, label='Baslangic')
        ax.plot(path_array[-1, 1], path_array[-1, 0], 'rs', markersize=12, label='Hedef')

    ax.legend(fontsize=11)
    ax.set_title(title, fontsize=14, fontweight='bold')
    ax.set_xlabel('X (piksel)', fontsize=11)
    ax.set_ylabel('Y (piksel)', fontsize=11)

    plt.tight_layout()
    if save_path:
        plt.savefig(save_path, dpi=150, bbox_inches='tight')
        print(f"Grafik kaydedildi: {save_path}")
    plt.close()

def plot_comparison_bars(results_path, save_dir=None):

    with open(results_path, 'r') as f:
        data = json.load(f)

    astar = [r for r in data['astar_results'] if r['success']]
    rrt = [r for r in data['rrt_results'] if r['success']]

    if not astar or not rrt:
        print("Yeterli basarili sonuc yok!")
        return

    if save_dir is None:
        save_dir = os.path.dirname(results_path)
    fig, axes = plt.subplots(1, 3, figsize=(18, 5))
    routes = sorted(set(r['route'] for r in astar))
    astar_by_route = {route: [r['path_length_m'] for r in astar if r['route'] == route] for route in routes}
    rrt_by_route = {route: [r['path_length_m'] for r in rrt if r['route'] == route] for route in routes}

    x = np.arange(len(routes))
    width = 0.35

    astar_means = [np.mean(astar_by_route[r]) for r in routes]
    astar_stds = [np.std(astar_by_route[r]) for r in routes]
    rrt_means = [np.mean(rrt_by_route[r]) for r in routes]
    rrt_stds = [np.std(rrt_by_route[r]) for r in routes]

    axes[0].bar(x - width/2, astar_means, width, yerr=astar_stds,
                label='A*', color='#2196F3', capsize=4, alpha=0.85)
    axes[0].bar(x + width/2, rrt_means, width, yerr=rrt_stds,
                label='RRT', color='#F44336', capsize=4, alpha=0.85)
    axes[0].set_ylabel('Yol Uzunlugu (m)', fontsize=11)
    axes[0].set_title('Rotalara Gore Yol Uzunlugu', fontsize=12, fontweight='bold')
    axes[0].set_xticks(x)
    axes[0].set_xticklabels(routes, rotation=45)
    axes[0].legend()
    axes[0].grid(axis='y', alpha=0.3)
    astar_times_by_route = {route: [r['planning_time_ms'] for r in astar if r['route'] == route] for route in routes}
    rrt_times_by_route = {route: [r['planning_time_ms'] for r in rrt if r['route'] == route] for route in routes}

    astar_t_means = [np.mean(astar_times_by_route[r]) for r in routes]
    astar_t_stds = [np.std(astar_times_by_route[r]) for r in routes]
    rrt_t_means = [np.mean(rrt_times_by_route[r]) for r in routes]
    rrt_t_stds = [np.std(rrt_times_by_route[r]) for r in routes]

    axes[1].bar(x - width/2, astar_t_means, width, yerr=astar_t_stds,
                label='A*', color='#2196F3', capsize=4, alpha=0.85)
    axes[1].bar(x + width/2, rrt_t_means, width, yerr=rrt_t_stds,
                label='RRT', color='#F44336', capsize=4, alpha=0.85)
    axes[1].set_ylabel('Planlama Suresi (ms)', fontsize=11)
    axes[1].set_title('Rotalara Gore Planlama Suresi', fontsize=12, fontweight='bold')
    axes[1].set_xticks(x)
    axes[1].set_xticklabels(routes, rotation=45)
    axes[1].legend()
    axes[1].grid(axis='y', alpha=0.3)
    all_astar_lengths = [r['path_length_m'] for r in astar]
    all_rrt_lengths = [r['path_length_m'] for r in rrt]

    bp = axes[2].boxplot([all_astar_lengths, all_rrt_lengths],
                         labels=['A*', 'RRT'], patch_artist=True,
                         boxprops=dict(alpha=0.85))
    bp['boxes'][0].set_facecolor('#2196F3')
    bp['boxes'][1].set_facecolor('#F44336')
    axes[2].set_ylabel('Yol Uzunlugu (m)', fontsize=11)
    axes[2].set_title('Yol Uzunlugu Dagilimi (Tum Rotalar)', fontsize=12, fontweight='bold')
    axes[2].grid(axis='y', alpha=0.3)

    plt.tight_layout()
    save_path = os.path.join(save_dir, "comparison_bars.png")
    plt.savefig(save_path, dpi=150, bbox_inches='tight')
    print(f"Karsilastirma grafigi kaydedildi: {save_path}")
    plt.close()
    fig, ax = plt.subplots(figsize=(8, 5))
    all_astar_times = [r['planning_time_ms'] for r in astar]
    all_rrt_times = [r['planning_time_ms'] for r in rrt]

    bp = ax.boxplot([all_astar_times, all_rrt_times],
                    labels=['A*', 'RRT'], patch_artist=True)
    bp['boxes'][0].set_facecolor('#2196F3')
    bp['boxes'][1].set_facecolor('#F44336')
    ax.set_ylabel('Planlama Suresi (ms)', fontsize=11)
    ax.set_title('Planlama Suresi Dagilimi', fontsize=12, fontweight='bold')
    ax.grid(axis='y', alpha=0.3)

    plt.tight_layout()
    save_path = os.path.join(save_dir, "planning_time_boxplot.png")
    plt.savefig(save_path, dpi=150, bbox_inches='tight')
    print(f"Planlama suresi kutu grafigi kaydedildi: {save_path}")
    plt.close()

def generate_all_plots(results_dir=None):

    if results_dir is None:
        results_dir = os.path.join(os.path.dirname(os.path.dirname(__file__)), "results")
    os.makedirs(results_dir, exist_ok=True)

    grid, resolution, origin = create_sample_map()
    print("A* yol planlaniyor...")
    astar = AStar(grid)
    astar_path, astar_metrics = astar.plan((10, 10), (180, 180))

    print("RRT yol planlaniyor...")
    np.random.seed(42)
    rrt = RRT(grid, step_size=5, max_iter=15000, goal_sample_rate=0.15)
    rrt_path, rrt_metrics = rrt.plan((10, 10), (180, 180))
    paths = {}
    if astar_path:
        paths['A*'] = astar_path
    if rrt_path:
        paths['RRT'] = rrt_path

    plot_map_with_paths(
        grid, paths, resolution,
        title="A* vs RRT Yol Karsilastirmasi (Rota 1)",
        save_path=os.path.join(results_dir, "path_comparison.png")
    )
    waypoints = [
        {"name": "Rota_1", "start": (10, 10), "goal": (180, 180)},
        {"name": "Rota_2", "start": (15, 180), "goal": (180, 15)},
        {"name": "Rota_3", "start": (100, 10), "goal": (100, 190)},
    ]

    fig, axes = plt.subplots(1, 3, figsize=(18, 6))
    cmap = plt.cm.colors.ListedColormap(['white', 'black'])

    for idx, wp in enumerate(waypoints):
        ax = axes[idx]
        ax.imshow(grid, cmap=cmap, origin='lower')

        astar_p, _ = astar.plan(wp['start'], wp['goal'])
        np.random.seed(42)
        rrt_planner = RRT(grid, step_size=5, max_iter=15000, goal_sample_rate=0.15)
        rrt_p, _ = rrt_planner.plan(wp['start'], wp['goal'])

        if astar_p:
            ap = np.array(astar_p)
            ax.plot(ap[:, 1], ap[:, 0], '-', color='#2196F3', linewidth=2, label='A*')
        if rrt_p:
            rp = np.array(rrt_p)
            ax.plot(rp[:, 1], rp[:, 0], '-', color='#F44336', linewidth=2, label='RRT')

        ax.plot(wp['start'][1], wp['start'][0], 'go', markersize=10)
        ax.plot(wp['goal'][1], wp['goal'][0], 'r^', markersize=10)
        ax.set_title(wp['name'], fontsize=12, fontweight='bold')
        ax.legend(fontsize=9)
        ax.grid(True, alpha=0.3)

    plt.suptitle('Farkli Rotalar Icin A* vs RRT Karsilastirmasi', fontsize=14, fontweight='bold')
    plt.tight_layout()
    plt.savefig(os.path.join(results_dir, "multi_route_comparison.png"), dpi=150, bbox_inches='tight')
    print(f"Coklu rota karsilastirmasi kaydedildi.")
    plt.close()
    results_file = os.path.join(results_dir, "offline_planning_results.json")
    if os.path.exists(results_file):
        plot_comparison_bars(results_file, results_dir)

    print("\nTum gorseller olusturuldu!")

if __name__ == "__main__":
    generate_all_plots()
