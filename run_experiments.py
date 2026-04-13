import numpy as np
import json
import os
import sys
import time
from datetime import datetime

from map_loader import create_sample_map, load_map
from astar import AStar
from rrt import RRT, RRTStar
NUM_TRIALS = 10
RESOLUTION = 0.05
WAYPOINTS = [
    {"name": "Rota_1", "start": (10, 10), "goal": (180, 180)},
    {"name": "Rota_2", "start": (15, 180), "goal": (180, 15)},
    {"name": "Rota_3", "start": (100, 10), "goal": (100, 190)},
    {"name": "Rota_4", "start": (10, 100), "goal": (190, 100)},
    {"name": "Rota_5", "start": (20, 20), "goal": (150, 170)},
]

def run_astar_experiments(grid, resolution):

    print("\n" + "="*60)
    print("A* ALGORITAMASI DENEYLERI")
    print("="*60)

    planner = AStar(grid, allow_diagonal=True)
    all_results = []

    for wp in WAYPOINTS:
        print(f"\n--- {wp['name']}: {wp['start']} -> {wp['goal']} ---")
        wp_results = []

        for trial in range(NUM_TRIALS):
            path, metrics = planner.plan(wp['start'], wp['goal'])

            if path:
                result = {
                    "algorithm": "A*",
                    "route": wp['name'],
                    "trial": trial + 1,
                    "start": wp['start'],
                    "goal": wp['goal'],
                    "path_length_px": metrics['path_length'],
                    "path_length_m": metrics['path_length'] * resolution,
                    "planning_time_s": metrics['planning_time'],
                    "planning_time_ms": metrics['planning_time'] * 1000,
                    "nodes_expanded": metrics['nodes_expanded'],
                    "path_points": metrics['path_points'],
                    "success": True
                }
            else:
                result = {
                    "algorithm": "A*",
                    "route": wp['name'],
                    "trial": trial + 1,
                    "start": wp['start'],
                    "goal": wp['goal'],
                    "path_length_px": 0,
                    "path_length_m": 0,
                    "planning_time_s": metrics['planning_time'],
                    "planning_time_ms": metrics['planning_time'] * 1000,
                    "nodes_expanded": metrics['nodes_expanded'],
                    "path_points": 0,
                    "success": False
                }

            wp_results.append(result)
            all_results.append(result)
        successful = [r for r in wp_results if r['success']]
        if successful:
            avg_length = np.mean([r['path_length_m'] for r in successful])
            avg_time = np.mean([r['planning_time_ms'] for r in successful])
            print(f"  Basari: {len(successful)}/{NUM_TRIALS}")
            print(f"  Ort. yol uzunlugu: {avg_length:.3f} m")
            print(f"  Ort. planlama suresi: {avg_time:.3f} ms")

    return all_results

def run_rrt_experiments(grid, resolution):

    print("\n" + "="*60)
    print("RRT ALGORITMASI DENEYLERI")
    print("="*60)

    all_results = []

    for wp in WAYPOINTS:
        print(f"\n--- {wp['name']}: {wp['start']} -> {wp['goal']} ---")
        wp_results = []

        for trial in range(NUM_TRIALS):
            np.random.seed(trial * 100 + hash(wp['name']) % 1000)

            planner = RRT(grid, step_size=5, max_iter=15000, goal_sample_rate=0.15)
            path, metrics = planner.plan(wp['start'], wp['goal'])

            if path:
                result = {
                    "algorithm": "RRT",
                    "route": wp['name'],
                    "trial": trial + 1,
                    "start": wp['start'],
                    "goal": wp['goal'],
                    "path_length_px": metrics['path_length'],
                    "path_length_m": metrics['path_length'] * resolution,
                    "planning_time_s": metrics['planning_time'],
                    "planning_time_ms": metrics['planning_time'] * 1000,
                    "nodes_expanded": metrics['nodes_expanded'],
                    "path_points": metrics['path_points'],
                    "tree_size": metrics['tree_size'],
                    "iterations": metrics['iterations'],
                    "success": True
                }
            else:
                result = {
                    "algorithm": "RRT",
                    "route": wp['name'],
                    "trial": trial + 1,
                    "start": wp['start'],
                    "goal": wp['goal'],
                    "path_length_px": 0,
                    "path_length_m": 0,
                    "planning_time_s": metrics['planning_time'],
                    "planning_time_ms": metrics['planning_time'] * 1000,
                    "nodes_expanded": metrics['nodes_expanded'],
                    "path_points": 0,
                    "tree_size": metrics['tree_size'],
                    "iterations": metrics['iterations'],
                    "success": False
                }

            wp_results.append(result)
            all_results.append(result)

        successful = [r for r in wp_results if r['success']]
        if successful:
            avg_length = np.mean([r['path_length_m'] for r in successful])
            avg_time = np.mean([r['planning_time_ms'] for r in successful])
            std_length = np.std([r['path_length_m'] for r in successful])
            print(f"  Basari: {len(successful)}/{NUM_TRIALS}")
            print(f"  Ort. yol uzunlugu: {avg_length:.3f} +/- {std_length:.3f} m")
            print(f"  Ort. planlama suresi: {avg_time:.3f} ms")

    return all_results

def save_results(astar_results, rrt_results, output_dir):

    os.makedirs(output_dir, exist_ok=True)

    all_results = {
        "experiment_info": {
            "date": datetime.now().isoformat(),
            "num_trials": NUM_TRIALS,
            "resolution": RESOLUTION,
            "map_type": "sample_map_200x200",
            "waypoints": [{"name": w["name"], "start": w["start"], "goal": w["goal"]} for w in WAYPOINTS]
        },
        "astar_results": astar_results,
        "rrt_results": rrt_results
    }

    output_path = os.path.join(output_dir, "offline_planning_results.json")
    with open(output_path, 'w', encoding='utf-8') as f:
        json.dump(all_results, f, indent=2, ensure_ascii=False)
    print(f"\nSonuclar kaydedildi: {output_path}")

    return output_path

def print_summary(astar_results, rrt_results):

    print("\n" + "="*60)
    print("GENEL KARSILASTIRMA OZETI")
    print("="*60)

    astar_success = [r for r in astar_results if r['success']]
    rrt_success = [r for r in rrt_results if r['success']]

    print(f"\n{'Metrik':<30} {'A*':>15} {'RRT':>15}")
    print("-" * 60)

    if astar_success and rrt_success:
        astar_lengths = [r['path_length_m'] for r in astar_success]
        rrt_lengths = [r['path_length_m'] for r in rrt_success]
        print(f"{'Ort. Yol Uzunlugu (m)':<30} {np.mean(astar_lengths):>15.3f} {np.mean(rrt_lengths):>15.3f}")
        print(f"{'Std. Yol Uzunlugu (m)':<30} {np.std(astar_lengths):>15.3f} {np.std(rrt_lengths):>15.3f}")

        astar_times = [r['planning_time_ms'] for r in astar_success]
        rrt_times = [r['planning_time_ms'] for r in rrt_success]
        print(f"{'Ort. Planlama Suresi (ms)':<30} {np.mean(astar_times):>15.3f} {np.mean(rrt_times):>15.3f}")
        print(f"{'Std. Planlama Suresi (ms)':<30} {np.std(astar_times):>15.3f} {np.std(rrt_times):>15.3f}")

        print(f"{'Basari Orani':<30} {len(astar_success)/len(astar_results):>15.1%} {len(rrt_success)/len(rrt_results):>15.1%}")

if __name__ == "__main__":
    print("Offline Planlama Deneyleri Baslatiliyor...")
    print(f"Deneme sayisi: {NUM_TRIALS}")
    print(f"Rota sayisi: {len(WAYPOINTS)}")
    map_path = os.path.join(os.path.dirname(__file__), "my_map.yaml")
    if os.path.exists(map_path):
        print(f"SLAM'den kaydedilen gercek harita yukleniyor: {map_path}")
        grid, resolution, origin = load_map(map_path)
    else:
        print("Gercek harita (my_map.yaml) bulunamadi. Test haritasi olusturuluyor...")
        grid, resolution, origin = create_sample_map()
    print(f"Harita boyutu: {grid.shape}")
    print(f"Cozunurluk: {resolution} m/piksel")
    astar_results = run_astar_experiments(grid, resolution)
    rrt_results = run_rrt_experiments(grid, resolution)
    print_summary(astar_results, rrt_results)
    results_dir = os.path.join(os.path.dirname(os.path.dirname(__file__)), "results")
    save_results(astar_results, rrt_results, results_dir)

    print("\nDeneyler tamamlandi!")
