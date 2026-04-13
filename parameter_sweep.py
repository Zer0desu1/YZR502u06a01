import rospy
import dynamic_reconfigure.client
import json
import os
import time
from datetime import datetime
from navigation_experiment import NavigationExperiment, GOALS
SIM_TIME_VALUES = [1.0, 1.5, 2.0]
PATH_DISTANCE_BIAS_VALUES = [0.2, 0.5, 0.8]
NUM_TRIALS = 10

def set_dwa_params(client, sim_time, path_distance_bias):

    params = {
        'sim_time': sim_time,
        'path_distance_bias': path_distance_bias
    }
    try:
        config = client.update_configuration(params)
        rospy.loginfo(f"Parametreler guncellendi: sim_time={sim_time}, path_distance_bias={path_distance_bias}")
        return True
    except Exception as e:
        rospy.logwarn(f"Parametre guncelleme hatasi: {e}")
        return False

def main():
    rospy.init_node('parameter_sweep', anonymous=True)
    rospy.loginfo("DWA dynamic reconfigure client bekleniyor...")
    try:
        dwa_client = dynamic_reconfigure.client.Client(
            '/move_base/DWAPlannerROS', timeout=30
        )
    except Exception as e:
        rospy.logerr(f"DWA client olusturulamadi: {e}")
        return

    experiment = NavigationExperiment()

    results_dir = os.path.expanduser('~/catkin_ws/src/ros_navigation/results')
    os.makedirs(results_dir, exist_ok=True)

    all_sweep_results = []

    total_configs = len(SIM_TIME_VALUES) * len(PATH_DISTANCE_BIAS_VALUES)
    config_idx = 0

    for sim_time in SIM_TIME_VALUES:
        for pdb in PATH_DISTANCE_BIAS_VALUES:
            config_idx += 1
            config_name = f"dwa_st{sim_time}_pdb{pdb}"

            rospy.loginfo(f"\n{'='*60}")
            rospy.loginfo(f"Konfigurasyon {config_idx}/{total_configs}: {config_name}")
            rospy.loginfo(f"sim_time={sim_time}, path_distance_bias={pdb}")
            rospy.loginfo(f"{'='*60}")
            if not set_dwa_params(dwa_client, sim_time, pdb):
                continue
            rospy.sleep(1.0)
            results = experiment.run_experiment(
                goals=GOALS,
                num_trials=NUM_TRIALS,
                experiment_name=config_name
            )
            for r in results:
                r['sim_time'] = sim_time
                r['path_distance_bias'] = pdb

            all_sweep_results.extend(results)
            output = {
                'experiment_info': {
                    'date': datetime.now().isoformat(),
                    'config': config_name,
                    'sim_time': sim_time,
                    'path_distance_bias': pdb
                },
                'results': results
            }
            with open(os.path.join(results_dir, f"{config_name}_results.json"), 'w') as f:
                json.dump(output, f, indent=2)
    final_output = {
        'experiment_info': {
            'date': datetime.now().isoformat(),
            'type': 'parameter_sweep',
            'sim_time_values': SIM_TIME_VALUES,
            'path_distance_bias_values': PATH_DISTANCE_BIAS_VALUES,
            'num_trials': NUM_TRIALS,
            'total_experiments': len(all_sweep_results)
        },
        'results': all_sweep_results
    }

    final_path = os.path.join(results_dir, 'parameter_sweep_results.json')
    with open(final_path, 'w') as f:
        json.dump(final_output, f, indent=2, ensure_ascii=False)

    rospy.loginfo(f"\nTum parametre taramasi tamamlandi!")
    rospy.loginfo(f"Toplam deney: {len(all_sweep_results)}")
    rospy.loginfo(f"Sonuclar: {final_path}")

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
