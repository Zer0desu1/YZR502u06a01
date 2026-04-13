import rospy
import actionlib
import json
import time
import os
import numpy as np
from datetime import datetime

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from actionlib_msgs.msg import GoalStatus

class NavigationExperiment:

    def __init__(self):
        rospy.init_node('navigation_experiment', anonymous=True)
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo("move_base action sunucusu bekleniyor...")
        self.client.wait_for_server()
        rospy.loginfo("move_base baglantisi kuruldu.")
        self.odom_data = []
        self.global_plan = []
        self.local_plan = []
        self.is_navigating = False
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.global_plan_sub = rospy.Subscriber(
            '/move_base/NavfnROS/plan', Path, self.global_plan_callback)
        self.local_plan_sub = rospy.Subscriber(
            '/move_base/DWAPlannerROS/local_plan', Path, self.local_plan_callback)
        self.results = []

    def odom_callback(self, msg):

        if self.is_navigating:
            self.odom_data.append({
                'time': rospy.Time.now().to_sec(),
                'x': msg.pose.pose.position.x,
                'y': msg.pose.pose.position.y,
                'vx': msg.twist.twist.linear.x,
                'wz': msg.twist.twist.angular.z
            })

    def global_plan_callback(self, msg):

        self.global_plan = [
            (p.pose.position.x, p.pose.position.y) for p in msg.poses
        ]

    def local_plan_callback(self, msg):

        self.local_plan = [
            (p.pose.position.x, p.pose.position.y) for p in msg.poses
        ]

    def navigate_to_goal(self, x, y, theta=0.0, timeout=120.0):
        self.odom_data = []
        self.global_plan = []
        self.is_navigating = True
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        goal.target_pose.pose.orientation.z = np.sin(theta / 2)
        goal.target_pose.pose.orientation.w = np.cos(theta / 2)

        rospy.loginfo(f"Hedef gonderiliyor: ({x:.2f}, {y:.2f})")
        start_time = time.time()
        self.client.send_goal(goal)
        success = self.client.wait_for_result(rospy.Duration(timeout))
        end_time = time.time()

        self.is_navigating = False
        nav_time = end_time - start_time
        state = self.client.get_state()
        status = "SUCCESS" if state == GoalStatus.SUCCEEDED else "FAILED"

        metrics = {
            'goal_x': x,
            'goal_y': y,
            'status': status,
            'navigation_time': nav_time,
            'path_length': self._calculate_odom_path_length(),
            'tracking_error_rmse': self._calculate_tracking_error(),
            'global_plan_length': self._calculate_plan_length(self.global_plan),
            'num_odom_points': len(self.odom_data)
        }

        rospy.loginfo(f"Sonuc: {status}, Sure: {nav_time:.2f}s, Yol: {metrics['path_length']:.3f}m")
        return metrics

    def _calculate_odom_path_length(self):

        if len(self.odom_data) < 2:
            return 0.0

        total_length = 0.0
        for i in range(1, len(self.odom_data)):
            dx = self.odom_data[i]['x'] - self.odom_data[i-1]['x']
            dy = self.odom_data[i]['y'] - self.odom_data[i-1]['y']
            total_length += np.sqrt(dx**2 + dy**2)

        return total_length

    def _calculate_tracking_error(self):

        if not self.global_plan or not self.odom_data:
            return 0.0

        plan = np.array(self.global_plan)
        errors = []

        for odom in self.odom_data:
            robot_pos = np.array([odom['x'], odom['y']])
            distances = np.linalg.norm(plan - robot_pos, axis=1)
            min_dist = np.min(distances)
            errors.append(min_dist)

        if errors:
            return float(np.sqrt(np.mean(np.array(errors)**2)))
        return 0.0

    def _calculate_plan_length(self, plan):

        if len(plan) < 2:
            return 0.0
        plan = np.array(plan)
        diffs = np.diff(plan, axis=0)
        return float(np.sum(np.linalg.norm(diffs, axis=1)))

    def run_experiment(self, goals, num_trials=10, experiment_name="default"):

        rospy.loginfo(f"Deney basliyor: {experiment_name}")
        rospy.loginfo(f"Hedef sayisi: {len(goals)}, Tekrar: {num_trials}")

        all_results = []

        for goal_idx, (gx, gy, gtheta) in enumerate(goals):
            rospy.loginfo(f"\n=== Hedef {goal_idx+1}/{len(goals)}: ({gx}, {gy}) ===")

            for trial in range(num_trials):
                rospy.loginfo(f"--- Deneme {trial+1}/{num_trials} ---")

                metrics = self.navigate_to_goal(gx, gy, gtheta)
                metrics['experiment'] = experiment_name
                metrics['goal_index'] = goal_idx + 1
                metrics['trial'] = trial + 1
                all_results.append(metrics)
                rospy.sleep(2.0)

        self.results = all_results
        return all_results

    def save_results(self, output_path):

        output = {
            'experiment_info': {
                'date': datetime.now().isoformat(),
                'total_trials': len(self.results)
            },
            'results': self.results
        }

        with open(output_path, 'w') as f:
            json.dump(output, f, indent=2, ensure_ascii=False)
        rospy.loginfo(f"Sonuclar kaydedildi: {output_path}")
GOALS = [
    (1.0, 0.5, 0.0),
    (0.5, 1.5, 1.57),
    (-1.0, 1.0, 3.14),
    (-1.5, -0.5, -1.57),
    (0.0, -1.5, 0.0),
]

def main():

    experiment = NavigationExperiment()
    configs = [
        {
            'name': 'navfn_dwa_default',
            'description': 'NavFn (Dijkstra) + DWA (varsayilan)',
        },
        {
            'name': 'global_planner_astar_dwa',
            'description': 'GlobalPlanner (A*) + DWA',
        },
    ]

    results_dir = os.path.expanduser('~/catkin_ws/src/ros_navigation/results')
    os.makedirs(results_dir, exist_ok=True)

    for config in configs:
        rospy.loginfo(f"\n{'='*60}")
        rospy.loginfo(f"Konfigurasyon: {config['name']}")
        rospy.loginfo(f"Aciklama: {config['description']}")
        rospy.loginfo(f"{'='*60}")

        results = experiment.run_experiment(
            goals=GOALS,
            num_trials=10,
            experiment_name=config['name']
        )

        output_path = os.path.join(results_dir, f"{config['name']}_results.json")
        experiment.save_results(output_path)

    rospy.loginfo("Tum deneyler tamamlandi!")

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
