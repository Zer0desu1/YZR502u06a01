# YZR502u06a01 - Otonom Mobil Robotlarda Hareket Planlama ve Navigasyon Stratejilerinin Simulasyon Tabanli Karsilastirmali Analizi

## Proje Hakkinda

Bu proje, Gazebo simulasyon ortaminda TurtleBot3 (burger) robotu kullanarak A* ve RRT kuresel planlama algoritmalarinin DWA yerel planlayicisi ile kullanildiginda yol uzunlugu, planlama suresi ve takip hatasi acisindan performanslarini karsilastirmaktadir.

## Sistem Gereksinimleri

- Ubuntu 20.04 LTS
- ROS Noetic
- Gazebo 11
- Python 3.8+
- TurtleBot3 paketleri

## Kurulum

### 1. ROS Noetic Kurulumu

```bash
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt install curl
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo apt update
sudo apt install ros-noetic-desktop-full
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 2. TurtleBot3 Paketleri

```bash
sudo apt install ros-noetic-turtlebot3 ros-noetic-turtlebot3-simulations ros-noetic-turtlebot3-navigation ros-noetic-turtlebot3-slam
echo "export TURTLEBOT3_MODEL=burger" >> ~/.bashrc
source ~/.bashrc
```

### 3. Ek ROS Paketleri

```bash
sudo apt install ros-noetic-dwa-local-planner ros-noetic-teb-local-planner ros-noetic-global-planner ros-noetic-gmapping ros-noetic-amcl ros-noetic-map-server
```

### 4. Catkin Workspace

```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
# Bu repoyu klonlayin
git clone https://github.com/KULLANICI_ADI/YZR502u06a01.git
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

### 5. Python Bagimliklar

```bash
pip3 install -r requirements.txt
```

## Proje Yapisi

```
YZR502u06a01/
├── offline_planning/           # Offline planlama algoritmalari
│   ├── map_loader.py          # Harita yukleme ve islem
│   ├── astar.py               # A* algoritmasi
│   ├── rrt.py                 # RRT ve RRT* algoritmalari
│   ├── visualize.py           # Gorselestirme modulu
│   └── run_experiments.py     # Deney calistirici
├── ros_navigation/            # ROS navigasyon dosyalari
│   ├── launch/
│   │   ├── gazebo_world.launch         # Gazebo ortami
│   │   ├── slam_gmapping.launch        # SLAM harita olusturma
│   │   └── navigation_experiment.launch # Navigasyon deneyleri
│   └── param/
│       ├── costmap_common_params.yaml   # Costmap ortak parametreleri
│       ├── global_costmap_params.yaml   # Global costmap
│       ├── local_costmap_params.yaml    # Local costmap
│       ├── dwa_local_planner_params.yaml # DWA parametreleri
│       ├── teb_local_planner_params.yaml # TEB parametreleri
│       └── move_base_params.yaml        # move_base parametreleri
├── trajectory_smoothing/      # Yorunge puruzsuzlestirme
│   ├── smoothing.py           # Kubik spline, min jerk, elastik bant
│   ├── velocity_profiles.py   # Trapezoidal ve S-curve hiz profilleri
│   └── run_smoothing_experiments.py # Deney calistirici
├── scripts/                   # Yardimci betikler
│   ├── navigation_experiment.py # ROS navigasyon deneyleri
│   ├── parameter_sweep.py     # DWA parametre taramasi
│   └── record_bag.sh          # rosbag kayit
├── results/                   # Sonuclar
│   └── statistical_analysis.py # Istatistiksel analiz
├── requirements.txt
└── README.md
```

## Calistirma Adimlari

### Adim 1: Harita Olusturma (SLAM)

```bash
# Terminal 1: Gazebo ortamini baslat
roslaunch ros_navigation gazebo_world.launch

# Terminal 2: SLAM'i baslat
roslaunch ros_navigation slam_gmapping.launch

# Terminal 3: Robotu gezdirmek icin teleop
roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch

# Terminal 4: Haritayi kaydet (robot tum alani gezdikten sonra)
rosrun map_server map_saver -f ~/my_map
```

### Adim 2: Offline Planlama Deneyleri

```bash
cd offline_planning
python3 run_experiments.py
python3 visualize.py
```

### Adim 3: ROS Navigasyon Deneyleri

```bash
# Terminal 1: Gazebo
roslaunch ros_navigation gazebo_world.launch

# Terminal 2: Navigasyon (NavFn + DWA)
roslaunch ros_navigation navigation_experiment.launch \
    global_planner:=navfn/NavfnROS \
    local_planner:=dwa_local_planner/DWAPlannerROS

# Terminal 3: Deney betigi
cd scripts
python3 navigation_experiment.py

# Terminal 4 (opsiyonel): rosbag kaydi
bash scripts/record_bag.sh
```

#### A* Kuresel Planlayici ile:

```bash
roslaunch ros_navigation navigation_experiment.launch \
    global_planner:=global_planner/GlobalPlanner \
    local_planner:=dwa_local_planner/DWAPlannerROS
```

### Adim 4: Parametre Taramasi

```bash
# Navigasyon launch aktifken:
cd scripts
python3 parameter_sweep.py
```

### Adim 5: Yorunge Puruzsuzlestirme

```bash
cd trajectory_smoothing
python3 run_smoothing_experiments.py
```

### Adim 6: Istatistiksel Analiz

```bash
cd results
python3 statistical_analysis.py
```

## Arastirma Tasarimi

### Bagimsiz Degisken
- Kuresel planlayici turu: A* / NavFn (Dijkstra tabanlı)

### Bagimli Degiskenler
- Yol uzunlugu (metre)
- Planlama suresi (saniye)
- Takip hatasi RMSE (metre)
- Hareket puruzsuzlugu (jerk)

### Kontrol Degiskenleri
- Harita: turtlebot3_world
- Robot: TurtleBot3 burger
- Baslangic/hedef noktalari: sabit (5 farkli rota)
- Her rota 10 kez tekrarlanir

### Hipotezler
- **H0**: A* ve RRT algoritmalarinin yol uzunlugu ortalamalari arasinda anlamli fark yoktur.
- **H1**: A* algoritmasi, RRT'ye gore anlamli derecede kisa yol bulur.

### Istatistiksel Yontem
- Bagimsiz orneklem t-testi (alpha = 0.05)
- Tek yonlu ANOVA (rota bazinda)
- Cohen's d etki buyuklugu

