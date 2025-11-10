# 🏎️ MPC Bicycle Controller

ROS 2 package for following CSV waypoints with a discrete-time MPC bicycle model.  
Designed for **F1TENTH-style simulators** but adaptable to real vehicles.

---

## Features

- **CSV-driven reference**  
  Loads `timestep, x, y, yaw, velocity` or `x, y, v` CSVs recorded via `odom_to_csv_node`.  
  Coordinates are interpreted in the same frame as the odometry topic  
  (default `/ego_racecar/odom`, typically `map`).

- **Ackermann output**  
  Publishes `AckermannDriveStamped` on `/drive` (or any topic via parameter)   
  with steering saturation and acceleration limits.

- **Nominal MPC**  
  Finite-horizon LTI MPC with optional curvature feed-forward, per-step constraints,   
  and diagnostics (`trace_flow`, saturation logging).

- **Visualization utilities**  
  `csv_trajectory_visualizer` publishes `nav_msgs/Path` + RViz markers straight from the CSV.

- **TF helper**  
  `odom_tf_broadcaster` mirrors `/ego_racecar/odom` into `/tf` so RViz sees `map → ego_racecar/base_link`.

- **CSV toolkit**  
  - `odom_to_csv_node` for logging odometry to CSV  
  - `csv_downsampler` for reducing dense files to ≈ N waypoints

---

## 📁 Package layout
```text
mpc_bicycle/
├─ launch/                # launch files
│   └─ mpc_bicycle.launch.py
├─ config/
│   └─ mpc_params.yaml    # MPC parameters
├─ global_path/           # example CSVs
├─ src/
│   ├─ mpc_bicycle_node.cpp
│   ├─ odom_to_csv.cpp
│   ├─ csv_trajectory_visualizer.cpp
│   ├─ csv_downsampler.cpp
│   └─ odom_tf_broadcaster.cpp
└─ scripts/
    └─ downsample_csv.py
⚙️ Building

이 패키지는 ROS 2 워크스페이스(dodger_ws) 내부에서 빌드됩니다.
colcon 빌드 시스템을 사용하며, ROS 2 Foxy 이후의 표준 구조를 따릅니다.

1️⃣ 워크스페이스 이동

cd /dodger_ws


2️⃣ 빌드 (특정 패키지 선택 가능)

colcon build --packages-select mpc_bicycle


3️⃣ 환경 설정

source install/setup.bash


빌드가 완료되면 install/mpc_bicycle 폴더 아래에 실행 파일과 설정 파일이 자동 설치됩니다.

🚀 Launching

mpc_bicycle.launch.py 는 CSV 파일을 참조하여 차량이 경로를 추종하도록 설정합니다.

기본 실행 예시는 다음과 같습니다:

ros2 launch mpc_bicycle mpc_bicycle.launch.py \
  path_csv:=/dodger_ws/mpc_bicycle/global_path/centered_trajectory_100.csv \
  trace_flow:=false debug:=false


이 런치 파일은 아래 노드들을 자동으로 실행합니다:

mpc_bicycle_node : MPC 제어기를 실행하여 /drive 토픽으로 Ackermann 명령 퍼블리시

csv_trajectory_viz_node : CSV 궤적을 시각화 (nav_msgs/Path 및 RViz Marker)

odom_tf_broadcaster : /ego_racecar/odom → /tf 변환 브로드캐스트

옵션:

🔹 trace_flow:=true : 각 타임스텝별 계산 로그 출력

🔹 debug:=true : 디버깅용 상세 메시지 활성화

🧭 Coordinate Frames

이 패키지는 CSV 파일의 좌표계와 로봇의 odometry 좌표계가 일치해야 합니다.
기본 시뮬레이터에서는 /ego_racecar/odom 의 frame_id = map 이므로 CSV 좌표와 동일한 map 프레임을 사용합니다.

만약 실제 차량처럼 odom → base_link 구조를 쓰는 경우에는:

(a) map 프레임 기반 위치를 퍼블리시하는 토픽을 구독하거나

(b) mpc_bicycle_node 내부에서 CSV 좌표계와 odometry 좌표계를 변환하도록 수정해야 합니다.

🧩 RViz2 시각화

odom_tf_broadcaster 노드를 실행하면 map → ego_racecar/base_link 변환이 자동 생성됩니다.
RViz의 Fixed Frame을 map 으로 설정하면 경로와 차량이 올바르게 표시됩니다.

📝 Parameters

모든 파라미터는 config/mpc_params.yaml 에 정의되어 있습니다.
주요 항목은 아래와 같습니다:

Parameter	Description
path_csv	절대경로 CSV 파일 지정 (예: /dodger_ws/mpc_bicycle/global_path/centered_trajectory_100.csv)
wheelbase, dt, horizon	차량 모델 파라미터 (축간거리, 샘플링 시간, 예측 구간)
v_ref, use_csv_speed	기준 속도 또는 CSV의 속도값 사용 여부
delta_max_deg, a_max	조향각 및 가속도 제한
q_y, q_psi, q_v	상태 가중치 (위치/방향/속도)
r_kappa, r_a	입력 가중치 (조향률, 가속도)
use_curvature_ff, debug, trace_flow, cmd_topic	추가 옵션 및 디버그 설정

📄 참고:
path_csv 는 런치 명령 시 직접 override 가능하며, FindPackageShare() 로 패키지 내부 경로를 자동 참조하도록 설정할 수도 있습니다.

📈 CSV Utilities

이 패키지는 CSV 파일을 기반으로 경로를 읽거나 생성할 수 있는 다양한 툴을 제공합니다.

🧩 1) odom_to_csv_node — 실시간 경로 기록

Odometry 토픽(/ego_racecar/odom)을 CSV로 저장합니다.

ros2 run mpc_bicycle odom_to_csv_node \
  --ros-args -p odom_topic:=/ego_racecar/odom \
             -p output_path:=/tmp/path.csv


결과:

/tmp/path.csv
timestep, x, y, yaw, velocity

✂️ 2) csv_downsampler — 포인트 간격 줄이기

CSV 파일을 일정 개수의 포인트로 리샘플링합니다.

ros2 run mpc_bicycle csv_downsampler \
  --input /tmp/path.csv \
  --output /tmp/path_100.csv \
  --count 100


원래 CSV 포인트가 너무 많을 경우 MPC 계산 부하를 줄이는 데 유용합니다.

🐍 3) Python helper — 스크립트 버전

scripts/downsample_csv.py 스크립트는 CSV 헤더 유무에 따라 직접 리샘플링이 가능한 간단한 Python CLI 도구입니다.

python3 scripts/downsample_csv.py --input path.csv --output path_100.csv --has-header


이러한 툴들을 조합하면:

주행 데이터를 CSV로 기록 (odom_to_csv_node)

필요한 해상도로 줄임 (csv_downsampler)

mpc_bicycle.launch.py 에서 바로 사용

이 전체 과정으로 시뮬레이터 주행 → 데이터 수집 → 경로 생성 → MPC 추종 파이프라인을 완성할 수 있습니다.
