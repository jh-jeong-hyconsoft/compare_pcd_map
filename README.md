# pcd_compare

`pcd_compare`는 여러 PCD 맵을 하나의 기준 맵에 자동 정합하고, ROS 2 RViz에서 파일별로 다른 색상으로 겹쳐 보는 패키지입니다.

- ROS 2 Humble 지원
- `data/ref`의 PCD 한 개를 기준 맵으로 사용
- `data/eval`의 PCD 여러 개를 각각 기준 맵에 정합
- roll/pitch는 고정하고 `x`, `y`, `z`, `yaw`만 추정
- 기준 맵과 평가 맵을 파일명 및 개별 색상으로 RViz에 표시
- 원본 PCD는 수정하거나 덮어쓰지 않음

## 요구 환경

- Ubuntu 22.04
- ROS 2 Humble
- PCL
- `pcl_conversions`
- RViz2
- Python 3 및 PyYAML

ROS 2 의존성은 워크스페이스에서 `rosdep`으로 설치할 수 있습니다.

```bash
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
```

## 디렉터리 구조

```text
pcd_compare/
├── config/
│   └── config.yaml
├── data/
│   ├── ref/
│   │   └── reference.pcd       # 반드시 한 개
│   └── eval/
│       ├── result_a.pcd
│       ├── result_b.pcd
│       └── ...                 # 여러 개 가능
├── include/pcd_compare/
├── launch/
│   └── compare.launch.py
├── src/
│   └── pcd_map_publisher.cpp
├── CMakeLists.txt
└── package.xml
```

PCD 파일은 `x`, `y`, `z`, `intensity` 필드를 갖는 형식을 기준으로 합니다.

### PCD 배치 규칙

1. 기준으로 사용할 PCD를 `data/ref`에 한 개만 넣습니다.
2. 비교할 PCD를 `data/eval`에 넣습니다.
3. eval 파일은 파일명 오름차순으로 정렬됩니다.
4. 정렬된 순서에 따라 RViz 색상과 `/pcd_compare/eval/map_N/points` 토픽이 결정됩니다.

`data/ref`에 PCD가 없거나 두 개 이상이면 launch가 실행을 중단합니다.

## 빌드

패키지를 ROS 2 워크스페이스의 `src` 아래에 배치합니다.

```bash
mkdir -p ~/pcd_compare_ws/src
cd ~/pcd_compare_ws/src
git clone <repository-url> pcd_compare

cd ~/pcd_compare_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-select pcd_compare
source install/setup.bash
```

현재 워크스페이스에서는 다음처럼 빌드할 수 있습니다.

```bash
cd /home/hycon_ubuntu/workspace/comparing_pcd_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select pcd_compare
source install/setup.bash
```

PCD 또는 설정 파일을 변경한 뒤에는 다시 빌드하는 것을 권장합니다.

## 실행

자동 정합 후 RViz까지 실행합니다.

```bash
ros2 launch pcd_compare compare.launch.py
```

RViz 없이 정합 결과와 토픽만 확인하려면 다음 옵션을 사용합니다.

```bash
ros2 launch pcd_compare compare.launch.py use_rviz:=false
```

정합은 eval 파일마다 순차적으로 수행되므로 PCD 크기와 파일 개수에 따라 RViz 표시까지 시간이 걸릴 수 있습니다.

## 정합 방식

처리 순서는 다음과 같습니다.

1. ref 및 eval 디렉터리에서 PCD 파일 검색
2. 기준 맵을 VoxelGrid로 다운샘플링
3. 각 eval 맵을 다운샘플링
4. eval 맵을 source, ref 맵을 target으로 ICP 수행
5. ICP 변환을 `x`, `y`, `z`, `yaw`로 제한
6. 정합 전후 MSE 및 대응점 비율 검사
7. 검증을 통과한 변환만 원본 해상도의 eval 맵에 적용
8. 변환된 PointCloud2를 RViz용 토픽으로 발행

roll과 pitch는 추정하거나 적용하지 않습니다. z는 ICP 대응점의 높이 차이로 계산합니다.

ICP가 수렴하더라도 다음 조건을 만족하지 않으면 결과를 거부하고 해당 eval 맵을 원래 좌표로 표시합니다.

- 정합 후 MSE가 설정된 비율 이상 개선될 것
- 정합 후 대응점 비율이 최소 기준 이상일 것
- 정합 전보다 대응점 비율이 과도하게 감소하지 않을 것

이 패키지는 정합 결과를 새 PCD 파일로 저장하지 않습니다. 변환은 RViz 표시용 메시지에만 적용됩니다.

## 설정

모든 노드 및 시각화 설정은 [`config/config.yaml`](config/config.yaml)에서 관리합니다.

```yaml
pcd_map_publisher:
  ros__parameters:
    ref_directory: data/ref
    eval_directory: data/eval
    frame_id: map

    alignment:
      enabled: true
      initial_guess: identity
      voxel_leaf_size: 0.5
      max_correspondence_distance: 3.0
      max_iterations: 100
      transformation_epsilon: 1.0e-8
      fitness_epsilon: 1.0e-6
      min_inlier_ratio: 0.5
      max_inlier_ratio_drop: 0.02
      min_relative_mse_improvement: 0.01
```

### 정합 파라미터

| 파라미터 | 설명 |
|---|---|
| `alignment.enabled` | 자동 정합 활성화 여부 |
| `alignment.initial_guess` | ICP 초기값. `identity` 또는 `centroid` |
| `alignment.voxel_leaf_size` | 다운샘플링 voxel 크기(m). 클수록 빠르지만 세부 형상이 감소 |
| `alignment.max_correspondence_distance` | ICP 대응점으로 인정할 최대 거리(m) |
| `alignment.max_iterations` | ICP 최대 반복 횟수 |
| `alignment.transformation_epsilon` | 변환 변화량 기반 수렴 기준 |
| `alignment.fitness_epsilon` | fitness 변화량 기반 수렴 기준 |
| `alignment.min_inlier_ratio` | 정합 결과를 수락할 최소 대응점 비율 |
| `alignment.max_inlier_ratio_drop` | 정합 전 대비 허용할 최대 대응점 비율 감소량 |
| `alignment.min_relative_mse_improvement` | 정합 결과를 수락할 최소 상대 MSE 개선 비율 |

부분 맵처럼 ref와 eval의 관측 범위가 다르면 `centroid`가 잘못된 초기 위치를 만들 수 있습니다. 기본값인 `identity`는 두 맵이 이미 대략 같은 좌표계에 있는 경우에 적합합니다.

초기 위치 차이가 `max_correspondence_distance`보다 훨씬 크거나 yaw 차이가 크면 ICP가 잘못된 지역 최솟값으로 수렴할 수 있습니다. 이 경우 초기 위치를 먼저 대략 맞추거나 전역 정합 단계가 추가로 필요합니다.

### RViz 색상

```yaml
visualization:
  reference_color: "255; 255; 255"
  eval_colors:
    - "255; 64; 64"
    - "64; 255; 64"
    - "64; 160; 255"
  point_size: 2
  grid_cell_size: 5.0
```

- ref 맵은 기본적으로 흰색이며 eval보다 한 픽셀 크게 표시됩니다.
- eval 색상은 파일명 오름차순으로 `eval_colors`에 대응합니다.
- eval 파일이 설정된 색상 수보다 많으면 추가 색상을 자동 생성합니다.
- RViz Display 이름은 `[REF] filename.pcd`, `[EVAL] filename.pcd` 형식입니다.

launch는 현재 파일 목록을 기준으로 임시 RViz 설정을 생성하며 종료할 때 해당 파일을 삭제합니다.

## 토픽과 TF

| 토픽 | 설명 |
|---|---|
| `/pcd_compare/ref/points` | 기준 PCD |
| `/pcd_compare/eval/map_0/points` | 첫 번째 eval PCD |
| `/pcd_compare/eval/map_1/points` | 두 번째 eval PCD |
| `/pcd_compare/eval/map_N/points` | N번째 eval PCD |
| `/tf_static` | `world -> map` identity 정적 TF |

PointCloud2 토픽은 `Reliable`, `Transient Local`, depth 1 QoS를 사용하므로 RViz가 늦게 실행되어도 마지막 맵을 받을 수 있습니다.

## 로그 확인

정합이 수락되면 파일별 변환과 개선 정도가 출력됩니다.

```text
robot1.pcd ICP accepted: MSE 1.599832 -> 0.284099 m^2 (82.2% better)
robot1.pcd transform: x=-2.549483 y=-0.718623 z=-0.206432 yaw=-0.006650 rad
```

실행 중 토픽을 확인하려면 다음 명령을 사용할 수 있습니다.

```bash
ros2 topic list | grep pcd_compare
ros2 topic info /pcd_compare/ref/points --verbose
```

## 문제 해결

### RViz에 맵이 바로 보이지 않는 경우

eval 맵 정합이 끝날 때까지 기다립니다. 파일이 많거나 `voxel_leaf_size`가 작으면 시간이 오래 걸릴 수 있습니다. 속도가 중요하면 voxel 크기를 키웁니다.

```yaml
alignment:
  voxel_leaf_size: 1.0
```

### `Reference directory must contain exactly one PCD` 오류

`data/ref`에 `.pcd` 파일이 정확히 한 개 있는지 확인합니다.

### ICP 결과가 거부되는 경우

로그에서 MSE와 inlier 비율을 확인한 뒤 다음 항목을 검토합니다.

- ref와 eval이 실제로 겹치는 영역을 포함하는지
- 두 맵의 초기 좌표 차이가 너무 크지 않은지
- `max_correspondence_distance`가 너무 작은지
- `voxel_leaf_size`가 맵 구조에 비해 너무 큰지

단순히 수락 기준을 낮추기 전에 RViz에서 원본 위치와 데이터 범위를 먼저 확인하는 것이 좋습니다.
