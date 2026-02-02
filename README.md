# Intel1_mini_proj
ROKEY 부트캠프 6기 E1조의 지능1 미니 프로젝트 레포지토리입니다.


### 실행 가이드
#### PC 1
- webcam 실행
  ```
  cd ~ && ros2 run turtlebot4_pkg webcam_pub --ros-args -r __ns:=/robot1
  ```
  실행 후, best.pt 입력

  (/home 위치에 best.pt가 있어야 합니다. 없을 경우, Intel1_mini_proj/src/turtlebot4_pkg/turtlebot4_pkg/best.pt를 복사하세요.)

#### PC 2
- localization 실행
  ```
    ros2 launch turtlebot4_navigation localization.launch.py   namespace:=/robot1   map:=/home/rokey/rokey_ws/maps/map_moonjung.yaml
  ```
- rviz 실행
  ```
    ros2 launch turtlebot4_viz view_robot.launch.py namespace:=/robot1
  ```
- navigation 실행
  ```
    ros2 launch turtlebot4_navigation nav2.launch.py namespace:=/robot1
  ```
- amr 인지 실행
  ```
    ros2 run turtlebot4_pkg perception --ros-args -r __ns:=/robot1 -r /tf:=/robot1/tf -r /tf_static:=/robot1/tf_static
  ```
- amr 제어
  ```
    ros2 run turtlebot4_pkg move --ros-args -r __ns:=/robot1
  ```
