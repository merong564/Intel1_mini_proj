# Intel1_mini_proj
ROKEY 부트캠프 6기 E1조의 지능1 미니 프로젝트 레포지토리입니다.


## 실행 가이드
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
- amr 카메라 실행
  ```
    ros2 run turtlebot4_pkg perception --ros-args -r __ns:=/robot1 -r /tf:=/robot1/tf -r /tf_static:=/robot1/tf_static
  ```
- amr 제어
  ```
    ros2 run turtlebot4_pkg move --ros-args -r __ns:=/robot1
  ```
