# ITUROV MATE 2022

## Requirements
- ROS Melodic [[github]](https://github.com/ros/ros) [[wiki]](http://wiki.ros.org/melodic/Installation/Ubuntu)
```
Ubuntu 18.04
```

- UUV Simulator [[github]](https://github.com/uuvsimulator/uuv_simulator.git) [[wiki]](https://uuvsimulator.github.io)
```sh
  sudo apt-get install ros-melodic-uuv-simulator
```

- MAVROS [[github]](https://github.com/mavlink/mavros.git) [[wiki]](https://ardupilot.org/dev/docs/ros.html)
```sh
  sudo apt-get install ros-kinetic-mavros ros-kinetic-mavros-extras
```
```bash
  wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
  chmod a+x install_geographiclib_datasets.sh
  ./install_geographiclib_datasets.sh
```

- Robot Localization [[github]](https://github.com/cra-ros-pkg/robot_localization.git) [[wiki]](http://wiki.ros.org/robot_localization)
```sh
  sudo apt-get install ros-melodic-robot-localization

```

- Dynamic Reconfigure [[github]](https://github.com/ros/dynamic_reconfigure.git) [[wiki]](http://wiki.ros.org/dynamic_reconfigure)
```sh
  sudo apt-get install ros-melodic-dynamic-reconfigure
```

- Video Stream OpenCV [[github]](https://github.com/ros-drivers/video_stream_opencv.git) [[wiki]](http://wiki.ros.org/video_stream_opencv)
```sh
  sudo apt-get install ros-melodic-video-stream-opencv

```
