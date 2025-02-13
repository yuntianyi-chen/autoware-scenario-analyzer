# Autoware Scenario Record Violation Analyzer
Scenario Violation Analyzer for Autoware ROS2 (.db3) Records

### Directory Structure
```
|--PROJECT_ROOT
   |--data
      |--maps
      |--records
   |--src
      |--environment
      |--objectives
      |--tools
      |--config.py
      |--run_analyzer.py
  ```

## Supporting Violation Types
| Violation Type           | Non-strict Features                                 | Strict Features |
|--------------------------|-----------------------------------------------------|-----------------|
| Collision               | $\{p^{E}_{t},s^{E}_{t},h^{E}_{t},p^{O}_{t},s^{O}_{t},h^{O}_{t}\}$ | -               |
| Fast Acceleration       | $\{p^{E}_{t},s^{E}_{t},h^{E}_{t},duration,accel\}$  | -               |
| Hard Braking           | $\{p^{E}_{t},s^{E}_{t},h^{E}_{t},duration,decel\}$  | -               |
| Speeding               | $\{p^{E}_{t},s^{E}_{t},h^{E}_{t},duration\}$        | -               |
| Unsafe Lane-change     | $\{p^{E}_{t},s^{E}_{t},h^{E}_{t},duration\}$        | -               |
| Lane-change in Junction | $\{p^{E}_{t},s^{E}_{t},h^{E}_{t}\}$                 | $id_{junction}$ |
| Module Delay           | $\{p^{E}_{t},s^{E}_{t},h^{E}_{t},duration\}$        | $type_{module}$ |
| Module Malfunction     | $\{p^{E}_{t},s^{E}_{t},h^{E}_{t}\}$                 | $type_{module}$ |
| Vehicle Paralysis      | $\{p^{E}_{t},s^{E}_{t},h^{E}_{t}\}$                 | $type_{module}$ |



## Prerequisite

- Python == 3.10 (Tested)
- OS == Linux Ubuntu 20.04 (Tested)
- Autoware == release/v1.0 (Tested)
- ROS2
- shapely == 2.0.3
- lanelet2 == 1.2.1
- lanelet2_extension_python

### Prepare Steps

1. Please clone and build the `release/v1.0` of the Autoware using Docker `git checkout release/v1.0`.

2. Place your maps under the `data/maps` directory.

3. Create a python virtual environment using `python3 create -m venv ./venv && source ./venv/bin/activate`

4. Install dependencies using `pip install -r requirements.txt`.


## Build
   
Make sure Autoware is installed and built or Autoware dependencies `ROS2`, `lanelet2`, `lanelet2_extension_python` are manually installed to your system in order to use `xxx_msgs` and `lanelet2` python packages globally.

1. Enter the `/autoware` directory (`cd autoware`) and run the following command:
    ```
    docker pull ghcr.io/autowarefoundation/autoware-universe:humble-1.0-cuda-amd64
    ```

2. Run the following command to start the docker container:
    ```
   rocker --nvidia --x11 --user --privileged --volume $HOME/autoware --volume $HOME/autoware_map --volume $HOME/autoware-scenario-analyer -- ghcr.io/autowarefoundation/autoware-universe:humble-1.0-cuda-amd64
    ```
3. Create a folder named `src` under the `/autoware` directory and run the following commands:
```bash
    cd autoware
    mkdir src
    vcs import src < autoware.repos
    vcs import src < simulator.repos
    sudo apt update
    rosdep update
    rosdep install -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO
```

4. Run the following commands to build the Autoware:
```bash
    source /opt/ros/humble/setup.bash
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

[//]: # (## Environment Setup)

[//]: # (- Run within the docker container:)

[//]: # (```bash)

[//]: # (   sudo ip link set lo multicast on)

[//]: # (   source ~/.bashrc)

[//]: # (   source install/setup.bash)

[//]: # (```)
   



---

## Use the Autoware if already built

1. Run the following command to start the docker container:
```bash
   rocker --nvidia --x11 --user --network c0 --privileged --volume $HOME/autoware --volume $HOME/autoware_map --volume $HOME/autoware-scenario-analyer -- ghcr.io/autowarefoundation/autoware-universe:humble-1.0-cuda-amd64
```

2. Update and install dependencies (Within the docker container):
```bash
   cd autoware
   sudo apt update
   rosdep update
   rosdep install -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO
```

[//]: # (3. Environment Setup &#40;Within the docker container&#41;:)

[//]: # (```bash)

[//]: # (   sudo ip link set lo multicast on)

[//]: # (   source ~/.bashrc)

[//]: # (   source install/setup.bash)

[//]: # (```)

----

## Usage
- Configure the `map_name` and `scenario_record_path` in the `run_analyzer.py` file.

- Run outside the docker container:
```bash
python3 src/run_analyzer.py
```


## Paper Citation
```aiignore
@article{ChenHLHG24,
  author       = {Yuntianyi Chen and
                  Yuqi Huai and
                  Shilong Li and
                  Changnam Hong and
                  Joshua Garcia},
  title        = {Misconfiguration Software Testing for Failure Emergence in Autonomous
                  Driving Systems},
  journal      = {Proc. {ACM} Softw. Eng.},
  volume       = {1},
  number       = {{FSE}},
  pages        = {1913--1936},
  year         = {2024},
  url          = {https://doi.org/10.1145/3660792},
  doi          = {10.1145/3660792},
  timestamp    = {Fri, 02 Aug 2024 21:41:21 +0200},
  biburl       = {https://dblp.org/rec/journals/pacmse/ChenHLHG24.bib},
  bibsource    = {dblp computer science bibliography, https://dblp.org}
}
```