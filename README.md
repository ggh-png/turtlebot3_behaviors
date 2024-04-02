# Turtlebot3 TTS && GoToPose Using FlexBE

---

![fir](./fig/1.png)

![sec](./fig/2.png)


## Bashrc Setting

---

```bash
alias cs="cd ~/ros2_ws/src"
alias cw="cd ~/ros2_ws/"

alias eb='nano ~/.bashrc'
alias sb='source ~/.bashrc'
alias gs='git status'
alias gp='git pull'  

alias humble="source /opt/ros/humble/setup.bash; echo \"ROS2 humble\""
alias ros2start="humble; source ~/ros2_ws/install/local_setup.bash; echo \"ros2 ws is activated.!!\""

# Gazebo 서버와 클라이언트 프로세스를 모두 종료하는 alias
alias killgazebo="killall -9 gazebo & killall -9 gzserver  & killall -9 gzclient"
alias ros_domain="export ROS_DOMAIN_ID=13"

alias cb="cd ~/ros2_ws && colcon build --symlink-install && source install/local_setup.bash"

export ROS_DISTRO=humble
export RCUTILS_COLORIZED_OUTPUT=1

# ==========================  Turtlebot3 setting =========================>
export GAZEBO_MODEL_PATH=${GAZEBO_MODEL_PATH}:~/ros2_ws/src/turtlebot3_sim>
export GAZEBO_WORLD_PATH=${GAZEBO_WORLD_PATH}:~/ros2_ws/src/turtlebot3_sim>
export TURTLEBOT3_MODEL=waffle
export ROS_DOMAIN_ID=30 #TURTLEBOT3
. /usr/share/gazebo/setup.sh
export WORKSPACE_ROOT=~/ros2_ws

```

## FlexBE Setting

---

### **개발환경 세팅 FlexBE**

> 다음 명령어를 통해 FlexBE를 설치해 줍니다.
> 

```bash
sudo apt-get install ros-humble-flexbe*
```

**FlexBE APP 설치**

> 설치가 완료 되었다면 이어서 FlexBE의 gui인 FlexBE APP도 설치해 줍니다.
> 

```bash
cd ~/ros2_ws/src
git clone -b humble https://github.com/FlexBE/flexbe_app.git
```

**의존성 설치**

> 다음으로 필요한 의존성들을 설치해 줍니다.
> 

```bash
sudo apt install python3-rosdep2
cd ~/ros2_ws
rosdep update
rosdep install --from-paths src --ignore-src
```

**빌드**

> 이어서 빌드를 진행합니다.
> 

```bash
cd ~/ros2_ws
cb
```

**바이너리 설치** 

> 마지막으로 **필요한 nwjs 바이너리 다운로드 즉,** FlexBE App을 실행하기 전에 필요한 nwjs 바이너리를 다운로드합니다.
> 

```bash
ros2 run flexbe_app nwjs_install
```

### Build

```bash
cd ~/ros2_ws
colcon build --symlink-install
```

## Turtlebot 3 Gazebo Simulator Install && Setting

---

### Dependent pkg install

```bash
# install gazebo
sudo apt install ros-humble-gazebo-*
# install Cartographer
sudo apt install ros-humble-cartographer
sudo apt install ros-humble-cartographer-ros
# install NAV2
sudo apt install ros-humble-navigation2
sudo apt install ros-humble-nav2-bringup
```

### TB3 pkg install

---

```bash
cd ~/ros2_ws/src 
git clone -b humble-devel https://github.com/ROBOTIS-GIT/DynamixelSDK.git
git clone -b humble-devel https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git
git clone -b humble-devel https://github.com/ROBOTIS-GIT/turtlebot3.git
git clone -b humble-devel https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
colcon build --symlink-install
source ~/.bashrc
```

## TTS Setting

---

### python pip Install

```bash
sudo apt install python3-pip 
```

### **TTS Install**

```bash
pip3 install gtts playsound
pip3 install pydub
```

### TB3  TTS  PKG Install

```bash
cd ~/ros2_ws/src
git clone https://github.com/ggh-png/turtlebot3_tts
```

### Build

```bash
cd ~/ros2_ws
colcon build --symlink-install
```

# TB FlexBE Launch

---

### TB3 Gazebo Sim Launch

---

```bash
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

### TB3 Nav2 Launch

```bash
ros2 launch turtlebot3_navigation2 navigation2.launch.py map:=$HOME/map.yaml
```

```bash
ros2 launch turtlebot3_navigation2 navigation2.launch.py
```

### TTS launch

```bash
ros2 run turtlebot3_tts tts_server 
```

### FlexBE Launch

```python
ros2 launch flexbe_app flexbe_full.launch.py 
```