# ece3091

## Setup process

The following was done on a fresh install of Raspberry Pi OS Lite (although I imagine full-version can be used with no issues)

```sh
#Using http://wiki.ros.org/action/fullsearch/ROSberryPi/Installing%20ROS%20Melodic%20on%20the%20Raspberry%20Pi?action=fullsearch&context=180&value=linkto%3A%22ROSberryPi%2FInstalling+ROS+Melodic+on+the+Raspberry+Pi%22

sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

#The second command did not work (to add keyserver) so did some research and found this 
#(https://discourse.ros.org/t/new-gpg-keys-deployed-for-packages-ros-org/9454) which suggested

sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

sudo apt-get update
sudo apt-get upgrade

sudo apt install -y python-rosdep python-rosinstall-generator python-wstool python-rosinstall build-essential cmake

sudo rosdep init
rosdep update

mkdir -p ~/ros_catkin_ws
cd ~/ros_catkin_ws

rosinstall_generator ros_comm --rosdistro melodic --deps --wet-only --tar > melodic-ros_comm-wet.rosinstall
wstool init src melodic-ros_comm-wet.rosinstall

cd ~/ros_catkin_ws
rosdep install -y --from-paths src --ignore-src --rosdistro melodic -r --os=debian:buster

#before doing make expand swapfile size (as I found the CMAKE command below failed without doing so)
#open the file in /etc/dphys-swapfile (you will need root permissions to modify) and change CONF_SWAPSIZE to
#CONF_SWAPSIZE=1024. then reboot to make sure changes take effect
sudo reboot now

sudo ./src/catkin/bin/catkin_make_isolated --install -DCMAKE_BUILD_TYPE=Release --install-space /opt/ros/melodic -j2

source /opt/ros/melodic/setup.bash
echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc

sudo apt-get install python-catkin-tools
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws
source /opt/ros/melodic/setup.bash
catkin build
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc

```
