**Note**:  
- For Windows users: It is recommended to have Ubuntu 24.04 installed on your system. You can also dual-boot with Ubuntu 24.04. If that's not possible, you can use WSL2.
- For Mac users: Please explore the options for installing and running a Docker container or an Ubuntu virtual machine.

## Installation Instructions

### On Windows
* Install Windows Subsystem For Linux (WSL2)
	* Open Command Prompt as administrator. Execute the following command. Restart the system if prompted
      ```
      wsl --set-default-version 2
      wsl --install -d Ubuntu-24.04
      ```
**Note**: If it gets stuck at `Enabling feature(s)` while installing wait ~20 minutes until it completely installs. If it is still not installed, press Ctrl+C. Now as the installation is incomplete, it needs to be uninstalled and installed again. For that, search for `Turn Windows features on or off` in control panel. Here, disable Windows Subsystem for Linux and restart. Then enable it and restart. Now follow the above commands to install it again.
- Install Docker Desktop [link](https://docs.docker.com/desktop/setup/install/windows-install/)
	- Note: If prompted, ensure the **Use WSL 2 instead of Hyper-V** option on the Configuration page for better support for linux (and for  many other reasons)
	- If there is some error after opening docker related to wsl, then the installation of wsl is incomplete. Follow the Note section from wsl installation instructions above

## Setting up WSL2
- Open wsl by executing `wsl` from cli. In the wsl terminal run the following commands to allow GUI from docker to be visible on the screen ([source](https://github.com/mviereck/x11docker#installation))
  ```
  curl -fsSL https://raw.githubusercontent.com/mviereck/x11docker/master/x11docker | sudo bash -s -- --update

  sudo apt install -y xinit xpra xserver-xephyr
  ```
- If you are using Windows machine and using WSL, then you can **directly install ROS and Gazebo instead of using the docker container**

## Installation of ROS and Gazebo
- If you are not using the docker image for ROS, then please install ROS by following this [tutorial](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html). Follow the steps in `System Setup` and `Install ROS2` and skip the last command for ros-base install

- If you are on docker or WSL, currently you have ROS setup. Now proceed with installing Gazebo
  ```
  sudo apt-get install curl

  sudo curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg

  echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null

  sudo apt-get update
  sudo apt-get install gz-harmonic
  ```

## Using docker container

- From the `docker/` folder, run the docker container. First pull the image and then execute it
  ```
  docker build -t tb4_image .

  docker run -it --rm --net=host --privileged -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix --name ros_jazzy_container tb4_image
  ```
- To have multiple terminals of docker container open, run the following command in another terminal
  ```
  docker exec -it ros_jazzy_container bash
  ```