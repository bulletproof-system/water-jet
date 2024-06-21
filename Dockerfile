FROM ros:melodic-ros-base-bionic

SHELL ["/bin/bash", "-c"]

COPY docker/assets/sources.list /etc/apt/sources.list

RUN <<EOF
apt-get update
apt-get install -y curl cmake g++ build-essential libpcl-dev pcl-tools
useradd -m -s /bin/bash wj
EOF

# ROS package
RUN <<EOF 
apt-get install -y ros-melodic-roscpp
apt-get install -y ros-melodic-rospy
apt-get install -y ros-melodic-tf
apt-get install -y ros-melodic-joint-state-publisher-gui
apt-get install -y ros-melodic-joy*
apt-get install -y ros-melodic-hector-mapping
apt-get install -y ros-melodic-gmapping
apt-get install -y ros-melodic-navigation
apt-get install -y ros-melodic-cv-bridge
apt-get install -y ros-melodic-audio-common
apt-get install -y ros-melodic-controller-manager
apt-get install -y ros-melodic-rosbridge-suite
apt-get install -y ros-melodic-desktop-full
apt-get install -y ros-melodic-gazebo-ros-control
apt-get install -y ros-melodic-joint-state-controller
apt-get install -y ros-melodic-position-controllers
apt-get install -y ros-melodic-effort-controllers
EOF
	
USER wj

RUN <<EOF
curl -o- https://raw.githubusercontent.com/nvm-sh/nvm/v0.39.7/install.sh | bash
export NVM_DIR="$HOME/.nvm"
[ -s "$NVM_DIR/nvm.sh" ] && \. "$NVM_DIR/nvm.sh"  # This loads nvm
[ -s "$NVM_DIR/bash_completion" ] && \. "$NVM_DIR/bash_completion"  # This loads nvm bash_completion
nvm install 17
nvm alias default 17
corepack enable pnpm
pnpm config set registry https://registry.npmmirror.com 
EOF
	

