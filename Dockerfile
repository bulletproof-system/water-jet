FROM ros:melodic-ros-core-bionic

SHELL ["/bin/bash", "-c"]

COPY docker/assets/sources.list /etc/apt/sources.list

RUN <<EOF
apt-get update
apt-get install -y curl
useradd -m -s /bin/bash wj
EOF
	
USER wj
	
RUN curl -fsSL https://get.pnpm.io/install.sh | ENV="$HOME/.bashrc" SHELL="$(which bash)" bash - &&\
	echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
