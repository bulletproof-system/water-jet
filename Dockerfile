FROM ros:melodic-ros-core-bionic

SHELL ["/bin/bash", "-c"]

COPY docker/assets/sources.list /etc/apt/sources.list

RUN <<EOF
apt-get update
apt-get install -y curl
useradd -m -s /bin/bash wj
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
EOF
	
	# source /opt/ros/melodic/setup.bash
