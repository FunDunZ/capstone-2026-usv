#!/bin/bash
clear

ubuntu_version=`lsb_release -c | awk '{print $2}'`
distro="humble"
WORKSPACE_PATH=`dirname "$(dirname "$(dirname "$(realpath $0)")")"`

VENV_DIR="rosenv"
helpMenu(){
	echo "setup.sh arguments"
	echo "-i: Installs and configures ROS2"
	echo "-u: Updates ROS2 workspace and adds helpful commands"
	echo "-h: Prints help menu"
}

# Installs any linux dependencies
installDependencies(){
	sudo apt update
	packages="${WORKSPACE_PATH}/src/setup/$1"
	while read -r package; do
		if ! dpkg -s "$package" &>/dev/null; then
			echo "$package is not installed. Installing..."
			sudo apt install -y "$package"
		else
			echo "$package is already installed."
		fi
	done < $packages
}

# Verify required packages are installed
installRos(){
	installDependencies "ros_linux_packages.txt"

	# Verify if ros is already installed on the system
	rospackage=`dpkg -l | grep -o "ros-.*-desktop"`
	if [[ -n $rospackage ]]; then
		echo "Existing ROS Version: $rospackage detected. Exiting installer."
		echo "If you want to reinstall ROS, please remove the existing package first."
		exit 0
	fi

	# Verify settings for locale
	echo "Verifying locale settings"
	if [ `locale | grep -o "LANG=en_US.UTF-8" | wc -l` -eq 0 ]; then
		echo "Updating locale to UTF-8"
		sudo locale-gen en_US en_US.UTF-8
		sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
		export LANG=en_US.UTF-8
	else
		echo "Successfully validated locale settings"
	fi

	# Enable Ubuntu Universe repository, add gpg key, and update repo data
	sudo add-apt-repository universe
	echo "Ensuring latest ROS2 GPG is set"
	sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
	echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
	sudo apt update

	echo "Installing ROS Distro: $distro"
	sudo apt install ros-$distro-desktop
	echo "source /opt/ros/$distro/setup.bash" >> ~/.bashrc

	echo "Setup complete"
	echo "Open a new terminal and run the following command to verify installation:"
	echo "ros2 run demo_nodes_py listener"

}

# Updates workdir and verifies venv is setup properly
updateEnvironment(){

	#Installs any new dependencies not on system
	installDependencies "linux_packages.txt"

	if ! [ -d "$WORKSPACE_PATH/$VENV_DIR" ]; then
		echo "venv directory missing generating new virutal environment"
		cd $WORKSPACE_PATH
		python3 -m venv $VENV_DIR
		touch $WORKSPACE_PATH/$VENV_DIR/COLCON_IGNORE
	fi

	# Install required python packages in venv
	echo "Verifying required python packages"
	source $WORKSPACE_PATH/$VENV_DIR/bin/activate
	pip install -r $WORKSPACE_PATH/src/setup/requirements.txt


	if [ ! -d /usr/share/GeographicLib ] || \
		[ -z "$(find /usr/share/GeographicLib -mindepth 1 -print -quit)" ]; then
		echo "Installing GeographicLib datasets..."
		source /opt/ros/$distro/setup.bash
		sudo ros2 run mavros install_geographiclib_datasets.sh
	else
		echo "GeographicLib Dataset already installed"
	fi

	# Update WORKSPACE VARIABLES

	if [ `grep -o ROS_WS_PATH ~/.bashrc | wc -l` -eq 0 ]; then
		echo "export ROS_WS_PATH=${WORKSPACE_PATH}" >> ~/.bashrc
	else
		sed -i "s|^ROS_WS_PATH.*|ROS_WS_PATH=$WORKSPACE_PATH|" ~/.bashrc
	fi

	# Add utility commands
	if [ `grep -o source_ros ~/.bashrc | wc -l` -eq 0 ]; then
		echo "Adding alias source_ros to .bashrc"
		echo -e "\nalias source_ros=sourceROS" >> ~/.bashrc
		echo -e "\nfunction sourceROS(){
			source /opt/ros/$distro/setup.bash
			source $WORKSPACE_PATH/install/setup.bash
			export PYTHONPATH=$WORKSPACE_PATH/$VENV_DIR/lib/python3.10/site-packages/:\$PYTHONPATH
		}" >> ~/.bashrc
	fi

	if [ `grep -o clean_ros ~/.bashrc | wc -l` -eq 0 ]; then
		echo "Adding alias clean_ros to .bashrc"
		echo -e "\nalias clean_ros=\"rm -rf $WORKSPACE_PATH/install $WORKSPACE_PATH/build $WORKSPACE_PATH/log\"" >> ~/.bashrc
	fi

	if [ `grep -o build_ros ~/.bashrc | wc -l` -eq 0 ]; then
		echo "Adding alias build_ros to .bashrc"
		echo -e "\nalias build_ros=\"cd $WORKSPACE_PATH && colcon build\"" >> ~/.bashrc
	fi

	source ~/.bashrc
}

while getopts "iuh" option; do
	case $option in
		i) installRos;;
		u) updateEnvironment;;
		h) helpMenu;;
		\\?) echo "Invalid option: -$OPTARG" >&2 && exit 1 ;;
	esac
done
