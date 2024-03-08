current_dir = $(shell pwd)

download-sources:
	git clone --branch develop https://github.com/EduArt-Robotik/edu_robot.git
	git clone --branch main https://github.com/EduArt-Robotik/edu_robot_control.git
	git clone --branch develop git@github.com:EduArt-Robotik/edu_simulation.git

build-docker:
	docker build -t eduart-simulation:0.1.0 .

all: download-sources build-docker
	echo "build edu_simulation into a docker image"

clean:
	rm -rf edu_robot
	rm -rf edu_robot_control
	rm -rf edu_simulation
