image_name = eduart-simulation:1.0.0

download-sources:
	git clone --branch develop https://github.com/EduArt-Robotik/edu_robot.git
	git clone --branch main https://github.com/EduArt-Robotik/edu_robot_control.git
	git clone --branch develop git@github.com:EduArt-Robotik/edu_simulation.git

build-docker:
	docker build -t $(image_name) .

upload-docker:
	docker tag $(image_name) eduartrobotik/$(image_name)
	docker push eduartrobotik/$(image_name)

all: download-sources build-docker
	echo "build edu_simulation into a docker image"

clean:
	rm -rf edu_robot
	rm -rf edu_robot_control
	rm -rf edu_simulation
