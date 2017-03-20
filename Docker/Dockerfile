#Installs and compile all the used in the ROVI2 course.
FROM stefanrvo/robworkstudio:latest

USER root
#install roslint
RUN apt-get -y update
RUN apt-get -y install ros-kinetic-roslint \
	ros-kinetic-cv-bridge \
	ros-kinetic-image-transport \
	ros-kinetic-image-geometry \
	ros-kinetic-desktop-full \
	ros-kinetic-openni2-launch \
	ros-kinetic-cv-bridge \
	nano

#Use bash..
RUN ln -s -f  /bin/bash /bin/sh
USER rw_user

#clone ROVI2 repo

RUN git clone https://github.com/Exchizz/ROVI2.git

RUN source /opt/ros/kinetic/setup.sh && cd ROVI2/code && catkin_make


#Compile it
RUN source ROVI2/code/devel/setup.sh && cd ROVI2/code/ && catkin_make


#Add file for ip settings
ADD run.sh run.sh
USER root
RUN chmod +x run.sh
RUN chown rw_user:rw_user run.sh
USER rw_user
ENTRYPOINT [ "/bin/bash" ]
CMD [ "/home/rw_user/run.sh" ]

