#Installs and compile all the used in the ROVI2 course.
FROM stefanrvo/robworkstudio:latest

USER root

#Use bash..
RUN ln -s -f  /bin/bash /bin/sh

#Add ROVI2 code
ADD ./ ROVI2
RUN 	chown rw_user:rw_user ROVI2 -R && \
	cp ROVI2/Docker/run.sh ./run.sh && \
	chmod +x ./run.sh
USER rw_user

#Compile it
RUN source /opt/ros/kinetic/setup.sh && cd ROVI2/code && catkin_make

ENTRYPOINT [ "/bin/bash" ]
CMD [ "/home/rw_user/run.sh" ]

