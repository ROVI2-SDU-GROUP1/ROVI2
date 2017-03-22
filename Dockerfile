#Installs and compile all the used in the ROVI2 course.
FROM stefanrvo/robworkstudio:latest

SHELL ["/bin/bash", "-c"]
USER root
#Add ROVI2 code
ADD ./ ROVI2
RUN 	chown rw_user:rw_user ROVI2 -R && \
	cp ROVI2/Docker/run.sh ./run.sh && \
	chmod +x ./run.sh
USER rw_user

#Compile it
RUN source /opt/ros/kinetic/setup.sh && cd ROVI2/code && catkin_make

EXPOSE 33333

ENTRYPOINT [ "/bin/bash" ]
CMD [ "/home/rw_user/run.sh" ]

