#Installs and compile all the used in the ROVI2 course.
FROM stefanrvo/robworkstudio:latest

USER root

#Use bash..
RUN ln -s -f  /bin/bash /bin/sh

#Add ROVI2 code
ADD ./ ROVI2
RUN chown rw_user:rw_user ROVI2 -R
USER rw_user



RUN source /opt/ros/kinetic/setup.sh && cd ROVI2/code && catkin_make


#Compile it
RUN source ROVI2/code/devel/setup.sh && cd ROVI2/code/ && catkin_make


#Add file for ip settings
ADD Docker/run.sh run.sh
USER root
RUN chmod +x run.sh
RUN chown rw_user:rw_user run.sh
USER rw_user
ENTRYPOINT [ "/bin/bash" ]
CMD [ "/home/rw_user/run.sh" ]

