Run with 
docker run -d \
    --env="DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --env="CONTAINER_IP=__ip__" \
    --env="UR_IP=__IP__" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" -p 33333:33333 stefanrvo/rovi2



You need to have an ur controller running on the given ip address.
