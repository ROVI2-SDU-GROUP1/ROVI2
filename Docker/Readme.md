#Docker images for installing SDU-CAROS
Run with 
docker run -d \
    --env="DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --env="CONTAINER_IP=__ip__" \
    --env="UR_IP=__IP__" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" stefanrvo/rovi2
