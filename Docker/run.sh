#Source the environment
source ROVI2/code/devel/setup.sh

#launch ros controller
#roslaunch openni2_launch openni2.launch	 &
#sleep a bit to be safe
# sleep 3
#Run demo program.
sed -i "s/192.168.100.4/$UR_IP/"   ROVI2/code/src/caros/hwcomponents/caros_universalrobot/launch/caros_universalrobot_param.xml
sed -i "s/192.168.100.1/$CONTAINER_IP/" ROVI2/code/src/caros/hwcomponents/caros_universalrobot/launch/caros_universalrobot_param.xml

#roslaunch caros_universalrobot simple_demo_using_move_ptp.test device_ip:=$UR_IP callback_ip:=$CONTAINER_IP
roslaunch caros_universalrobot simple_demo_using_move_ptp.test

