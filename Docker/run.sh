#Source the environment
source ROVI2/code/devel/setup.sh

#launch ros controller
#roslaunch openni2_launch openni2.launch	 &
#sleep a bit to be safe
# sleep 3
#Run demo program.
if [ $UR_IP = "192.168.100.1" ]; then
	sed -i "s/\"192.168.100.1\"/\"$CONTAINER_IP\"/" ROVI2/code/src/caros/hwcomponents/caros_universalrobot/launch/caros_universalrobot_param.xml
	sed -i "s/\"192.168.100.4\"/\"$UR_IP\"/"   ROVI2/code/src/caros/hwcomponents/caros_universalrobot/launch/caros_universalrobot_param.xml
else
	sed -i "s/\"192.168.100.4\"/\"$UR_IP\"/"   ROVI2/code/src/caros/hwcomponents/caros_universalrobot/launch/caros_universalrobot_param.xml
	sed -i "s/\"192.168.100.1\"/\"$CONTAINER_IP\"/" ROVI2/code/src/caros/hwcomponents/caros_universalrobot/launch/caros_universalrobot_param.xml
fi

#roslaunch caros_universalrobot simple_demo_using_move_ptp.test device_ip:=$UR_IP callback_ip:=$CONTAINER_IP
roslaunch rovi2_development start_hw_node.launch &
#Wait for the core to start
sleep 5

#run squaremover
rosrun rovisquaremover node

