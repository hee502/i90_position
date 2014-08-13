///////////////////////////////////////////////////////////////////////////////
//Source for i90_position node to estimate i90 position based on encoders		 //
//v1.0 																																			 //
//First creation 																														 //
//Huseyin Emre Erdem 																												 //
//08.08.2014 																																 //
///////////////////////////////////////////////////////////////////////////////

/*This node reads the encoder values from /drrobot_motor topic and estimates the
new position based on encoder values. The new position is published under the 
i90_pos topic. The type of the messages published is std_msgs::Float32MultiArray.*/

#include "ros/ros.h"
#include "pos.h"//position values(x,y,yaw)
#include "MotorInfoArray.h"//encoder readings

/*Prototypes*/
void recalculateTranslation(const i90_position::pos newPos);
void recalculateRotation(const i90_position::pos newPos);
void updateEncoder(const drrobot_I90_player::MotorInfoArray motorInfoArray);

/*Main function*/
int main(int argc, char **argv)
{
	ros::init(argc, argv, "i90_position");//Create node called "i90_movement"
	ros::NodeHandle n;//Create nodehandler to modify features of the node
	ros::Publisher posPub = n.advertise<i90_position::pos>("i90_current_pos", 1);
	ros::Subscriber translationSub = n.subscribe("i90_translation_done", 1, recalculateTranslation);
	ros::Subscriber rotationSub = n.subscribe("i90_rotation_done", 1, recalculateRotation);
	ros::Subscriber encoderSub = n.subscribe("drrobot_motor", 1, updateEncoder);

	ros::Duration d = ros::Duration(2,0);
	ros::Rate loop_rate(2);

	while (ros::ok())
	{
		i90_position::pos posValue;
		posValue.fXPos = 3.130;
		posValue.fYPos = 2.52;
		posValue.fYawAngle = 259.04;
		posPub.publish(posValue);
		// d.sleep();
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}

void recalculateTranslation(const i90_position::pos newPos){
	ROS_INFO("I heard: [%f]/t[%f]", newPos.fXPos, newPos.fYPos);
}

void recalculateRotation(const i90_position::pos newPos){
	ROS_INFO("I heard: [%f]", newPos.fYawAngle);
}

void updateEncoder(const drrobot_I90_player::MotorInfoArray motorInfoArray){
	ROS_INFO("I heard: [%u]/t[%u]", motorInfoArray.motorInfos[0].encoder_pos, motorInfoArray.motorInfos[1].encoder_pos);
}
