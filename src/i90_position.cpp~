///////////////////////////////////////////////////////////////////////////////
//Source for i90_position node to estimate i90 position based on encoders		 //																																			 //
//Node Structure creation																										 //
//Huseyin Emre Erdem 																												 //
//v1.2																					                             //	
//Changelog:
//-Visualization of estimation points using rviz package is added			
//19.08.2014 																																 //
///////////////////////////////////////////////////////////////////////////////

/*This node reads the encoder values from /drrobot_motor topic and estimates the
new position based on encoder values. The new position is published under the 
i90_pos topic. The type of the messages published is std_msgs::Float32MultiArray.
After the estimation is done, a signal published on i90_estimation_done
http://www.robotnav.com/position-estimation/
Since encoder signal directions are different, there are slight differences
After estimation of a new position, the position is sent to visualization node
*/

#include "ros/ros.h"
#include "pos.h"//position values(x,y,yaw)
#include "MotorInfoArray.h"//encoder readings
#include <math.h>
#include <std_msgs/UInt8.h>
#include <visualization_msgs/Marker.h>//for visualization
#include <cmath>

#define MAXENCODER 32768//Encoder maximum pulse value. Goes to 0 after this
#define MAXENCODERDIFF 500//Maximum difference in encoder values during rotation towards ir
#define PI 3.141593
#define ENC2DEG 0.23484375//Constant for conversion from encoder to angle (((2 * PI * 0.0835(wheel radius)) * 360) / ((PI * 0.32(track)) * 800(encoder pulses per cycle)))
#define ENC2DIS 0.000655809//Constant for conversion from encoder to distance (PI * 2 * 0.0835 (wheel radius) / 800 (encoder pulses per cycle))

/*Variables*/
int iPrevEnc[2]={0,0};//0:left wheel, 1:right wheel Previous encoder readings
int iCurEnc[2];	      //Current encoder readings for both wheels
int iEncoderDifference[2]; //Difference of Current encoder reading and previous encoder reading for both wheels
float fPrevPosX=0;// Previous x position of i90 
float fPrevPosY=0;// Previous y position of i90 
float fPrevYaw=45;// previous yaw angle of i90
int iPubFlag=0;//Flag to allow publishment of the last calculated position estimation
int iRotError;//Encoder values difference
int iRotAverage;//Average of encoder value changes during rotation
float fDistanceTravelled;//This variable stores the distance travelled by i90 from its previous position
float fTargetAngleYaw;//Target heading
int iTransError;  // This variable stores the difference in the wheel encoder differences reading which is error in the translational motion
int iTransAverage;//Average of encoder value changes during translation
bool bTurnDir;//0:CW, 1:CCW
float f = 0.00;//Delete
i90_position::pos currentPos;
i90_position::pos prevPos;

/*Prototypes*/
void recalculateTranslation(const std_msgs::UInt8 iTranslationFlag);// Calculates new position of i90 
void recalculateRotation(const std_msgs::UInt8 iRotationFlag);//Calculates new yaw angle for i90
void updateEncoder(const drrobot_I90_player::MotorInfoArray motorInfoArray);//Reads encoder readings of both wheels
void readTargetPos(const i90_position::pos posReceived);//Reads target yaw angle

/*Main function*/
int main(int argc, char **argv){
	ros::init(argc, argv, "i90_position");//Create node called "i90_position"
	ros::NodeHandle n;//Create nodehandler to modify features of the node
	ros::Publisher posPub = n.advertise<i90_position::pos>("i90_current_pos", 1);//Publishes current position of i90
	ros::Publisher markerPub = n.advertise<visualization_msgs::Marker>("visualization_marker", 10);
	ros::Subscriber rotationSub = n.subscribe("i90_rotation_done", 1, recalculateRotation);//Calculates the new angle when after a rotation is performed
	ros::Subscriber translationSub = n.subscribe("i90_translation_done", 1, recalculateTranslation);//Calculates the new position when a translation is performed
	ros::Subscriber encoderSub = n.subscribe("drrobot_motor", 1, updateEncoder);//Reads the encoder values and assigns to variables to be used in other functions
	ros::Subscriber targetSub = n.subscribe("i90_target_pos", 1, readTargetPos);//To read target positions
	ros::Duration d = ros::Duration(2,0);
	ros::Rate loop_rate(1);

	/*Set initial position*/
	iPrevEnc[0] = 0;
	iPrevEnc[1] = 0;
	prevPos.fXPos=0;
	prevPos.fYPos=0;
	prevPos.fYawAngle=45.00;

	/*Visualization*/
	visualization_msgs::Marker line_strip;//Visualization
	geometry_msgs::Point p;//Visualization

	while (ros::ok()){
		if(iPubFlag==1){//If both rotation and translation is done
			posPub.publish(currentPos);//Publish the new position			

			/*Visualization settings*/
			line_strip.header.frame_id = "/my_frame";
			line_strip.header.stamp = ros::Time::now();
			line_strip.ns = "i90_position";
			line_strip.action = visualization_msgs::Marker::ADD;
			line_strip.pose.orientation.w = 1.0;
			line_strip.id = 1;
			line_strip.type = visualization_msgs::Marker::LINE_STRIP;
			line_strip.scale.x = 0.1;
			line_strip.color.b = 1.0;
			line_strip.color.a = 1.0;
			p.x = currentPos.fXPos;
			p.y = currentPos.fYPos;
			p.z = 0.00;
	/*	p.x = rand()%20;
		p.y = rand()%20;
		p.z = 0;
		*/	line_strip.points.push_back(p);
			markerPub.publish(line_strip);

			ROS_INFO("Published position: %f\t%f\t%f", currentPos.fXPos, currentPos.fYPos, currentPos.fYawAngle);
		}
		iPubFlag=0;
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}

/*Calculates the new angle when after a rotation is performed*/
void recalculateRotation(const std_msgs::UInt8 iRotFlag){

	/*Calculate the differences in encoder readings*/
	ROS_INFO("Encoders: %u\t%u\t%u\t%u", iPrevEnc[0], iPrevEnc[1], iCurEnc[0], iCurEnc[1]);

	/*Calculate the changes in encoder values together with turning direction*/
	for(int i=0;i<2;i++){
		iEncoderDifference[i] = iCurEnc[i] - iPrevEnc[i];
		if(iEncoderDifference[i] < 0){
			iEncoderDifference[i] = abs(iEncoderDifference[i]);
			if(iEncoderDifference[i] > 500){
				iEncoderDifference[i] = MAXENCODER - iEncoderDifference[i];
				bTurnDir = 0;
			}
			else{
				bTurnDir = 1;
			}
		}
		else{
			if(iEncoderDifference[i] > 500){
				iEncoderDifference[i] = MAXENCODER - iEncoderDifference[i];
				bTurnDir = 1;
			}
			else{
				bTurnDir = 0;
			}
		}
	}
	ROS_INFO("Encoders: %d\t%d\t%d", iEncoderDifference[0], iEncoderDifference[1], bTurnDir);

	/*Calculate the angle change*/
	iRotAverage = round ((iEncoderDifference[0] + iEncoderDifference[1]) / 2);//Average change
	if(bTurnDir == 0){//CW
		currentPos.fYawAngle = prevPos.fYawAngle - (iRotAverage * ENC2DEG);//Compensates angular error. 0.235 is angular precision of wheel encoder in degrees.
		if(currentPos.fYawAngle < 0.00){
			currentPos.fYawAngle += 360.00;
		}
	}
	else{//CCW
		currentPos.fYawAngle = prevPos.fYawAngle + (iRotAverage * ENC2DEG);//Compensates angular error. 0.235 is angular precision of wheel encoder in degrees.
		if(currentPos.fYawAngle > 360.00){
			currentPos.fYawAngle -= 360.00;
		}
	}
	ROS_INFO("New Angle: %f", currentPos.fYawAngle);
/*
	/*Translation during rotation
	iRotError = iEncoderDifference[1] - iEncoderDifference[0];//Difference in wheels
	fDistanceTravelled = iRotError * 0.00066;//Compensates error
	currentPos.fXPos = prevPos.fXPos + (fDistanceTravelled * (cos(currentPos.fYawAngle * (PI / 180.00)))); // Compensates error in position
	currentPos.fXPos = prevPos.fXPos + (fDistanceTravelled * (sin(currentPos.fYawAngle * (PI / 180.00))));

	if(iRotError >= 0){   // Calculates new yaw using difference in the value of wheel with minimum difference
		currentPos.fYawAngle = currentPos.fYawAngle + (iEncoderDifference[0] * 0.235);
	}
	else{
		currentPos.fYawAngle = currentPos.fYawAngle + (iEncoderDifference[1] * 0.235);
	}
*/
	prevPos.fYawAngle=currentPos.fYawAngle;
}

/*Calculates the new position when a translation is performed*/
void recalculateTranslation(const std_msgs::UInt8 iTransFlag){

	/*Calculate difference in encoder values*/
	if(iCurEnc[0] < iPrevEnc[0]){//Left encoder value should have increased
		iEncoderDifference[0] = MAXENCODER + iCurEnc[0] - iPrevEnc[0];
	}
	else{
		iEncoderDifference[0] = iCurEnc[0] - iPrevEnc[0];
	}
	if(iCurEnc[1] > iPrevEnc[1]){//Right encoder value should have decreased
		iEncoderDifference[1] = MAXENCODER + iPrevEnc[1] - iCurEnc[1];
	}
	else{
		iEncoderDifference[1] = iPrevEnc[1] - iCurEnc[1];
	}

	/*Calculate the angle change during translation due to the motor problem
	if(iEncoderDifference[1] > iEncoderDifference[0]){//CCW turning during translation
		iTransError = iEncoderDifference[1] - iEncoderDifference[0]; // Calulates the difference in wheel encoders
		currentPos.fYawAngle += iTransError * ENC2DEG;//Compensates angular error. 0.235 is angular precision of wheel encoder in degrees.
		if(currentPos.fYawAngle > 360.00){
			currentPos.fYawAngle -= 360.00;
		}
	}
	else{//CW turning during translation
		iTransError = iEncoderDifference[0] - iEncoderDifference[1];
		currentPos.fYawAngle -= iTransError * ENC2DEG;//Compensates angular error. 0.235 is angular precision of wheel encoder in degrees.
		if(currentPos.fYawAngle < 0.00){
			currentPos.fYawAngle += 360.00;
		}
	}
*/
	/*Calculate the change in translation*/
	iTransAverage = round((iEncoderDifference[0] + iEncoderDifference[1]) / 2);//Average change in encoders
	fDistanceTravelled = iTransAverage * ENC2DIS;//Change in meters

	/*Calculate the new positions*/
	currentPos.fXPos = prevPos.fXPos + (fDistanceTravelled * (cos(currentPos.fYawAngle * (M_PI / 180))));// Updates position of i90
	currentPos.fYPos = prevPos.fYPos + (fDistanceTravelled * (sin(currentPos.fYawAngle * (M_PI / 180))));

	/*Set current values as previous to be used in the next loop*/
	prevPos.fYawAngle=currentPos.fYawAngle;
	prevPos.fXPos=currentPos.fXPos;
	prevPos.fYPos=currentPos.fYPos;
	iPrevEnc[0]=iCurEnc[0];
	iPrevEnc[1]=iCurEnc[1];

	iPubFlag=1;
}

/*Reads the encoder values and assigns to variables to be used in other functions*/
void updateEncoder(const drrobot_I90_player::MotorInfoArray motorInfoArray){
	iCurEnc[0] = motorInfoArray.motorInfos[0].encoder_pos;
	iCurEnc[1] = motorInfoArray.motorInfos[1].encoder_pos;
}

/*Reads target yaw angle*/
void readTargetPos(const i90_position::pos posReceived){
		fTargetAngleYaw = posReceived.fYawAngle;//Get target angle
}

