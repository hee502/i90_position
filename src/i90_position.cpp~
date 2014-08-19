///////////////////////////////////////////////////////////////////////////////
//Source for i90_position node to estimate i90 position based on encoders		 //																																			 //
//Node Structure creation																										 //
//Huseyin Emre Erdem 																												 //
//v1.1																					                             //	
//Stopped at recalculation for turning
//16.08.2014 																																 //
///////////////////////////////////////////////////////////////////////////////

/*This node reads the encoder values from /drrobot_motor topic and estimates the
new position based on encoder values. The new position is published under the 
i90_pos topic. The type of the messages published is std_msgs::Float32MultiArray.
After the estimation is done, a signal published on i90_estimation_done*/

#include "ros/ros.h"
#include "pos.h"//position values(x,y,yaw)
#include "MotorInfoArray.h"//encoder readings
#include <math.h>
#include <std_msgs/UInt8.h>

#define MAXENCODER 32768
#define MAXENCODERDIFF 500//Maximum difference in encoder values during rotation towards ir
#define PI 3.141593

/*Variables*/
int iPrevEnc[2]={0,0};//0:left wheel, 1:right wheel Previous encoder readings
int iCurEnc[2];	      //Current encoder readings for both wheels
int iEncoderDifference[2]; //Difference of Current encoder reading and previous encoder reading for both wheels
float fPrevPosX=0;// Previous x position of i90 
float fPrevPosY=0;// Previous y position of i90 
float fPrevYaw=45;// previous yaw angle of i90
int iPubFlag=0;//Flag to allow publishment of the last calculated position estimation
int iRotError;//Encoder values difference
float fDistanceTravelled;//This variable stores the distance travelled by i90 from its previous position
float fTargetAngleYaw;//Target heading
i90_position::pos currentPos;
i90_position::pos prevPos;

/*Prototypes*/
void recalculateTranslation(const std_msgs::UInt8 iTranslationFlag);// Calculates new position of i90 
void recalculateRotation(const std_msgs::UInt8 iRotationFlag);//Calculates new yaw angle for i90
//void recalculateTranslation(const i90_position::pos newPos);
//void recalculateRotation(const i90_position::pos newPos);
void updateEncoder(const drrobot_I90_player::MotorInfoArray motorInfoArray);//Reads encoder readings of both wheels
void readTargetPos(const i90_movement::pos posReceived);//Reads target yaw angle

/*Main function*/
int main(int argc, char **argv){
	ros::init(argc, argv, "i90_position");//Create node called "i90_position"
	ros::NodeHandle n;//Create nodehandler to modify features of the node
	ros::Publisher posPub = n.advertise<i90_position::pos>("i90_current_pos", 1);//Publishes current position of i90
	ros::Subscriber rotationSub = n.subscribe("i90_rotation_done", 1, recalculateRotation);//Calculates the new angle when after a rotation is performed
	ros::Subscriber translationSub = n.subscribe("i90_translation_done", 1, recalculateTranslation);//Calculates the new position when a translation is performed
	ros::Subscriber encoderSub = n.subscribe("drrobot_motor", 1, updateEncoder);//Reads the encoder values and assigns to variables to be used in other functions
	ros::Subscriber targetSub = n.subscribe("i90_target_pos", 1, readTargetPos);//To read target positions
	ros::Duration d = ros::Duration(2,0);
	ros::Rate loop_rate(1);

	/*Set initial position*/
	prevPos.fXPos=0;
	prevPos.fYPos=0;
	prevPos.fYawAngle=45.00;

	while (ros::ok()){
		if(iPubFlag==1){//If both rotation and translation is done
			//posPub.publish(currentPos);//Publish the new position
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
	iEncoderDifference[0] = iCurEnc[0] - iPrevEnc[0];
	iEncoderDifference[1] = iCurEnc[1] - iPrevEnc[1];

	/*Calcultate the new heading based on the turning direction*/
	if(targetAngleYaw < prevPos.fYawAngle){//CW rotation, both encoders should have increased
		for(int i=0; i<2; i++){
			if(iCurEnc[i] < iPrevEnc[i]){
				iCurEnc[i] += MAXENCODER;
				iEncoderDifference[i] = iCurEnc[i] - iPrevEnc[i];
			}
			iEncoderDifference[i] = iPrevEnc[i] - iCurEnc[i];
		}		
	}
	else{//CCW rotation, both encoders should have decreased
		for(int i=0; i<2; i++){
			if(iCurEnc[i] > iPrevEnc[i]){
				iPrevEnc[i] += MAXENCODER;
				iEncoderDifference[i] = iPrevEnc[i] - iCurEnc[i];
			}
			iEncoderDifference[i] = iCurEnc[i] - iPrevEnc[i];
		}		
	}

	/*Set current values as previous to be used in the next loop*/
	iPrevEnc[0]=iCurEnc[0];
	iPrevEnc[1]=iCurEnc[1];

	/*Calculate the difference average*/
	iRotError = iEncoderDifference[1] - iEncoderDifference[0];//Calulates the error in wheel encoders
	currentPos.fYawAngle = prevPos.fYawAngle + (iRotError * 0.235);//Compensates angular error. 0.235 is angular precision of wheel encoder in degrees.
	
	fDistanceTravelled = iRotError * 0.00066;//Compensates compensates error
	currentPos.fXPos = prevPos.fXPos + (fDistanceTravelled * (cos(currentPos.fYawAngle * (PI / 180.00)))); // Compensates error in position
	currentPos.fXPos = prevPos.fXPos + (fDistanceTravelled * (sin(currentPos.fYawAngle * (PI / 180.00))));

	if(iRotError>=0)   // Calculates new yaw using difference in the value of wheel with minimum difference
		currentPos.fYawAngle = currentPos.fYawAngle + (iEncoderDifference[0] * 0.235);
	else
		currentPos.fYawAngle = currentPos.fYawAngle + (iEncoderDifference[1] * 0.235);

	prevPos.fXPos=currentPos.fXPos;
	prevPos.fYPos=currentPos.fYPos;
	prevPos.fYawAngle=currentPos.fYawAngle;
}

/*Calculates the new position when a translation is performed*/
void recalculateTranslation(const std_msgs::UInt8 iTransFlag){
	int iTransError;  // This variable stores the difference in the wheel encoder differences reading which is error in the translational motion
	float fDistanceTravelled; // This variable stores the distance travelled by i90 from its previous position
	iCurEnc[1]=32768-iCurEnc[1];
	iEncoderDifference[0]=iCurEnc[0]-iPrevEnc[0];  
	iEncoderDifference[1]=iCurEnc[1]-iPrevEnc[1];
	if(iEncoderDifference[0]<0)     // It ensures if encoder has finished loop or not. If yes then loop should be added to the difference
		iEncoderDifference[0]=32768+iEncoderDifference[0];
	if(iEncoderDifference[1]<0)
		iEncoderDifference[1]=32768+iEncoderDifference[1];

	iPrevEnc[0]=iCurEnc[0];    //Updating previous encoder values to current encoder readings to use for next translation
	iPrevEnc[1]=iCurEnc[1];

	iTransError=iEncoderDifference[1]-iEncoderDifference[0]; // Calulates the error in wheel encoders
	if(iTransError>=0)  // Calculates distance using difference in the value of wheel with minimum difference
		fDistanceTravelled=iEncoderDifference[0]*0.00066; // 0.00066 is precision of wheel encoder for linear movement
	else
		fDistanceTravelled=iEncoderDifference[1]*0.00066;
	
	currentPos.fYawAngle = prevPos.fYawAngle+(iTransError*0.235);  // Compensates error. 0.235 is angular precision of wheel encoder in degrees.
	currentPos.fXPos = prevPos.fXPos+(fDistanceTravelled*(cos(currentPos.fYawAngle*(3.1416/180)))); // Updates position of i90
	currentPos.fXPos = prevPos.fXPos+(fDistanceTravelled*(sin(currentPos.fYawAngle*(3.1416/180))));
	prevPos.fYawAngle=currentPos.fYawAngle;
	prevPos.fXPos=currentPos.fXPos;
	prevPos.fYPos=currentPos.fYPos;

	iPubFlag=1;
}

/*Reads the encoder values and assigns to variables to be used in other functions*/
void updateEncoder(const drrobot_I90_player::MotorInfoArray motorInfoArray){
	iCurEnc[0] = motorInfoArray.motorInfos[0].encoder_pos;
	iCurEnc[1] = motorInfoArray.motorInfos[1].encoder_pos;
}

/*Reads target yaw angle*/
void readTargetPos(const i90_movement::pos posReceived){
		fTargetAngleYaw = posReceived.fYawAngle;//Get target angle
}

