#ifndef ENTROPYDRIVE_H_
#define ENTROPYDRIVE_H_

#include "EntropySubsystemTemplate.h"

//class EntropyDrive : public RobotDrive, public EntropySubsystemTemplate {
class EntropyDrive : public EntropySubsystemTemplate {
	
private:

	enum DriveMode 
	{
		Rotate = 0, 
		Radius
	};

	RobotDrive * wpiDrive; 
	
	//The robot drive system motors
	CANJaguar *MotorDriveLeft1;   
	CANJaguar *MotorDriveLeft2;
	CANJaguar *MotorDriveRight1;
	CANJaguar *MotorDriveRight2;
	
	float absolutevalue(float x);
	bool range(float x, float y, float maxthreshold);
	float drive_table_limit(float x, float max, float min);
	void get_index(int &x_index, int &y_index, float moveValue, float rotateValue, DriveMode mode);	
	float EntropyDrive::Limit(float num);
	
	float right_scale(float rotateValue, float moveValue, DriveMode mode);
	float left_scale(float rotateValue, float moveValue, DriveMode mode);
			
public:
	
	EntropyDrive () { }; 
	
	bool Initialize ();	
	
	void Cleanup ();
	
	char * GetFeedback() { return NULL; }
	
	bool DriveRobot(float MoveValue, float RotateValue);
	bool DriveRobotTrig(float MoveValue, float RotateValue);
	
};


#endif


