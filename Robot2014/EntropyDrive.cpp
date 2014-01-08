#include "IODefinitions.h"
#include "WPILib.h"
#include "EntropySubsystemTemplate.h"
#include "EntropyDrive.h"
#include "EntropyDriveTable.h"
#include <math.h>


    bool EntropyDrive::Initialize () { 
		MotorDriveLeft1 = new CANJaguar(IODefinitions::MOTOR_DRIVE_LEFT_1);
		MotorDriveLeft2 = new CANJaguar(IODefinitions::MOTOR_DRIVE_LEFT_2);
		MotorDriveRight1 = new CANJaguar(IODefinitions::MOTOR_DRIVE_RIGHT_1);
		MotorDriveRight2 = new CANJaguar(IODefinitions::MOTOR_DRIVE_RIGHT_2);
		
		wpiDrive = new RobotDrive( 	MotorDriveLeft1,
									MotorDriveLeft2,
									MotorDriveRight1,
									MotorDriveRight2 );
    	return true;
	}
		
	
	void EntropyDrive::Cleanup (){
		MotorDriveLeft1->Disable();
		MotorDriveLeft2->Disable();
		MotorDriveRight1->Disable();
		MotorDriveRight2->Disable();
    }
	
		
	bool EntropyDrive::DriveRobot(float MoveValue, float RotateValue){
	
		float LeftMotors = 0; 
		float RightMotors = 0; 
		
		MoveValue=Limit(MoveValue);
		RotateValue=Limit(RotateValue);
		
				
		LeftMotors = left_scale(RotateValue, MoveValue, Rotate);  //Scale Motor inpputs from drive table
		RightMotors = right_scale(RotateValue, MoveValue, Rotate);
				
		
		//Command motors
		wpiDrive->SetLeftRightMotorOutputs( LeftMotors, RightMotors );
		
		return true;
	}
	
		
	
	bool EntropyDrive::DriveRobotTrig(float MoveValue, float RotateValue){
		
			float LeftMotors = 0; 
			float RightMotors = 0; 
			float OutsideWheels = 0;
			float InsideWheels = 0;
			float Hypot = 0;
			float AbsMoveValue = 0;
			float AbsRotateValue = 0;
			
			float CompMoveValuePlus=0.99;
			float CompMoveValueMinus=0.60;
			float CompRotateValuePlus=0.99;
			float CompRotateValueMinus=0.99;
			
			//Normalize Joystick inputs
			if (MoveValue >= 0.0)
			{
				MoveValue=MoveValue/CompMoveValuePlus;
			}
			else
			{
				MoveValue=MoveValue/CompMoveValueMinus;
			}
			
			if (RotateValue>=0.0)
			{
				RotateValue=RotateValue/CompRotateValuePlus;
			}
			else
			{
				RotateValue=RotateValue/CompRotateValueMinus;
			}
			
			
			MoveValue=Limit(MoveValue);
			RotateValue=Limit(RotateValue);
			AbsMoveValue=absolutevalue(MoveValue);
			AbsRotateValue= absolutevalue(RotateValue);
			
			
			//Theta = atanf(AbsMoveValue/(AbsRotateValue+0.000001));
			//Theta = asinf(0.2);
			Hypot = sqrt(AbsMoveValue*AbsMoveValue+AbsRotateValue*AbsRotateValue);
			
			
			OutsideWheels = AbsMoveValue*(AbsMoveValue/Hypot);   
			InsideWheels = AbsMoveValue*( 1- (AbsRotateValue/Hypot));
			
			//Scale Motor inputs

			if (RotateValue<=0.0)
			{
				LeftMotors = InsideWheels * MoveValue/AbsMoveValue;
				RightMotors = OutsideWheels * MoveValue/AbsMoveValue;		
			}
			else
			{
				LeftMotors = OutsideWheels *  MoveValue/AbsMoveValue;
				RightMotors = InsideWheels * MoveValue/AbsMoveValue;
			}	
			
			
				
			//Command motors
			wpiDrive->SetLeftRightMotorOutputs( -1*LeftMotors,-1*RightMotors );
			
			return true;
		}


	/* search left drive table using binary search */
	/* Input:   x_value  (rotate)*/
	/*          y_value  (move -forward/backward)*/
	/*          slow_mo  if true, scale output - not being done */
	/* return: left scale_value */
	float EntropyDrive::left_scale(float rotateValue, float moveValue, DriveMode mode)
	{
		int x_index = 0;
		int y_index = 0;
		float temp_drive = 0;
		int x_idx = 0;
		float absRotate = rotateValue;
		
		if(mode == Radius)
		{
			absRotate = fabs(rotateValue);
		}	
		
		get_index(x_index, y_index, moveValue, absRotate, mode);
		
		if(mode == Rotate)
		{
			x_idx = x_index;
		}
		else
		{
			if(rotateValue < 0)
			{
				x_idx = 32-x_index;
			}
			else
			{
				x_idx = x_index;
			}
			
		}
		
		temp_drive = left_fast_njxy[y_index][x_idx];
	    

		return temp_drive;
	}

	
	// Code is replicated from RobotDrive class
	float EntropyDrive::Limit(float num)
	{
		if (num > 1.0)
		{
			return 1.0;
		}
		if (num < -1.0)
		{
			return -1.0;
		}
		return num;
	}
	
	
	
	/* search right drive table using binary search if axis index tables */
	/* Input:   x_value  (rotate)*/
	/*          y_value  (move -forward/backward)*/
	/*          slow_mo  if true, scale output - not being done */
	/* return: left scale_value */
	float EntropyDrive::right_scale(float rotateValue, float moveValue, DriveMode mode)
	{
		int x_index = 0;
		int y_index = 0;
		float temp_drive = 0;
		int x_idx = 0;
		float absRotate = rotateValue;
		
		if(mode == Radius)
		{
			absRotate = fabs(rotateValue);
		}

		get_index(x_index, y_index, moveValue, absRotate, mode);
		
		temp_drive = left_fast_njxy[y_index][32-x_index];

		if(mode == Rotate)
		{
			x_idx = 32-x_index;
		}
		else
		{
			if(rotateValue < 0)
			{
				x_idx = x_index;
			}
			else
			{
				x_idx = 32-x_index;
			}
		}
		
		temp_drive = left_fast_njxy[y_index][x_idx];
		
	  
		return temp_drive;
	}
	

	float EntropyDrive::absolutevalue(float x)
	{
		if (x < 0.0) 
		{
			x = x*-1.0;
		}
		return x;
	}

	bool EntropyDrive::range(float x, float y, float z) 
	{  
	   return (((y <= x) && (x <= z)) || ((y >= x) && (x >= z)));
	}


	
	
	float EntropyDrive::drive_table_limit(float x, float max, float min)
	{
		if(x > max)
		{
			return max;
		}
		else if(x < min)
		{
			return min;
		}
		else
		{
			return x;
		}
	}


	void EntropyDrive::get_index(int &x_index, int &y_index, float moveValue, float rotateValue, DriveMode mode)
	{
		float rotate = 0;
		float move = 0;
		float minRotate = 0;
		float maxRotate = 0;
		const float *arrayPtr = 0;
		unsigned int arrayLength = 0;
		float diff1 = 0;
		float diff2 = 0;
		
		if(mode == Radius)
		{
			arrayPtr = left_lookup_radius;
			arrayLength = 18;//sizeof(left_lookup_radius)/sizeof(float);
			minRotate = arrayPtr[16];
			maxRotate = arrayPtr[15];
		}
		else /*Rotate*/
		{
			arrayPtr = left_lookupx;
			arrayLength = sizeof(left_lookupx)/sizeof(float);
			minRotate = arrayPtr[0];
			maxRotate = arrayPtr[arrayLength-1];
		}
		
		rotate = drive_table_limit(rotateValue, maxRotate, minRotate);

		for(unsigned int i = 0; i < arrayLength; i++) 
		{
			if(i+1 >= arrayLength || range(rotate, arrayPtr[i], arrayPtr[i+1]))
			{
				//Assume match found
				if((i + 1) >= arrayLength)
				{
				   x_index = i;	
				}
				else
				{
					diff1 = fabs(rotate - arrayPtr[i]);
					diff2 = fabs(rotate - arrayPtr[i+1]);
					
					if(diff1 < diff2)
					{
						x_index = i;
					}
					else
					{
						x_index = i + 1;
					}
				}
				break;
			}
		}

	    arrayLength = (sizeof(left_lookupy)/sizeof(float));
		move = drive_table_limit(moveValue, left_lookupy[32], left_lookupy[0]);
		
		for(unsigned int i = 0; i < arrayLength; i++) 
		{
			if(i+1 >= arrayLength || range(move, left_lookupy[i], left_lookupy[i+1]))
			{
				//Assume match found
				if((i + 1) >= arrayLength)
				{
				   y_index = i;	
				}
				else
				{
					diff1 = fabs(move - left_lookupy[i]);
					diff2 = fabs(move - left_lookupy[i+1]);
					
					if(diff1 < diff2)
					{
						y_index = i;
					}
					else
					{
						y_index = i + 1;
					}
				}
				break;
			}
		}
	}

