#ifndef __EXAMPLE_SHS__
#define __EXAMPLE_SHS__

#include "WPILib.h"
#include "EntropySubsystemTemplate.h"
	
double const SPEED = 0.2;

class ExampleSHS : public EntropySubsystemTemplate 
{
	
private:
	// The robot shooter left, right, up, down control
	CANJaguar *MotorShootAz;
	CANJaguar *MotorShootEl;

	
public:
	ExampleSHS( );
	
	bool Initialize ();
	
	void Cleanup ();
	
	char * GetFeedback();

	void StopAz();
	void MoveAz( bool move_left, bool move_right);
};
#endif
