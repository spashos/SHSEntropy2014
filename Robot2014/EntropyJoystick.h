#ifndef __ENTROPY_JOYSTICK_H
#define __ENTROPY_JOYSTICK_H


#include "WPILib.h"

/**
 * This is a wrapper to assure we can make updates to a Joystick class. 
 * If no changes are needed, no need to put more code.
 */
class EntropyJoystick : public Joystick {
	
public: 
	EntropyJoystick ( int port );
};

#endif
