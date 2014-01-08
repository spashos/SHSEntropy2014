#ifndef IO_DEFINITIONS_H_
#define IO_DEFINITIONS_H_


class IODefinitions {
	
public:
	enum  {
		UNUSED_0 =0,
		UNUSED_1,
		MOTOR_DRIVE_LEFT_1,
		MOTOR_DRIVE_LEFT_2,
		MOTOR_DRIVE_RIGHT_1,
		MOTOR_DRIVE_RIGHT_2,
		UNUSED_6,
		UNUSED_7,
		UNUSED_8,
		UNUSED_9,
		UNUSED_10
	} CanBus;

    enum {
    	USB_PORT_1 = 1,
    	USB_PORT_2
    	
    } Physical_USB_Port;
    
    /**
     *  TODO: Map the correct buttons from the controller to this. 
     */
    enum { 
    	GAME_BUTTON_NOT_USED = 0,
    	GAME_BUTTON_1        = 1,
    	GAME_BUTTON_2,
    	GAME_BUTTON_3,
    	GAME_BUTTON_4,
    	GAME_BUTTON_SHOOTER_UP,
    	GAME_BUTTON_SHOOTER_DOWN,
    	GAME_BUTTON_SHOOTER_LEFT,
    	GAME_BUTTON_SHOOTER_RIGHT
    } Game_Stick_IO;
    
    /**
     * TODO: Map the correct buttons from the drive stick to this. 
     */
    enum {
    	DRIVE_BUTTON_A, 
    	DRIVE_BUTTON_B
    } Drive_Stick_IO;
};

#endif
