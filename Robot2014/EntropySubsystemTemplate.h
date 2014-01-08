#ifndef __ENTROPY_SUBSYSTEM_TEMPLATE
#define __ENTROPY_SUBSYSTEM_TEMPLATE

class EntropySubsystemTemplate {
	
public:
	EntropySubsystemTemplate (void)
	{
					
	}
	
	
public:
	
	virtual bool Initialize ()=0;
	
	virtual void Cleanup ()=0;
	
	virtual char * GetFeedback()=0;
};

#endif
