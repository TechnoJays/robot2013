#ifndef COMMON_H_
#define COMMON_H_

#include <ctype.h>

// Macros to delete pointers
#define SafeDelete(pointer) if((pointer))\
{ delete(pointer); pointer = NULL;}

#define SafeDeleteArray(pointer) if((pointer))\
{ delete[](pointer); pointer = NULL;}

#define CharArrayToLower(arraypointer) if((arraypointer))\
{ for (unsigned int i=0; i<sizeof(arraypointer); i++) {\
	if (arraypointer[i]) arraypointer[i]=tolower(arraypointer[i]);}}

// General directional enum
enum Direction {
	kLeft,
	kRight,
	kForward,
	kBackward,
	kUp,
	kDown
};

// Robot state enum
enum ProgramState {
	kDisabled,
	kAutonomous,
	kTeleop
};

#endif
