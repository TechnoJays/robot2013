#ifndef COMMON_H_
#define COMMON_H_

#define SafeDelete(pointer) if((pointer))\
{ delete(pointer); pointer = NULL;}

enum Direction {
	kLeft,
	kRight,
	kForward,
	kBackward,
	kUp,
	kDown
};

enum ProgramState {
	kDisabled,
	kAutonomous,
	kTeleop
};

#endif
