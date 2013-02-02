#ifndef DRIVETRAIN_H_
#define DRIVETRAIN_H_

#include "common.h"

// Forward class definitions
class DataLog;
class Jaguar;
class Parameters;
class RobotDrive;


/**
 * \class DriveTrain
 * \brief Drives a robot.
 * 
 * Provides an interface to manually drive the robot, or automatically using
 * various types of sensors.
 */
class DriveTrain {
public:	
	// Public methods
	DriveTrain();
	DriveTrain(bool logging_enabled);
	DriveTrain(const char * parameters);
	DriveTrain(const char * parameters, bool logging_enabled);
	~DriveTrain();
	bool LoadParameters();
	void SetRobotState(ProgramState state);
	void SetLogState(bool state);
	void Drive(float directional_speed, float directional_turn, bool turbo);	// Manual driving
	
	// Public member variables

private:
	// Private methods
	void Initialize(const char * parameters, bool logging_enabled);
		
	// Private member objects
	Jaguar *left_controller_;				///< motor controller used to move the left wheels
	Jaguar *right_controller_;				///< motor controller used to move the right wheels
	RobotDrive *robot_drive_;				///< robot drive object used to drive and turn the robot
	DataLog *log_;							///< log object used to log data or status comments to a file
	Parameters *parameters_;				///< parameters object used to load drive train parameters from a file

	// Private parameters
	float normal_linear_speed_ratio_;		///< linear movement speed ratio (percentage) used during 'normal' mode
	float turbo_linear_speed_ratio_;		///< linear movement speed ratio (percentage) used during 'turbo' mode
	float normal_turning_speed_ratio_;		///< turning movement speed ratio (percentage) used during 'normal' mode
	float turbo_turning_speed_ratio_;		///< turning movement speed ratio (percentage) used during 'turbo' mode
	float forward_direction_;				///< motor direction required to move the robot forward
	float backward_direction_;				///< motor direction required to move the robot backward
	float left_direction_;					///< motor direction required to turn the robot left
	float right_direction_;					///< motor direction required to turn the robot right 
	float invert_multiplier_;				///< used to reverse motor speed/direction if the user-input controls are inverted
	int left_motor_inverted_;				///< specifies if the left motor controller should invert the movement direction of the motor
	int right_motor_inverted_;				///< specifies if the right motor controller should invert the movement direction of the motor
	float linear_filter_constant_;			///< low pass filter constant used to smooth accelerations in linear movement
	float turn_filter_constant_;			///< low pass filter constant used to smooth accelerations in turning movement
	float maximum_linear_speed_change_;		///< maximum amount of change allowed in motor speed used to smooth accelerations during linear movement
	float maximum_turn_speed_change_;		///< maximum amount of change allowed in motor speed used to smooth accelerations during turning movement

	// Private member variables
	float previous_linear_speed_;	///< stores the last known linear motor speed of the robot
	float previous_turn_speed_;		///< stores the last known turning motor speed of the robot
	bool log_enabled_;				///< true if logging is enabled
	char *parameters_file_;			///< path and filename of the parameter file to read
	ProgramState robot_state_;		///< current state of the robot obtained from the field
};

#endif
