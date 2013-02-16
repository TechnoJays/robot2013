#ifndef CLIMBER_H_
#define CLIMBER_H_

#include "common.h"

// Forward class definitions
class DataLog;
class Encoder;
class Jaguar;
class Parameters;
class Timer;

/**
 * \class Climber
 * \brief Controls a climbing winch.
 * 
 * Provides a simple interface to manually move a climbing winch, or set it to a specific
 * position using sensors.
 */
class Climber {

public:	
	// Public methods
	Climber();
	Climber(bool logging_enabled);
	Climber(char * parameters);
	Climber(char * parameters, bool logging_enabled);
	~Climber();
	bool LoadParameters();
	void ReadSensors();
	void ResetAndStartTimer();
	void SetRobotState(ProgramState state);
	void GetCurrentState(char * output_buffer);
	void LogCurrentState();
	void SetLogState(bool state);
	bool Set(int encoder_count, float speed);
	bool Set(double time, Direction direction, float speed);
	void Move(float directional_speed, bool turbo);
	
	// Public member variables
	bool climber_enabled_;	///< true if the climber (motor) is present and initialized
	bool encoder_enabled_;	///< true if the climber encoder is present and initialized


private:
	// Private methods
	void Initialize(char * parameters, bool logging_enabled);

	// Private member objects
	Jaguar *controller_;		///< motor controller used to move the climber
	Encoder *encoder_;			///< encoder used to track current climber position
	DataLog *log_;				///< log object used to log data or status comments to a file
	Parameters *parameters_;	///< parameters object used to load climber parameters from a file
	Timer *timer_;				///< timer object used for timed autonomous functions
	
	// Private parameters
	float normal_up_speed_ratio_;		///< upward movement speed ratio (percentage) used during 'normal' mode
	float normal_down_speed_ratio_;		///< downward movement speed ratio (percentage) used during 'normal' mode
	float turbo_up_speed_ratio_;		///< upward movement speed ratio (percentage) used during 'turbo' mode
	float turbo_down_speed_ratio_;		///< downward movement speed ratio (percentage) used during 'turbo' mode
	float auto_far_speed_ratio_;		///< movement speed ratio used in autonomous functions when the set point is far away from the current position
	float auto_medium_speed_ratio_;		///< movement speed ratio used in autonomous functions when the set point is a medium distance from the current position
	float auto_near_speed_ratio_;		///< movement speed ratio used in autonomous functions when the set point is close to the current position
	float up_direction_;				///< motor direction required to move the climber up
	float down_direction_;				///< motor direction required to move the climber down
	float invert_multiplier_;			///< used to reverse motor speed/direction if the user-input controls are inverted
	int encoder_threshold_;				///< encoder count threshold for autonomous functions to decide when the climber is 'close enough' to the set point 
	int auto_medium_encoder_threshold_;	///< encoder count threshold between near and medium for autonomous functions
	int auto_far_encoder_threshold_;	///< encoder count threshold between medium and far for autonomous functions
	int encoder_max_limit_;				///< number of encoder counts when the climber reaches its maximum allowed movement
	int encoder_min_limit_;				///< number of encoder counts when the climber reaches its minimum allowed movement
	double time_threshold_;				///< time in seconds for autonomous functions to decide when the climber is 'close enough' to the timed movement
	float auto_medium_time_threshold_;	///< time threshold between near and medium for autonomous functions
	float auto_far_time_threshold_;		///< time threshold between medium and far for autonomous functions

	// Private member variables
	int encoder_count_;			///< current number of encoder counts for the climber
	bool log_enabled_;			///< true if logging is enabled
	char parameters_file_[25];	///< path and filename of the parameter file to read
	ProgramState robot_state_;	///< current state of the robot obtained from the field
};

#endif
