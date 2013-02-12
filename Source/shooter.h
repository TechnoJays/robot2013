#ifndef SHOOTER_H_
#define SHOOTER_H_

#include "common.h"

// Forward class definitions
class DataLog;
class Encoder;
class Jaguar;
class Parameters;
class Timer;

/**
 * \class Shooter
 * \brief Controls a robot shooting mechanism.
 * 
 * Provides a simple interface to shoot a disc, manually set the pitch of the shot,
 * or set the pitch position using sensors.
 */
class Shooter {

public:		
	// Public methods
	Shooter();
	Shooter(bool logging_enabled);
	Shooter(char * parameters);
	Shooter(char * parameters, bool logging_enabled);	
	~Shooter();
	bool LoadParameters();
	void ReadSensors();
	void ResetAndStartTimer();
	void SetRobotState(ProgramState state);
	void GetCurrentState(char * output_buffer);
	void LogCurrentState();
	void SetLogState(bool state);
	bool SetPitch(int encoder_count, float speed);
	bool SetPitch(double time, Direction direction, float speed);
	bool SetPitchAngle(float angle, float speed);
	void MovePitch(float directional_speed, bool turbo);
	void Shoot(int power_as_percent);
	bool Shoot(double time, int power_as_percent);
	
	// Public member variables
	bool encoder_enabled_;	///< true if the pitch encoder is present and initialized
	bool pitch_enabled_;	///< true if the pitch (motor) is present and initialized
	bool shooter_enabled_;	///< true if the shooter (motor) is present and initialized

private:
	// Private methods
	void Initialize(char * parameters, bool logging_enabled);
	
	// Private member objects
	Jaguar *pitch_controller_;		///< motor controller used to move the pitch
	Jaguar *shooter_controller_;	///< motor controller used to move the shooter
	Encoder *encoder_;				///< encoder used to track current pitch position
	DataLog *log_;					///< log object used to log data or status comments to a file
	Parameters *parameters_;		///< parameters object used to load shooter parameters from a file
	Timer *timer_;					///< timer object used for timed autonomous functions
	
	// Private parameters
	float shooter_normal_speed_ratio_;		///< shooter movement speed ratio (percentage) used during 'normal' mode
	float shooter_min_power_speed_;			///< motor speed to use as the baseline power (0%) for the shooter
	float shooter_power_adjustment_ratio_;	///< ratio to convert shooter power percentage value into a motor speed
	float pitch_normal_speed_ratio_;		///< pitch movement speed ratio (percentage) used during 'normal' mode
	float pitch_turbo_speed_ratio_;			///< pitch movement speed ratio (percentage) used during 'turbo' mode
	float auto_far_speed_ratio_;			///< movement speed ratio used in autonomous functions when the set point is far away from the current position
	float auto_medium_speed_ratio_;			///< movement speed ratio used in autonomous functions when the set point is a medium distance from the current position
	float auto_near_speed_ratio_;			///< movement speed ratio used in autonomous functions when the set point is close to the current position
	float pitch_up_direction_;				///< motor direction required to move the pitch up
	float pitch_down_direction_;			///< motor direction required to move the pitch down
	float shoot_forward_direction_;			///< motor direction required to shoot forward
	float shoot_backward_direction_;		///< motor direction required to shoot backward
	float invert_multiplier_;				///< used to reverse motor speed/direction if the user-input controls are inverted
	int encoder_threshold_;					///< encoder count threshold for autonomous functions to decide when the pitch is 'close enough' to the set point
	int auto_medium_encoder_threshold_;		///< encoder count threshold between near and medium for autonomous functions
	int auto_far_encoder_threshold_;		///< encoder count threshold between medium and far for autonomous functions
	int encoder_max_limit_;					///< number of encoder counts when the pitch reaches its maximum allowed movement
	int encoder_min_limit_;					///< number of encoder counts when the pitch reaches its minimum allowed movement
	double time_threshold_;					///< time in seconds for autonomous functions to decide when the pitch is 'close enough' to the timed movement
	float auto_medium_time_threshold_;		///< time threshold between near and medium for autonomous functions
	float auto_far_time_threshold_;			///< time threshold between medium and far for autonomous functions
	float angle_linear_fit_gradient_;		///< linear fit gradient used in converting an angle to encoder counts for setting the pitch
	float angle_linear_fit_constant_;		///< linear fit constant used in converting an angle to encoder counts for setting the pitch

	// Private member variables
	int encoder_count_;			///< current number of encoder counts for the pitch
	bool log_enabled_;			///< true if logging is enabled
	char *parameters_file_;		///< path and filename of the parameter file to read
	ProgramState robot_state_;	///< current state of the robot obtained from the field
};

#endif
