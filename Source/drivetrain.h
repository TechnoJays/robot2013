#ifndef DRIVETRAIN_H_
#define DRIVETRAIN_H_

#include "common.h"

// Forward class definitions
class ADXL345_I2C;
class DataLog;
class Gyro;
class Jaguar;
class Parameters;
class RobotDrive;
class Timer;


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
	void ReadSensors();
	void ResetSensors();
	void ResetAndStartTimer();
	void SetRobotState(ProgramState state);
	void GetCurrentState(char * output_buffer);
	void LogCurrentState();
	void SetLogState(bool state);
	bool AdjustHeading(float adjustment, float speed);
	bool Drive(double length, float speed);										// Diving via accelerometer
	bool Drive(double time, Direction direction, float speed);					// Driving via time
	void Drive(float directional_speed, float directional_turn, bool turbo);	// Manual driving
	void TankDrive(float left_stick, float right_stick, bool turbo);			// Manual driving in 'Tank' mode
	bool Turn(float heading, float speed);										// Turning via gyro
	bool Turn(double time, Direction direction, float speed);					// Turning via time
	float GetHeading();

	// Public member variables
	bool accelerometer_enabled_;	///< true if the accelerometer is present and initialized
	bool gyro_enabled_;				///< true if the gyro is present and initialized

private:
	// Private methods
	void Initialize(const char * parameters, bool logging_enabled);
		
	// Private member objects
	Jaguar *left_controller_;				///< motor controller used to move the left wheels
	Jaguar *right_controller_;				///< motor controller used to move the right wheels
	RobotDrive *robot_drive_;				///< robot drive object used to drive and turn the robot
	ADXL345_I2C *accelerometer_;			///< accelerometer used to track accelerations in 3 dimensions
	Gyro *gyro_;							///< gyro used to track robot's current heading in degrees
	DataLog *log_;							///< log object used to log data or status comments to a file
	Parameters *parameters_;				///< parameters object used to load drive train parameters from a file
	Timer *acceleration_timer_;				///< timer object used to calculate distance traveled using the accelerometer
	Timer *timer_;							///< timer object used for timed autonomous functions

	// Private parameters
	float normal_linear_speed_ratio_;		///< linear movement speed ratio (percentage) used during 'normal' mode
	float turbo_linear_speed_ratio_;		///< linear movement speed ratio (percentage) used during 'turbo' mode
	float normal_turning_speed_ratio_;		///< turning movement speed ratio (percentage) used during 'normal' mode
	float turbo_turning_speed_ratio_;		///< turning movement speed ratio (percentage) used during 'turbo' mode
	float auto_far_linear_speed_ratio_;		///< linear movement speed ratio used in autonomous functions when the set point is far away from the current position
	float auto_medium_linear_speed_ratio_;	///< linear movement speed ratio used in autonomous functions when the set point is a medium distance from the current position
	float auto_near_linear_speed_ratio_;	///< linear movement speed ratio used in autonomous functions when the set point is close to the current position
	float auto_far_turning_speed_ratio_;	///< turning movement speed ratio used in autonomous functions when the set point is far away from the current position
	float auto_medium_turning_speed_ratio_;	///< turning movement speed ratio used in autonomous functions when the set point is a medium distance from the current position
	float auto_near_turning_speed_ratio_;	///< turning movement speed ratio used in autonomous functions when the set point is close to the current position
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
	double time_threshold_;					///< time in seconds for autonomous functions to decide when the robot is 'close enough' to the timed movement
	float auto_medium_time_threshold_;		///< time threshold between near and medium for autonomous functions
	float auto_far_time_threshold_;			///< time threshold between medium and far for autonomous functions
	float distance_threshold_;				///< distance in meters for autonomous functions to decide when the robot is 'close enough' to the set point
	float auto_medium_distance_threshold_;	///< distance threshold between near and medium for autonomous functions
	float auto_far_distance_threshold_;		///< distance threshold between medium and far for autonomous functions
	float heading_threshold_;				///< heading in degrees for autonomous functions to decide when the robot is 'close enough' to the set point
	float auto_medium_heading_threshold_;	///< heading threshold between near and medium for autonomous functions
	float auto_far_heading_threshold_;		///< heading threshold between medium and far for autonomous functions
	int accelerometer_axis_;				///< accelerometer axis to use for linear distance calculations

	// Private member variables
	double acceleration_;			///< current acceleration of the specified axis
	double distance_traveled_;		///< current distance traveled by the robot
	float gyro_angle_;				///< current heading
	float initial_heading_;			///< stores the initial heading of the robot when a heading adjustment is requested
	float previous_linear_speed_;	///< stores the last known linear motor speed of the robot
	float previous_turn_speed_;		///< stores the last known turning motor speed of the robot
	bool adjustment_in_progress_;	///< true if a heading adjustment is in progress, false if it is a new request
	bool log_enabled_;				///< true if logging is enabled
	char parameters_file_[25];		///< path and filename of the parameter file to read
	ProgramState robot_state_;		///< current state of the robot obtained from the field
};

#endif
