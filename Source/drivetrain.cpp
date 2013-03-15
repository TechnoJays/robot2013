#include <math.h>
#include "WPILib.h"
#include "drivetrain.h"
#include "datalog.h"
#include "parameters.h"

/**
 * \def PI
 * \brief the value of Pi to 8 decimal places.
 */
#define PI 3.14159265


/**
 * \brief Create and initialize a drive train.
 *
 * Use the default parameter file "drivetrain.par" and logging is disabled.
*/
DriveTrain::DriveTrain() {
	Initialize("drivetrain.par", false);
}

/**
 * \brief Create and initialize a drive train.
 *
 * Use the default parameter file "drivetrain.par" and enable/disable
 * logging based on the parameter.
 *
 * \param logging_enabled true if logging is enabled.
*/
DriveTrain::DriveTrain(bool logging_enabled) {
	Initialize("drivetrain.par", logging_enabled);
}

/**
 * \brief Create and initialize a drive train.
 *
 * Use the user specified parameter file and logging is disabled.
 *
 * \param parameters drive train parameter file path and name.
*/
DriveTrain::DriveTrain(const char * parameters) {
	Initialize(parameters, false);
}

/**
 * \brief Create and initialize a drive train.
 *
 * Use the user specified parameter file and enable/disable
 * logging based on the parameter.
 *
 * \param parameters drive train parameter file path and name.
 * \param logging_enabled true if logging is enabled.
*/
DriveTrain::DriveTrain(const char * parameters, bool logging_enabled) {
	Initialize(parameters, logging_enabled);
}

/**
 * \brief Delete and clear all objects and pointers.
*/
DriveTrain::~DriveTrain() {
	if (log_ != NULL) {
		log_->Close();
	}
	
	SafeDelete(timer_);
	SafeDelete(acceleration_timer_);
	SafeDelete(log_);
	SafeDelete(parameters_);
	SafeDelete(accelerometer_);
	SafeDelete(robot_drive_);
	SafeDelete(left_controller_);
	SafeDelete(right_controller_);
	SafeDelete(gyro_);
}

/**
 * \brief Initialize the DriveTrain object.
 *
 * Create member objects, initialize default values, read parameters from the param file.
 *
 * \param parameters drive train parameter file path and name.
 * \param logging_enabled true if logging is enabled.
*/
void DriveTrain::Initialize(const char * parameters, bool logging_enabled) {
	// Initialize public member variables
	gyro_enabled_ = false;
	accelerometer_enabled_ = false;

	// Initialize private member objects
	robot_drive_ = NULL;
	left_controller_ = NULL;
	right_controller_ = NULL;
	gyro_ = NULL;
	accelerometer_ = NULL;
	timer_ = NULL;
	acceleration_timer_ = NULL;
	log_ = NULL;
	parameters_ = NULL;

	
	// Initialize private parameters
	invert_multiplier_ = 0.0;
	normal_linear_speed_ratio_ = 1.0;
	turbo_linear_speed_ratio_ = 1.0;
	normal_turning_speed_ratio_ = 1.0;
	turbo_turning_speed_ratio_ = 1.0;
	auto_far_linear_speed_ratio_ = 1.0;
	auto_medium_linear_speed_ratio_ = 1.0;
	auto_near_linear_speed_ratio_ = 1.0;
	auto_far_turning_speed_ratio_ = 1.0;
	auto_medium_turning_speed_ratio_ = 1.0;
	auto_near_turning_speed_ratio_ = 1.0;
	auto_medium_time_threshold_ = 0.5;
	auto_far_time_threshold_ = 1.0;
	auto_medium_distance_threshold_ = 2.0;
	auto_far_distance_threshold_ = 5.0;
	auto_medium_heading_threshold_ = 15.0;
	auto_far_heading_threshold_ = 25.0;
	distance_threshold_ = 0.5;
	heading_threshold_ = 3.0;
	time_threshold_ = 0.1;
	forward_direction_ = 1.0;
	backward_direction_ = -1.0;
	left_direction_ = -1.0;
	right_direction_ = 1.0;
	left_motor_inverted_ = 0;
	right_motor_inverted_ = 0;
	accelerometer_axis_ = 0;
	maximum_linear_speed_change_ = 0.0;
	maximum_turn_speed_change_ = 0.0;
	linear_filter_constant_ = 0.0;
	turn_filter_constant_ = 0.0;
	
	// Initialize private member variables
	acceleration_ = 0.0;
	gyro_angle_ = 0.0;
	initial_heading_ = 0.0;
	adjustment_in_progress_ = false;
	distance_traveled_ = 0.0;
	log_enabled_ = false;
	robot_state_ = kDisabled;
	previous_linear_speed_ = 0.0;
	previous_turn_speed_ = 0.0;

		
	// Create a new data log object
	log_ = new DataLog("drivetrain.log");

	// Enable logging if specified
	if (log_ != NULL && log_->file_opened_) {
		log_enabled_ = logging_enabled;
	}
	else {
		log_enabled_ = false;
	}
	
	// Create a timer object
	timer_ = new Timer();

	// Attempt to read the parameters file
	strncpy(parameters_file_, parameters, sizeof(parameters_file_));

	LoadParameters();
}

/**
 * \brief Loads the parameter file into memory, copies the values into local/member variables,
 * and creates and initializes objects using those values.
*/
bool DriveTrain::LoadParameters() {
	// Define and initialize local variables
	int left_motor_slot = -1;
	int left_motor_channel = -1;
	int right_motor_slot = -1;
	int right_motor_channel = -1;
	int invert_controls = 0;
	int accelerometer_slot = -1;
	int accelerometer_range = -1;
	int gyro_channel = -1;
	float gyro_sensitivity = 0.007;
	float motor_safety_timeout = 2.0;
	bool parameters_read = false;	// This should default to false


	
	// Close and delete old objects
	SafeDelete(parameters_);
	SafeDelete(robot_drive_);
	SafeDelete(left_controller_);
	SafeDelete(right_controller_);
	SafeDelete(accelerometer_);
	SafeDelete(acceleration_timer_);
	SafeDelete(gyro_);

	
	// Attempt to read the parameters file
	parameters_ = new Parameters(parameters_file_);
	if (parameters_ != NULL && parameters_->file_opened_) {
		parameters_read = parameters_->ReadValues();
		parameters_->Close();
	}
	
	if (log_enabled_) {
		if (parameters_read)
			log_->WriteLine("DriveTrain parameters loaded successfully\n");
		else
			log_->WriteLine("DriveTrain parameters failed to read\n");
	}
	
	// Set arm variables based on the parameters file
	if (parameters_read) {
		parameters_->GetValue("LEFT_MOTOR_SLOT", &left_motor_slot);
		parameters_->GetValue("LEFT_MOTOR_CHANNEL", &left_motor_channel);
		parameters_->GetValue("LEFT_MOTOR_INVERTED", &left_motor_inverted_);
		parameters_->GetValue("RIGHT_MOTOR_SLOT", &right_motor_slot);
		parameters_->GetValue("RIGHT_MOTOR_CHANNEL", &right_motor_channel);
		parameters_->GetValue("RIGHT_MOTOR_INVERTED", &right_motor_inverted_);
		parameters_->GetValue("MOTOR_SAFETY_TIMEOUT", &motor_safety_timeout);
		parameters_->GetValue("ACCELEROMETER_SLOT", &accelerometer_slot);
		parameters_->GetValue("ACCELEROMETER_RANGE", &accelerometer_range);
		parameters_->GetValue("ACCELEROMETER_AXIS", &accelerometer_axis_);
		parameters_->GetValue("GYRO_CHANNEL", &gyro_channel);
		parameters_->GetValue("GYRO_SENSITIVITY", &gyro_sensitivity);
		parameters_->GetValue("INVERT_CONTROLS", &invert_controls);		
		parameters_->GetValue("FORWARD_DIRECTION", &forward_direction_);
		parameters_->GetValue("BACKWARD_DIRECTION", &backward_direction_);
		parameters_->GetValue("LEFT_DIRECTION", &left_direction_);
		parameters_->GetValue("RIGHT_DIRECTION", &right_direction_);
		parameters_->GetValue("NORMAL_LINEAR_SPEED_RATIO", &normal_linear_speed_ratio_);
		parameters_->GetValue("TURBO_LINEAR_SPEED_RATIO", &turbo_linear_speed_ratio_);
		parameters_->GetValue("NORMAL_TURNING_SPEED_RATIO", &normal_turning_speed_ratio_);
		parameters_->GetValue("TURBO_TURNING_SPEED_RATIO", &turbo_turning_speed_ratio_);
		parameters_->GetValue("AUTO_FAR_LINEAR_SPEED_RATIO", &auto_far_linear_speed_ratio_);
		parameters_->GetValue("AUTO_MEDIUM_LINEAR_SPEED_RATIO", &auto_medium_linear_speed_ratio_);
		parameters_->GetValue("AUTO_NEAR_LINEAR_SPEED_RATIO", &auto_near_linear_speed_ratio_);
		parameters_->GetValue("AUTO_FAR_TURNING_SPEED_RATIO", &auto_far_turning_speed_ratio_);
		parameters_->GetValue("AUTO_MEDIUM_TURNING_SPEED_RATIO", &auto_medium_turning_speed_ratio_);
		parameters_->GetValue("AUTO_NEAR_TURNING_SPEED_RATIO", &auto_near_turning_speed_ratio_);
		parameters_->GetValue("DISTANCE_THRESHOLD", &distance_threshold_);
		parameters_->GetValue("HEADING_THRESHOLD", &heading_threshold_);
		parameters_->GetValue("TIME_THRESHOLD", &time_threshold_);
		parameters_->GetValue("AUTO_MEDIUM_TIME_THRESHOLD", &auto_medium_time_threshold_);
		parameters_->GetValue("AUTO_FAR_TIME_THRESHOLD", &auto_far_time_threshold_);
		parameters_->GetValue("AUTO_MEDIUM_DISTANCE_THRESHOLD", &auto_medium_distance_threshold_);
		parameters_->GetValue("AUTO_FAR_DISTANCE_THRESHOLD", &auto_far_distance_threshold_);
		parameters_->GetValue("AUTO_MEDIUM_HEADING_THRESHOLD", &auto_medium_heading_threshold_);
		parameters_->GetValue("AUTO_FAR_HEADING_THRESHOLD", &auto_far_heading_threshold_);
		parameters_->GetValue("MAXIMUM_LINEAR_SPEED_CHANGE", &maximum_linear_speed_change_);
		parameters_->GetValue("MAXIMUM_TURN_SPEED_CHANGE", &maximum_turn_speed_change_);
		parameters_->GetValue("LINEAR_FILTER_CONSTANT", &linear_filter_constant_);
		parameters_->GetValue("TURN_FILTER_CONSTANT", &turn_filter_constant_);
	}

	// Check if the accelerometer is present/enabled
	if (accelerometer_slot > 0 && accelerometer_range >= 0) {
		accelerometer_ = new ADXL345_I2C(accelerometer_slot, (ADXL345_I2C::DataFormat_Range) accelerometer_range);
		if (accelerometer_ != NULL) {
			accelerometer_enabled_ = true;
			acceleration_timer_ = new Timer();
		}
	}
	else {
		accelerometer_enabled_ = false;
	}

	// Check if the gyro is present/enabled
	if (gyro_channel > 0) {
		gyro_ = new Gyro(gyro_channel);
		if (gyro_ != NULL) {
			gyro_->SetSensitivity(gyro_sensitivity);
			gyro_enabled_ = true;
		}
	}
	else {
		gyro_enabled_ = false;
	}
	
	// Create motor controller objects
	if (left_motor_slot > 0 && left_motor_channel > 0)
		left_controller_ = new Jaguar(left_motor_slot, left_motor_channel);
	if (right_motor_slot > 0 && right_motor_channel > 0)
		right_controller_ = new Jaguar(right_motor_slot, right_motor_channel);
	
	// Create the RobotDrive object using the motor controllers
	if (left_controller_ != NULL && right_controller_ != NULL) {
		robot_drive_ = new RobotDrive(left_controller_, right_controller_);
		robot_drive_->SetExpiration(motor_safety_timeout);
		robot_drive_->SetSafetyEnabled(true);
	}
	
	// Invert motors if specified
	if (left_motor_inverted_ && robot_drive_ != NULL) {
		robot_drive_->SetInvertedMotor(RobotDrive::kRearLeftMotor, true);
	}
	if (right_motor_inverted_ && robot_drive_ != NULL) {
		robot_drive_->SetInvertedMotor(RobotDrive::kRearRightMotor, true);
	}
	
	if (log_enabled_) {
		if (accelerometer_enabled_) {
			log_->WriteLine("Accelerometer enabled\n");
		}
		else {
			log_->WriteLine("Accelerometer disabled\n");
		}
		if (gyro_enabled_) {
			log_->WriteLine("Gyro enabled\n");
		}
		else {
			log_->WriteLine("Gyro disabled\n");
		}
	}
	
	// Set the inversion multiplier depending on the controls setting
	if (invert_controls) {
		invert_multiplier_ = -1.0;
	}
	else {
		invert_multiplier_ = 1.0;
	}
	
	return parameters_read;
}

/**
 * \brief Read and store current sensor values.
*/
void DriveTrain::ReadSensors() {
	double loop_time = 0.0;
	
	if (gyro_enabled_) {
		gyro_angle_ = gyro_->GetAngle();
	}
	if (accelerometer_enabled_) {
		acceleration_ = accelerometer_->GetAcceleration((ADXL345_I2C::Axes) accelerometer_axis_);
		if (acceleration_timer_ != NULL) {
			loop_time = acceleration_timer_->Get();
			acceleration_timer_->Reset();
			distance_traveled_ += (acceleration_ * loop_time * loop_time);
		}
	}
}

/**
 * \brief Reset sensors
*/
void DriveTrain::ResetSensors() {	
	if (gyro_enabled_) {
		gyro_->Reset();
	}
	if (accelerometer_enabled_) {
		acceleration_timer_->Reset();
		distance_traveled_ = 0.0;
	}
}

/**
 * \brief Resets and restarts the timer for time based movement.
*/
void DriveTrain::ResetAndStartTimer() {
	if (timer_ != NULL) {
		timer_->Stop();
		timer_->Reset();
		timer_->Start();
	}
}

/**
 * \brief Set the current state of the robot and perform any actions necessary during mode changes.
 *
 * \param state current robot state.
*/
void DriveTrain::SetRobotState(ProgramState state) {
	robot_state_ = state;

	if (timer_ != NULL) {
		timer_->Stop();
	}
	
	if (accelerometer_enabled_) {
		if (acceleration_timer_ != NULL) {
			acceleration_timer_->Stop();
			acceleration_timer_->Reset();
			acceleration_timer_->Start();
		}
		distance_traveled_ = 0.0;
	}
	
	if (state == kDisabled) {
		robot_drive_->SetSafetyEnabled(true);
	}
	else if (state == kTeleop) {
		robot_drive_->SetSafetyEnabled(true);
	}
	else if (state == kAutonomous) {
		robot_drive_->SetSafetyEnabled(false);
	}
}

/**
 * \brief Return a string containing sensor and status variables.
 *
 * \param output_buffer empty character array to contain the string.
*/
void DriveTrain::GetCurrentState(char * output_buffer) {
	sprintf(output_buffer, "%3.0f %3.2f %2.1f", gyro_angle_, acceleration_, distance_traveled_);
}

/**
 * \brief Log sensor and status variables when requested.
*/
void DriveTrain::LogCurrentState() {
	if (log_ != NULL) {
		if (gyro_enabled_)
			log_->WriteValue("Gyro angle", gyro_angle_, true);
		if (accelerometer_enabled_) {
			log_->WriteValue("Acceleration", acceleration_, true);
			log_->WriteValue("Distance traveled", distance_traveled_, true);
		}
	}
}

/**
 * \brief Enable or disable logging for this object.
 *
 * \param state true if logging should be enabled.
*/
void DriveTrain::SetLogState(bool state) {
	if (state && log_ != NULL)
		log_enabled_ = true;
	else
		log_enabled_ = false;
}

/**
 * \brief Turns left/right to adjust by a specific heading provided by the argument.
 *
 * Uses the gyro to keep track of the current heading.
 *
 * \param adjustment the heading adjustment in degrees.
 * \param speed the motor speed ratio.
 * \return true when the requested heading has been reached.
*/
bool DriveTrain::AdjustHeading(float adjustment, float speed) {
	// Abort if robot drive or gyro is not available
	if (robot_drive_ == NULL || !gyro_enabled_) {
		adjustment_in_progress_ = false;
		return true;
	}
	
	if (!adjustment_in_progress_) {
		initial_heading_ = gyro_angle_;
		adjustment_in_progress_ = true;
	}
	
	float angle_remaining = 0.0;
	float turn_direction = 0.0;
			
	// Calculate the amount left to turn, in degrees
	angle_remaining = (initial_heading_ + adjustment) - gyro_angle_;

	// Determine the turn direction
	if (angle_remaining < 0) {
		turn_direction = left_direction_;
	}
	else {
		turn_direction = right_direction_;
	}
	
	// Check to see if we've reached the desired heading
	if (fabs(angle_remaining) < heading_threshold_) {
		//robot_drive_->Drive(0.0, 0.0);
		robot_drive_->ArcadeDrive(0.0, 0.0, false);
		adjustment_in_progress_ = false;
		return true;
	}
	else {
		if (fabs(angle_remaining) > auto_far_heading_threshold_) {
			turn_direction = turn_direction * speed * auto_far_turning_speed_ratio_;
		}
		else if (fabs(angle_remaining) > auto_medium_heading_threshold_) {
			turn_direction = turn_direction * speed * auto_medium_turning_speed_ratio_;
		}
		else {
			turn_direction = turn_direction * speed * auto_near_turning_speed_ratio_;
		}
		
		//robot_drive_->Drive(0.0, turn_direction);
		robot_drive_->ArcadeDrive(0.0, turn_direction, false);
		return false;
	}
}

/**
 * \brief Drives forward/backward a distance provided by the argument.
 *
 * \param directional_length the distance in meters.
 * \param speed motor speed ratio.
 * \return true when the desired position has been reached.
*/
bool DriveTrain::Drive(double directional_length, float speed) {
	// Abort if robot drive or accelerometer is not available
	if (robot_drive_ == NULL || !accelerometer_enabled_) {
		return true;
	}
	
	double distance_left = 0.0;
	float direction_multiplier = 0.0;
	
	// Determine if we're driving forwards or backwards
	if (directional_length > 0) {
		direction_multiplier = forward_direction_;
	}
	else {
		direction_multiplier = backward_direction_;
	}

	// Calculate distance left to drive
	distance_left = fabs(directional_length) - fabs(distance_traveled_);
	
	// Check to see if we've reached the desired distance
	if (distance_left < distance_threshold_) {
		// Stop driving
		//robot_drive_->Drive(0.0, 0.0);
		robot_drive_->ArcadeDrive(0.0, 0.0, false);
		return true;
	}
	else {
		if (distance_left > auto_far_distance_threshold_) {
			direction_multiplier = direction_multiplier * speed * auto_far_linear_speed_ratio_;
		}
		else if (distance_left > auto_medium_distance_threshold_) {
			direction_multiplier = direction_multiplier * speed * auto_medium_linear_speed_ratio_;
		}
		else {
			direction_multiplier = direction_multiplier * speed * auto_near_linear_speed_ratio_;
		}
		
		//robot_drive_->Drive(direction_multiplier, 0.0);
		robot_drive_->ArcadeDrive(direction_multiplier, 0.0, false);
		return false;
	}	
}

/**
 * \brief Drives forward/backward for a time duration provided by the argument.
 *
 * \param time amount of time to drive.
 * \param direction the direction to drive.
 * \param speed motor speed ratio.
 * \return true when the desired position is reached.
*/
bool DriveTrain::Drive(double time, Direction direction, float speed) {
	// Abort if robot drive is not available
	if (robot_drive_ == NULL || timer_ == NULL) {
		return true;
	}
	
	double time_left = 0.0;
	float directional_speed = 0.0;
	double elapsed_time = 999.0;

	// Get the timer value since we started moving
	elapsed_time = timer_->Get();
		
	// Calculate time left to move
	time_left = time - elapsed_time;
	
	// Check to see if we've reached the proper position
	if ((time_left < time_threshold_) || (time_left < 0)) {
		//robot_drive_->Drive(0.0, 0.0);
		robot_drive_->ArcadeDrive(0.0, 0.0, false);
		timer_->Stop();
		return true;
	}
	// Continue moving
	else {		
		if (direction == kForward) {
			directional_speed = forward_direction_;
		}
		else {
			directional_speed = backward_direction_;
		}
		
		if (time_left > auto_far_time_threshold_){
			directional_speed = directional_speed * speed * auto_far_linear_speed_ratio_;
		}
		else if (time_left > auto_medium_time_threshold_) {
			directional_speed = directional_speed * speed * auto_medium_linear_speed_ratio_;
		}
		else {
			directional_speed = directional_speed * speed * auto_near_linear_speed_ratio_;
		}
		
		//robot_drive_->Drive(directional_speed, 0.0);
		robot_drive_->ArcadeDrive(directional_speed, 0.0, false);
		return false;
	}
}

/**
 * \brief Drives the robot using the specified linear speed and turning speed.
 *
 * \param directional_speed the speed and direction to drive forward/backward.
 * \param directional_turn the speed and direction to turn left/right.
 * \param turbo true if the robot should move at 'turbo' speed.
*/
void DriveTrain::Drive(float directional_speed, float directional_turn, bool turbo) {
	// Abort if robot drive is not available
	if (robot_drive_ == NULL) {
		return;
	}
	
	float linear = 0.0;
	float turn = 0.0;
	
	// Determine actual speed using normal/turbo speed ratios
	if (turbo) {
		linear = turbo_linear_speed_ratio_ * directional_speed;
		turn = turbo_turning_speed_ratio_ * directional_turn;
	}
	else {
		linear = normal_linear_speed_ratio_ * directional_speed;
		turn = normal_turning_speed_ratio_ * directional_turn;
	}
	
	// Make sure the requested speed isn't too much of a change from the last request
	// This is to prevent tipping or jerky movement
	// E.g., Going from Full reverse to full forward
	// If the difference in the requested speed is greater than the threshold,
	// throttle the request to only be 1 * maximum change from the previous value
	if (fabs(linear - previous_linear_speed_) > maximum_linear_speed_change_) {
		if ((linear - previous_linear_speed_) < 0)
			linear = previous_linear_speed_ - maximum_linear_speed_change_;
		else
			linear = previous_linear_speed_ + maximum_linear_speed_change_;
	}
	if (fabs(turn - previous_turn_speed_) > maximum_turn_speed_change_) {
		if ((turn - previous_turn_speed_) < 0)
			turn = previous_turn_speed_ - maximum_turn_speed_change_;
		else
			turn = previous_turn_speed_ + maximum_turn_speed_change_;
	}

	// Other method of smoothing acceleration
	// Uses a simple LPF
	// New speed = target speed - K * (target speed - current speed)
	// Where K should be about 0.8? (higher = slower rate of change)
	//linear = linear - linear_filter_constant_ * (linear - previous_linear_speed_);
	//turn = turn - turn_filter_constant_ * (turn - previous_turn_speed_);
	
	robot_drive_->ArcadeDrive(linear, turn, false);
	//robot_drive_->Drive(linear, turn);
	
	previous_linear_speed_ = linear;
	previous_turn_speed_ = turn;
}

/**
 * \brief Drives the robot using the left and right thumbsticks 'Tank' style
 *
 * \param left_stick the vertical position of the left thumbstick
 * \param right_stick the vertical position of the right thumbstick
 * \param turbo true if the robot should move at 'turbo' speed.
*/
void DriveTrain::TankDrive(float left_stick, float right_stick, bool turbo) {
	// Abort if robot drive is not available
	if (robot_drive_ == NULL) {
		return;
	}
	
	float left = 0.0;
	float right = 0.0;
	
	// Determine actual speed using normal/turbo speed ratios
	if (turbo) {
		left = turbo_linear_speed_ratio_ * left_stick;
		right = turbo_linear_speed_ratio_ * right_stick;
	}
	else {
		left = normal_linear_speed_ratio_ * left_stick;
		right = normal_linear_speed_ratio_ * right_stick;
	}
		
	robot_drive_->TankDrive(left, right, false);	
}

/**
 * \brief Turns left/right to face a specific heading provided by the argument.
 *
 * \param heading the desired heading in degrees.
 * \param speed the motor speed ratio.
 * \return true when the requested heading has been reached.
*/
bool DriveTrain::Turn(float heading, float speed) {
	// Abort if robot drive or gyro is not available
	if (robot_drive_ == NULL || !gyro_enabled_) {
		return true;
	}
	
	float angle_remaining = 0.0;
	float turn_direction = 0.0;
			
	// Calculate the amount left to turn, in degrees
	angle_remaining = heading - gyro_angle_;

	// Determine the turn direction
	if (angle_remaining < 0) {
		turn_direction = left_direction_;
	}
	else {
		turn_direction = right_direction_;
	}
	
	// Check to see if we've reached the desired heading
	if (fabs(angle_remaining) < heading_threshold_) {
		//robot_drive_->Drive(0.0, 0.0);
		robot_drive_->ArcadeDrive(0.0, 0.0, false);
		return true;
	}
	else {
		if (fabs(angle_remaining) > auto_far_heading_threshold_) {
			turn_direction = turn_direction * speed * auto_far_turning_speed_ratio_;
		}
		else if (fabs(angle_remaining) > auto_medium_heading_threshold_) {
			turn_direction = turn_direction * speed * auto_medium_turning_speed_ratio_;
		}
		else {
			turn_direction = turn_direction * speed * auto_near_turning_speed_ratio_;
		}
		
		//robot_drive_->Drive(0.0, turn_direction);
		robot_drive_->ArcadeDrive(0.0, turn_direction, false);
		return false;
	}	
}

/**
 * \brief Turns left/right for a time duration provided by the argument.
 *
 * \param time amount of time to turn.
 * \param direction the direction to turn.
 * \param speed motor speed ratio.
 * \return true when the requested heading has been reached.
*/
bool DriveTrain::Turn(double time, Direction direction, float speed) {
	// Abort if robot drive is not available
	if (robot_drive_ == NULL || timer_ == NULL) {
		return true;
	}
	
	double time_left = 0.0;
	float directional_speed = 0.0;
	double elapsed_time = 999.0;

	// Get the timer value since we started moving
	elapsed_time = timer_->Get();
	
	// Calculate time left to move
	time_left = time - elapsed_time;
	
	// Check to see if we've reached the proper position
	if ((time_left < time_threshold_) || (time_left < 0)) {
		//robot_drive_->Drive(0.0, 0.0);
		robot_drive_->ArcadeDrive(0.0, 0.0, false);
		timer_->Stop();
		return true;
	}
	// Continue moving
	else {		
		if (direction == kLeft) {
			directional_speed = left_direction_;
		}
		else {
			directional_speed = right_direction_;
		}
		
		if (time_left > auto_far_time_threshold_){
			directional_speed = directional_speed * speed * auto_far_turning_speed_ratio_;
		}
		else if (time_left > auto_medium_time_threshold_) {
			directional_speed = directional_speed * speed * auto_medium_turning_speed_ratio_;
		}
		else {
			directional_speed = directional_speed * speed * auto_near_turning_speed_ratio_;
		}
		
		//robot_drive_->Drive(0.0, directional_speed);
		robot_drive_->ArcadeDrive(0.0, directional_speed, false);
		return false;
	}
}

/**
 * \brief Returns the current heading of the robot.
 *
 * \return the current robot heading in degrees.
*/
float DriveTrain::GetHeading() {
	return gyro_angle_;
}
