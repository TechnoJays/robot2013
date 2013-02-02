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
	SafeDelete(log_);
	SafeDelete(parameters_);
	SafeDelete(robot_drive_);
	SafeDelete(left_controller_);
	SafeDelete(right_controller_);
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

	// Initialize private member objects
	robot_drive_ = NULL;
	left_controller_ = NULL;
	right_controller_ = NULL;
	log_ = NULL;
	parameters_ = NULL;
	
	// Initialize private parameters
	invert_multiplier_ = 0.0;
	normal_linear_speed_ratio_ = 1.0;
	turbo_linear_speed_ratio_ = 1.0;
	normal_turning_speed_ratio_ = 1.0;
	turbo_turning_speed_ratio_ = 1.0;
	forward_direction_ = 1.0;
	backward_direction_ = -1.0;
	left_direction_ = -1.0;
	right_direction_ = 1.0;
	left_motor_inverted_ = 0;
	right_motor_inverted_ = 0;
	maximum_linear_speed_change_ = 0.0;
	maximum_turn_speed_change_ = 0.0;
	linear_filter_constant_ = 0.0;
	turn_filter_constant_ = 0.0;
	
	// Initialize private member variables
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
	float motor_safety_timeout = 2.0;
	bool parameters_read = false;	// This should default to false

	
	// Close and delete old objects
	SafeDelete(parameters_);
	SafeDelete(robot_drive_);
	SafeDelete(left_controller_);
	SafeDelete(right_controller_);
	
	// Attempt to read the parameters file
	// FIXME
	//parameters_ = new Parameters(parameters_file_);
	parameters_ = new Parameters("drivetrain.par");
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
		parameters_->GetValue("INVERT_CONTROLS", &invert_controls);		
		parameters_->GetValue("FORWARD_DIRECTION", &forward_direction_);
		parameters_->GetValue("BACKWARD_DIRECTION", &backward_direction_);
		parameters_->GetValue("LEFT_DIRECTION", &left_direction_);
		parameters_->GetValue("RIGHT_DIRECTION", &right_direction_);
		parameters_->GetValue("NORMAL_LINEAR_SPEED_RATIO", &normal_linear_speed_ratio_);
		parameters_->GetValue("TURBO_LINEAR_SPEED_RATIO", &turbo_linear_speed_ratio_);
		parameters_->GetValue("NORMAL_TURNING_SPEED_RATIO", &normal_turning_speed_ratio_);
		parameters_->GetValue("TURBO_TURNING_SPEED_RATIO", &turbo_turning_speed_ratio_);
		parameters_->GetValue("MAXIMUM_LINEAR_SPEED_CHANGE", &maximum_linear_speed_change_);
		parameters_->GetValue("MAXIMUM_TURN_SPEED_CHANGE", &maximum_turn_speed_change_);
		parameters_->GetValue("LINEAR_FILTER_CONSTANT", &linear_filter_constant_);
		parameters_->GetValue("TURN_FILTER_CONSTANT", &turn_filter_constant_);
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
		// FIXME
		//robot_drive_->SetSafetyEnabled(true);
		robot_drive_->SetSafetyEnabled(false);
	}
	
	// Invert motors if specified
	if (left_motor_inverted_ && robot_drive_ != NULL) {
		robot_drive_->SetInvertedMotor(RobotDrive::kRearLeftMotor, true);
	}
	if (right_motor_inverted_ && robot_drive_ != NULL) {
		robot_drive_->SetInvertedMotor(RobotDrive::kRearRightMotor, true);
	}
	
	if (log_enabled_) {
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
 * \brief Set the current state of the robot and perform any actions necessary during mode changes.
 *
 * \param state current robot state.
*/
void DriveTrain::SetRobotState(ProgramState state) {
	robot_state_ = state;

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
