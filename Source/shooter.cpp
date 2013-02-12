#include <math.h>
#include "WPILib.h"
#include "shooter.h"
#include "datalog.h"
#include "parameters.h"

/**
 * \brief Create and initialize a shooter.
 *
 * Use the default parameter file "shooter.par" and logging is disabled.
*/
Shooter::Shooter() {
	Initialize("shooter.par", false);
}

/**
 * \brief Create and initialize a shooter.
 *
 * Use the default parameter file "shooter.par" and enable/disable
 * logging based on the parameter.
 *
 * \param logging_enabled true if logging is enabled.
*/
Shooter::Shooter(bool logging_enabled) {
	Initialize("shooter.par", logging_enabled);
}

/**
 * \brief Create and initialize a shooter.
 *
 * Use the user specified parameter file and logging is disabled.
 *
 * \param parameters shooter parameter file path and name.
*/
Shooter::Shooter(char * parameters) {
	Initialize(parameters, false);
}

/**
 * \brief Create and initialize a shooter.
 *
 * Use the user specified parameter file and enable/disable
 * logging based on the parameter.
 *
 * \param parameters shooter parameter file path and name.
 * \param logging_enabled true if logging is enabled.
*/
Shooter::Shooter(char * parameters, bool logging_enabled) {
	Initialize(parameters, logging_enabled);
}

/**
 * \brief Delete and clear all objects and pointers.
*/
Shooter::~Shooter() {
	if (log_ != NULL) {
		log_->Close();
	}
	SafeDelete(shooter_controller_);
	SafeDelete(pitch_controller_);
	SafeDelete(encoder_);
	SafeDelete(timer_);
	SafeDelete(log_);
	SafeDelete(parameters_);
}

/**
 * \brief Initialize the Shooter object.
 *
 * Create member objects, initialize default values, read parameters from the param file.
 *
 * \param parameters shooter parameter file path and name.
 * \param logging_enabled true if logging is enabled.
*/
void Shooter::Initialize(char * parameters, bool logging_enabled) {
	// Initialize public member variables
	encoder_enabled_ = false;
	shooter_enabled_ = false;
	pitch_enabled_ = false;

	// Initialize private member objects
	shooter_controller_ = NULL;
	pitch_controller_ = NULL;
	encoder_ = NULL;
	timer_ = NULL;
	log_ = NULL;
	parameters_ = NULL;

	// Initialize private parameters
	invert_multiplier_ = 0.0;
	shooter_normal_speed_ratio_ = 1.0;
	pitch_normal_speed_ratio_ = 1.0;
	pitch_turbo_speed_ratio_ = 1.0;
	auto_far_speed_ratio_ = 1.0;
	auto_medium_speed_ratio_ = 1.0;
	auto_near_speed_ratio_ = 1.0;	
	auto_medium_encoder_threshold_ = 50;
	auto_far_encoder_threshold_ = 100;
	auto_medium_time_threshold_ = 0.5;
	auto_far_time_threshold_ = 1.0;
	encoder_threshold_ = 10;
	pitch_up_direction_ = 1.0;
	pitch_down_direction_ = -1.0;
	shoot_forward_direction_ = 1.0;
	shoot_backward_direction_ = -1.0;
	time_threshold_ = 0.1;
	encoder_max_limit_ = -1;
	encoder_min_limit_ = -1;
	shooter_min_power_speed_ = 0.4;
	shooter_power_adjustment_ratio_ = 0.006;

	// Initialize private member variables
	encoder_count_ = 0;
	log_enabled_ = false;
	robot_state_ = kDisabled;

	// Create a new data log object
	log_ = new DataLog("shooter.log");

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
bool Shooter::LoadParameters() {
	// Define and initialize local variables
	int pitch_motor_slot = -1;
	int pitch_motor_channel = -1;
	int shooter_motor_slot = -1;
	int shooter_motor_channel = -1;
	int encoder_a_slot = -1;
	int encoder_a_channel = -1;
	int encoder_b_slot = -1;
	int encoder_b_channel = -1;
	int encoder_reverse = 0;
	int encoder_type = 2;
	int invert_controls = 0;
	float motor_safety_timeout = 2.0;
	bool parameters_read = false;	// This should default to false

	
	// Close and delete old objects
	SafeDelete(parameters_);
	SafeDelete(encoder_);
	SafeDelete(pitch_controller_);
	SafeDelete(shooter_controller_);
	
	// Attempt to read the parameters file
	parameters_ = new Parameters(parameters_file_);
	if (parameters_ != NULL && parameters_->file_opened_) {
		parameters_read = parameters_->ReadValues();
		parameters_->Close();
	}
	
	if (log_enabled_) {
		if (parameters_read)
			log_->WriteLine("Shooter parameters loaded successfully\n");
		else
			log_->WriteLine("Shooter parameters failed to read\n");
	}
	
	// Set shooter variables based on the parameters file
	if (parameters_read) {
		parameters_->GetValue("SHOOTER_MOTOR_SLOT", &shooter_motor_slot);
		parameters_->GetValue("SHOOTER_MOTOR_CHANNEL", &shooter_motor_channel);
		parameters_->GetValue("PITCH_MOTOR_SLOT", &pitch_motor_slot);
		parameters_->GetValue("PITCH_MOTOR_CHANNEL", &pitch_motor_channel);		
		parameters_->GetValue("ENCODER_A_SLOT", &encoder_a_slot);
		parameters_->GetValue("ENCODER_A_CHANNEL", &encoder_a_channel);
		parameters_->GetValue("ENCODER_B_SLOT", &encoder_b_slot);
		parameters_->GetValue("ENCODER_B_CHANNEL", &encoder_b_channel);
		parameters_->GetValue("ENCODER_REVERSE", &encoder_reverse);
		parameters_->GetValue("ENCODER_TYPE", &encoder_type);
		parameters_->GetValue("ENCODER_THRESHOLD", &encoder_threshold_);
		parameters_->GetValue("MOTOR_SAFETY_TIMEOUT", &motor_safety_timeout);
		parameters_->GetValue("INVERT_CONTROLS", &invert_controls);		
		parameters_->GetValue("PITCH_UP_DIRECTION", &pitch_up_direction_);
		parameters_->GetValue("PITCH_DOWN_DIRECTION", &pitch_down_direction_);
		parameters_->GetValue("PITCH_NORMAL_SPEED_RATIO", &pitch_normal_speed_ratio_);
		parameters_->GetValue("PITCH_TURBO_SPEED_RATIO", &pitch_turbo_speed_ratio_);
		parameters_->GetValue("SHOOTER_NORMAL_SPEED_RATIO", &shooter_normal_speed_ratio_);
		parameters_->GetValue("AUTO_FAR_SPEED_RATIO", &auto_far_speed_ratio_);
		parameters_->GetValue("AUTO_MEDIUM_SPEED_RATIO", &auto_medium_speed_ratio_);
		parameters_->GetValue("AUTO_NEAR_SPEED_RATIO", &auto_near_speed_ratio_);
		parameters_->GetValue("TIME_THRESHOLD", &time_threshold_);
		parameters_->GetValue("AUTO_MEDIUM_ENCODER_THRESHOLD", &auto_medium_encoder_threshold_);
		parameters_->GetValue("AUTO_FAR_ENCODER_THRESHOLD", &auto_far_encoder_threshold_);
		parameters_->GetValue("AUTO_MEDIUM_TIME_THRESHOLD", &auto_medium_time_threshold_);
		parameters_->GetValue("AUTO_FAR_TIME_THRESHOLD", &auto_far_time_threshold_);
		parameters_->GetValue("ENCODER_MAX_LIMIT", &encoder_max_limit_);
		parameters_->GetValue("ENCODER_MIN_LIMIT", &encoder_min_limit_);
		parameters_->GetValue("SHOOT_FORWARD_DIRECTION", &shoot_forward_direction_);
		parameters_->GetValue("SHOOT_BACKWARD_DIRECTION", &shoot_backward_direction_);
		parameters_->GetValue("SHOOTER_MIN_POWER_SPEED", &shooter_min_power_speed_);
		parameters_->GetValue("SHOOTER_POWER_ADJUSTMENT_RATIO", &shooter_power_adjustment_ratio_);
		parameters_->GetValue("ANGLE_LINEAR_FIT_GRADIENT", &angle_linear_fit_gradient_);
		parameters_->GetValue("ANGLE_LINEAR_FIT_CONSTANT", &angle_linear_fit_constant_);
	}

	// Check if the encoder is present/enabled
	if (encoder_a_slot > 0 && encoder_a_channel > 0 && encoder_b_slot > 0 && encoder_b_channel > 0) {
		encoder_ = new Encoder(encoder_a_slot, encoder_a_channel, encoder_b_slot, encoder_b_channel, encoder_reverse, (CounterBase::EncodingType) encoder_type);
		if (encoder_ != NULL) {
			encoder_enabled_ = true;
			encoder_->Start();
		}
	}
	else {
		encoder_enabled_ = false;
	}
	
	// Check if the pitch motor is present/enabled
	if (pitch_motor_slot > 0 && pitch_motor_channel > 0) {
		pitch_controller_ = new Jaguar(pitch_motor_slot, pitch_motor_channel);
		if (pitch_controller_ != NULL) {
			pitch_controller_->SetExpiration(motor_safety_timeout);
			pitch_controller_->SetSafetyEnabled(true);
			pitch_enabled_ = true;
		}
	}
	else {
		pitch_enabled_ = false;
	}

	// Check if the shooter motor is present/enabled
	if (shooter_motor_slot > 0 && shooter_motor_channel > 0) {
		shooter_controller_ = new Jaguar(shooter_motor_slot, shooter_motor_channel);
		if (shooter_controller_ != NULL) {
			shooter_controller_->SetExpiration(motor_safety_timeout);
			shooter_controller_->SetSafetyEnabled(true);
			shooter_enabled_ = true;
		}
	}
	else {
		shooter_enabled_ = false;
	}
	
	if (log_enabled_) {
		if (encoder_enabled_) {
			log_->WriteLine("Pitch encoder enabled\n");
		}
		else {
			log_->WriteLine("Pitch encoder disabled\n");
		}
		if (pitch_enabled_) {
			log_->WriteLine("Pitch motor enabled\n");
		}
		else {
			log_->WriteLine("Pitch motor disabled\n");
		}
		if (shooter_enabled_) {
			log_->WriteLine("Shooter motor enabled\n");
		}
		else {
			log_->WriteLine("Shooter motor disabled\n");
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
void Shooter::ReadSensors() {
	if (encoder_enabled_) {
		encoder_count_ = encoder_->Get();
	}
}

/**
 * \brief Resets and restarts the timer for time based movement.
*/
void Shooter::ResetAndStartTimer() {
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
void Shooter::SetRobotState(ProgramState state) {
	robot_state_ = state;
	
	if (timer_ != NULL) {
		timer_->Stop();
	}
	
	if (state == kDisabled) {
		if (pitch_enabled_)
			pitch_controller_->SetSafetyEnabled(true);
		if (shooter_enabled_)
			shooter_controller_->SetSafetyEnabled(true);
	}
	else if (state == kTeleop) {
		if (pitch_enabled_)
			pitch_controller_->SetSafetyEnabled(true);
		if (shooter_enabled_)
			shooter_controller_->SetSafetyEnabled(true);
	}
	else if (state == kAutonomous) {
		if (pitch_enabled_)
			pitch_controller_->SetSafetyEnabled(false);
		if (shooter_enabled_)
			shooter_controller_->SetSafetyEnabled(false);
	}
}

/**
 * \brief Return a string containing sensor and status variables.
 *
 * \param output_buffer empty character array to contain the string.
*/
void Shooter::GetCurrentState(char * output_buffer) {
	if (encoder_enabled_)
		sprintf(output_buffer, "%6i", encoder_count_);
}

/**
 * \brief Log sensor and status variables when requested.
*/
void Shooter::LogCurrentState() {
	if (log_ != NULL) {
		if (encoder_enabled_)
			log_->WriteValue("Encoder count", encoder_count_, true);
	}
}

/**
 * \brief Enable or disable logging for this object.
 *
 * \param state true if logging should be enabled.
*/
void Shooter::SetLogState(bool state) {
	if (state && log_ != NULL)
		log_enabled_ = true;
	else
		log_enabled_ = false;
}

/**
 * \brief Sets the shooter pitch to a position provided by the argument.
 *
 * \param encoder_count desired position in encoder counts.
 * \param speed motor speed ratio.
 * \return true when the desired position is reached.
*/
bool Shooter::SetPitch(int encoder_count, float speed) {
	// Abort if the pitch control or encoder are not available
	if (!encoder_enabled_ || !pitch_enabled_)
		return true;

	// Movement direction/speed
	float movement_direction = 0.0;
	
	// Check the encoder position against the boundaries if boundaries enabled
	// Check Max limit
	if (encoder_max_limit_ > 0 && (encoder_count > encoder_count_)) {
		if (encoder_count_ > encoder_max_limit_) {
			return true;
		}
	}
	// Check Min limit
	if (encoder_min_limit_ > 0 && (encoder_count < encoder_count_)) {
		if (encoder_count_ < encoder_min_limit_) {
			return true;
		}
	}

	// Check to see if we've reached the proper height
	if (abs(encoder_count - encoder_count_) <= encoder_threshold_) {
		pitch_controller_->Set(0, 0);
		return true;
	}
	// Continue moving
	else {
		// Calculate the direction needed to move, and turn into a speed
		if ((encoder_count - encoder_count_) > 0) {
			if (abs(encoder_count - encoder_count_) > auto_far_encoder_threshold_){
				movement_direction = pitch_up_direction_ * speed * auto_far_speed_ratio_;
			}
			else if (abs(encoder_count - encoder_count_) > auto_medium_encoder_threshold_) {
				movement_direction = pitch_up_direction_ * speed * auto_medium_speed_ratio_;
			}
			else {
				movement_direction = pitch_up_direction_ * speed * auto_near_speed_ratio_;
			}
		}
		else {
			if (abs(encoder_count - encoder_count_) > auto_far_encoder_threshold_){
				movement_direction = pitch_down_direction_ * speed * auto_far_speed_ratio_;
			}
			else if (abs(encoder_count - encoder_count_) > auto_medium_encoder_threshold_) {
				movement_direction = pitch_down_direction_ * speed * auto_medium_speed_ratio_;
			}
			else {
				movement_direction = pitch_down_direction_ * speed * auto_near_speed_ratio_;
			}
		}
		
		// Move
		pitch_controller_->Set(movement_direction, 0);
		return false;
	}
}

/**
 * \brief Sets the shooter pitch to a position provided by the argument.
 *
 * Reset and start the timer before calling this function.
 *
 * \param time amount of time to move the pitch.
 * \param direction the direction to move the pitch.
 * \param speed motor speed ratio.
 * \return true when the desired position is reached.
*/
bool Shooter::SetPitch(double time, Direction direction, float speed) {
	// Abort if pitch not available
	if (!pitch_enabled_ || timer_ == NULL)
		return true;

	double time_left = 0.0;
	float directional_speed = 0.0;
	double elapsed_time = 999.0;

	// Get the timer value since we started moving
	elapsed_time = timer_->Get();
		
	// Calculate time left to move
	time_left = time - elapsed_time;

	if (encoder_enabled_) {
		// Check the encoder position against the boundaries if boundaries enabled
		// Check Max limit
		if (encoder_max_limit_ > 0 && direction == kUp) {
			if (encoder_count_ > encoder_max_limit_) {
				return true;
			}
		}
		// Check Min limit
		if (encoder_min_limit_ > 0 && direction == kDown) {
			if (encoder_count_ < encoder_min_limit_) {
				return true;
			}
		}
	}

	// Check to see if we've reached the proper height
	if ((time_left < time_threshold_) || (time_left < 0)) {
		pitch_controller_->Set(0, 0);
		timer_->Stop();
		return true;
	}
	// Continue moving
	else {		
		if (direction == kUp) {
			directional_speed = pitch_up_direction_;
		}
		else {
			directional_speed = pitch_down_direction_;
		}
		
		if (time_left > auto_far_time_threshold_){
			directional_speed = directional_speed * speed * auto_far_speed_ratio_;
		}
		else if (time_left > auto_medium_time_threshold_) {
			directional_speed = directional_speed * speed * auto_medium_speed_ratio_;
		}
		else {
			directional_speed = directional_speed * speed * auto_near_speed_ratio_;
		}
		
		pitch_controller_->Set(directional_speed, 0);
		return false;
	}
}

/**
 * \brief Sets the shooter pitch to an angle provided by the argument.
 *
 * \param angle desired angle in degrees.
 * \param speed motor speed ratio.
 * \return true when the desired angle is reached.
*/
bool Shooter::SetPitchAngle(float angle, float speed) {
	// Abort if the pitch control or encoder are not available
	if (!encoder_enabled_ || !pitch_enabled_)
		return true;

	// Movement direction/speed
	float movement_direction = 0.0;
	
	// Convert angle to encoder position
	int encoder_count = (int) floor((angle_linear_fit_gradient_ * angle) + angle_linear_fit_constant_);
	
	// Check the encoder position against the boundaries if boundaries enabled
	// Check Max limit
	if (encoder_max_limit_ > 0 && (encoder_count > encoder_count_)) {
		if (encoder_count_ > encoder_max_limit_) {
			return true;
		}
	}
	// Check Min limit
	if (encoder_min_limit_ > 0 && (encoder_count < encoder_count_)) {
		if (encoder_count_ < encoder_min_limit_) {
			return true;
		}
	}

	// Check to see if we've reached the proper height
	if (abs(encoder_count - encoder_count_) <= encoder_threshold_) {
		pitch_controller_->Set(0, 0);
		return true;
	}
	// Continue moving
	else {
		// Calculate the direction needed to move, and turn into a speed
		if ((encoder_count - encoder_count_) > 0) {
			if (abs(encoder_count - encoder_count_) > auto_far_encoder_threshold_){
				movement_direction = pitch_up_direction_ * speed * auto_far_speed_ratio_;
			}
			else if (abs(encoder_count - encoder_count_) > auto_medium_encoder_threshold_) {
				movement_direction = pitch_up_direction_ * speed * auto_medium_speed_ratio_;
			}
			else {
				movement_direction = pitch_up_direction_ * speed * auto_near_speed_ratio_;
			}
		}
		else {
			if (abs(encoder_count - encoder_count_) > auto_far_encoder_threshold_){
				movement_direction = pitch_down_direction_ * speed * auto_far_speed_ratio_;
			}
			else if (abs(encoder_count - encoder_count_) > auto_medium_encoder_threshold_) {
				movement_direction = pitch_down_direction_ * speed * auto_medium_speed_ratio_;
			}
			else {
				movement_direction = pitch_down_direction_ * speed * auto_near_speed_ratio_;
			}
		}
		
		// Move
		pitch_controller_->Set(movement_direction, 0);
		return false;
	}
}

/**
 * \brief Moves the shooter pitch until commanded otherwise.
 *
 * \param directional_speed the speed and direction to move the pitch.
 * \param turbo true if the pitch should move at 'turbo' speed.
*/
void Shooter::MovePitch(float directional_speed, bool turbo) {
	// Abort if pitch control not available
	if (!pitch_enabled_)
		return;
	
	directional_speed = directional_speed * invert_multiplier_;
	// Set the encoder couting to match movement direction
	if (encoder_enabled_) {
		// Check the encoder position against the boundaries if boundaries enabled
		// Check Max limit
		if (encoder_max_limit_ > 0 && ((directional_speed * pitch_up_direction_) > 0)) {
			if (encoder_count_ > encoder_max_limit_) {
				directional_speed = 0.0;
			}
		}
		// Check Min limit
		if (encoder_min_limit_ > 0 && ((directional_speed * pitch_down_direction_) > 0)) {
			if (encoder_count_ < encoder_min_limit_) {
				directional_speed = 0.0;
			}
		}
	}

	if (turbo) {
		directional_speed = directional_speed * pitch_turbo_speed_ratio_;
	}
	else {
		directional_speed = directional_speed * pitch_normal_speed_ratio_;
	}

	// Set the controller speed
	pitch_controller_->Set(directional_speed, 0);
}

/**
 * \brief Power the shooting mechanism with the given power percentage.
 *
 * The power is converted into a motor speed and applied to the motor,
 * using conversion values specified in the parameter file.
 *
 * \param power_as_percent the amount of power/speed to shoot with.
*/
void Shooter::Shoot(int power_as_percent) {
	// Abort if shooter not available
	if (!shooter_enabled_)
		return;
	
	float shooting_power_as_speed = 0.0;
	
	if (power_as_percent == 0) {
		shooting_power_as_speed = 0.0;
	}
	else if (power_as_percent > 0) {
		shooting_power_as_speed = ((((float) power_as_percent) * shooter_power_adjustment_ratio_) + shooter_min_power_speed_) * shooter_normal_speed_ratio_;
	}
	else {
		shooting_power_as_speed = ((((float) power_as_percent) * shooter_power_adjustment_ratio_) - shooter_min_power_speed_) * shooter_normal_speed_ratio_;
	}

	// Set the controller speed
	shooter_controller_->Set(shooting_power_as_speed, 0);
}

/**
 * \brief Power the shooting mechanism with the given power percentage for a certain amount of time.
 *
 * The power is converted into a motor speed and applied to the motor,
 * using conversion values specified in the parameter file.
 *
 * \param time amount of time to move the shooter.
 * \param power_as_percent the amount of power/speed to shoot with.
 * \return true when the desired time is reached.
*/
bool Shooter::Shoot(double time, int power_as_percent) {
	// Abort if shooter not available
	if (!shooter_enabled_ || timer_ == NULL)
		return true;

	double time_left = 0.0;
	double elapsed_time = 999.0;

	// Get the timer value since we started moving
	elapsed_time = timer_->Get();

	// Calculate time left to move
	time_left = time - elapsed_time;
	
	// Check to see if we've reached the proper time
	if ((time_left < time_threshold_) || (time_left < 0)) {
		shooter_controller_->Set(0, 0);
		timer_->Stop();
		return true;
	}
	// Continue
	else {		
		float shooting_power_as_speed = 0.0;

		if (power_as_percent == 0) {
			shooting_power_as_speed = 0.0;
		}
		else if (power_as_percent > 0) {
			shooting_power_as_speed = ((((float) power_as_percent) * shooter_power_adjustment_ratio_) + shooter_min_power_speed_) * shooter_normal_speed_ratio_;
		}
		else {
			shooting_power_as_speed = ((((float) power_as_percent) * shooter_power_adjustment_ratio_) - shooter_min_power_speed_) * shooter_normal_speed_ratio_;
		}
				
		shooter_controller_->Set(shooting_power_as_speed, 0);
		return false;
	}	
}
