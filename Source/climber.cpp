#include <cmath>
#include "WPILib.h"
#include "climber.h"
#include "datalog.h"
#include "parameters.h"

/**
 * \brief Create and initialize a climber.
 *
 * Use the default parameter file "climber.par" and logging is disabled.
*/
Climber::Climber() {
	Initialize("climber.par", false);
}

/**
 * \brief Create and initialize a climber.
 *
 * Use the default parameter file "climber.par" and enable/disable
 * logging based on the parameter.
 *
 * \param logging_enabled true if logging is enabled.
*/
Climber::Climber(bool logging_enabled) {
	Initialize("climber.par", logging_enabled);
}

/**
 * \brief Create and initialize a climber.
 *
 * Use the user specified parameter file and logging is disabled.
 *
 * \param parameters climber parameter file path and name.
*/
Climber::Climber(char * parameters) {
	Initialize(parameters, false);
}

/**
 * \brief Create and initialize a climber.
 *
 * Use the user specified parameter file and enable/disable
 * logging based on the parameter.
 *
 * \param parameters climber parameter file path and name.
 * \param logging_enabled true if logging is enabled.
*/
Climber::Climber(char * parameters, bool logging_enabled) {
	Initialize(parameters, logging_enabled);
}

/**
 * \brief Delete and clear all objects and pointers.
*/
Climber::~Climber() {
	if (log_ != NULL) {
		log_->Close();
	}
	SafeDelete(controller_);
	SafeDelete(encoder_);
	SafeDelete(timer_);
	SafeDelete(log_);
	SafeDelete(parameters_);
}

/**
 * \brief Initialize the Climber object.
 *
 * Create member objects, initialize default values, read parameters from the param file.
 *
 * \param parameters climber parameter file path and name.
 * \param logging_enabled true if logging is enabled.
*/
void Climber::Initialize(char * parameters, bool logging_enabled) {
	// Initialize public member variables
	encoder_enabled_ = false;
	climber_enabled_ = false;

	// Initialize private member objects
	controller_ = NULL;
	encoder_ = NULL;
	timer_ = NULL;
	log_ = NULL;
	parameters_ = NULL;
	
	// Initialize private parameters
	invert_multiplier_ = 0.0;
	normal_up_speed_ratio_ = 1.0;
	normal_down_speed_ratio_ = 1.0;
	turbo_up_speed_ratio_ = 1.0;
	turbo_down_speed_ratio_ = 1.0;
	auto_far_speed_ratio_ = 1.0;
	auto_medium_speed_ratio_ = 1.0;
	auto_near_speed_ratio_ = 1.0;
	auto_medium_encoder_threshold_ = 50;
	auto_far_encoder_threshold_ = 100;
	auto_medium_time_threshold_ = 0.5;
	auto_far_time_threshold_ = 1.0;
	encoder_threshold_ = 10;
	up_direction_ = 1.0;
	down_direction_ = -1.0;
	time_threshold_ = 0.1;
	encoder_max_limit_ = -1;
	encoder_min_limit_ = -1;

	// Initialize private member variables
	encoder_count_ = 0;
	log_enabled_ = false;
	robot_state_ = kDisabled;
	
	// Create a new data log object
	log_ = new DataLog("climber.log");

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
bool Climber::LoadParameters() {
	// Define and initialize local variables
	int motor_slot = -1;
	int motor_channel = -1;
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
	SafeDelete(controller_);
	
	// Attempt to read the parameters file
	parameters_ = new Parameters(parameters_file_);
	if (parameters_ != NULL && parameters_->file_opened_) {
		parameters_read = parameters_->ReadValues();
		parameters_->Close();
	}
	
	if (log_enabled_) {
		if (parameters_read)
			log_->WriteLine("Climber parameters loaded successfully\n");
		else
			log_->WriteLine("Climber parameters failed to read\n");
	}

	// Set climber variables based on the parameters file
	if (parameters_read) {
		parameters_->GetValue("MOTOR_SLOT", &motor_slot);
		parameters_->GetValue("MOTOR_CHANNEL", &motor_channel);
		parameters_->GetValue("ENCODER_A_SLOT", &encoder_a_slot);
		parameters_->GetValue("ENCODER_A_CHANNEL", &encoder_a_channel);
		parameters_->GetValue("ENCODER_B_SLOT", &encoder_b_slot);
		parameters_->GetValue("ENCODER_B_CHANNEL", &encoder_b_channel);
		parameters_->GetValue("ENCODER_REVERSE", &encoder_reverse);
		parameters_->GetValue("ENCODER_TYPE", &encoder_type);
		parameters_->GetValue("ENCODER_THRESHOLD", &encoder_threshold_);
		parameters_->GetValue("MOTOR_SAFETY_TIMEOUT", &motor_safety_timeout);
		parameters_->GetValue("INVERT_CONTROLS", &invert_controls);		
		parameters_->GetValue("NORMAL_UP_SPEED_RATIO", &normal_up_speed_ratio_);
		parameters_->GetValue("NORMAL_DOWN_SPEED_RATIO", &normal_down_speed_ratio_);
		parameters_->GetValue("TURBO_UP_SPEED_RATIO", &turbo_up_speed_ratio_);
		parameters_->GetValue("TURBO_DOWN_SPEED_RATIO", &turbo_down_speed_ratio_);
		parameters_->GetValue("AUTO_FAR_SPEED_RATIO", &auto_far_speed_ratio_);
		parameters_->GetValue("AUTO_MEDIUM_SPEED_RATIO", &auto_medium_speed_ratio_);
		parameters_->GetValue("AUTO_NEAR_SPEED_RATIO", &auto_near_speed_ratio_);
		parameters_->GetValue("UP_DIRECTION", &up_direction_);
		parameters_->GetValue("DOWN_DIRECTION", &down_direction_);
		parameters_->GetValue("TIME_THRESHOLD", &time_threshold_);
		parameters_->GetValue("ENCODER_MAX_LIMIT", &encoder_max_limit_);
		parameters_->GetValue("ENCODER_MIN_LIMIT", &encoder_min_limit_);
		parameters_->GetValue("AUTO_MEDIUM_ENCODER_THRESHOLD", &auto_medium_encoder_threshold_);
		parameters_->GetValue("AUTO_FAR_ENCODER_THRESHOLD", &auto_far_encoder_threshold_);
		parameters_->GetValue("AUTO_MEDIUM_TIME_THRESHOLD", &auto_medium_time_threshold_);
		parameters_->GetValue("AUTO_FAR_TIME_THRESHOLD", &auto_far_time_threshold_);
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
		
	// Check if the motor is present/enabled
	if (motor_slot > 0 && motor_channel > 0) {
		controller_ = new Jaguar(motor_slot, motor_channel);
		if (controller_ != NULL) {
			controller_->SetExpiration(motor_safety_timeout);
			controller_->SetSafetyEnabled(true);
			climber_enabled_ = true;
		}
	}
	else {
		climber_enabled_ = false;
	}
	
	if (log_enabled_) {
		if (encoder_enabled_) {
			log_->WriteLine("Climber encoder enabled\n");
		}
		else {
			log_->WriteLine("Climber encoder disabled\n");
		}
		if (climber_enabled_) {
			log_->WriteLine("Climber motor enabled\n");
		}
		else {
			log_->WriteLine("Climber motor disabled\n");
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
void Climber::ReadSensors() {
	if (encoder_enabled_) {
		encoder_count_ = encoder_->Get();
	}
}

/**
 * \brief Resets and restarts the timer for time based movement.
*/
void Climber::ResetAndStartTimer() {
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
void Climber::SetRobotState(ProgramState state) {
	robot_state_ = state;
	
	if (timer_ != NULL) {
		timer_->Stop();
	}
	
	if (state == kDisabled) {
		if (climber_enabled_)
			controller_->SetSafetyEnabled(true);
	}
	else if (state == kTeleop) {
		if (climber_enabled_)
			controller_->SetSafetyEnabled(true);
	}
	else if (state == kAutonomous) {
		if (climber_enabled_)
			controller_->SetSafetyEnabled(false);
	}
}

/**
 * \brief Return a string containing sensor and status variables.
 *
 * \param output_buffer empty character array to contain the string.
*/
void Climber::GetCurrentState(char * output_buffer) {
	if (encoder_enabled_)
		sprintf(output_buffer, "%6i", encoder_count_);
}

/**
 * \brief Log sensor and status variables when requested.
*/
void Climber::LogCurrentState() {
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
void Climber::SetLogState(bool state) {
	if (state && log_ != NULL)
		log_enabled_ = true;
	else
		log_enabled_ = false;
}

/**
 * \brief Sets the robot climber to a position provided by the argument.
 *
 * \param encoder_count desired position in encoder counts.
 * \param speed motor speed ratio.
 * \return true when the desired position is reached.
*/
bool Climber::Set(int encoder_count, float speed) {
	// Abort if the climber or encoder are not available
	if (!encoder_enabled_ || !climber_enabled_)
		return true;

	// Movement direction/speed
	float movement_direction = 0.0;
	
	// Check the encoder position against the boundaries if boundaries enabled
	// Check Max limit
	if (encoder_max_limit_ > 0 && (encoder_count > encoder_count_)) {
		if (encoder_count_ > encoder_max_limit_) {
			controller_->Set(0, 0);
			return true;
		}
	}
	// Check Min limit
	if (encoder_min_limit_ > 0 && (encoder_count < encoder_count_)) {
		if (encoder_count_ < encoder_min_limit_) {
			controller_->Set(0, 0);
			return true;
		}
	}

	// Check to see if we've reached the proper height
	if (abs(encoder_count - encoder_count_) <= encoder_threshold_) {
		controller_->Set(0, 0);
		return true;
	}
	// Continue moving the climber
	else {
		// Calculate the direction needed to move, and turn into a speed
		if ((encoder_count - encoder_count_) > 0) {
			if (abs(encoder_count - encoder_count_) > auto_far_encoder_threshold_){
				movement_direction = up_direction_ * speed * auto_far_speed_ratio_;
			}
			else if (abs(encoder_count - encoder_count_) > auto_medium_encoder_threshold_) {
				movement_direction = up_direction_ * speed * auto_medium_speed_ratio_;
			}
			else {
				movement_direction = up_direction_ * speed * auto_near_speed_ratio_;
			}
		}
		else {
			if (abs(encoder_count - encoder_count_) > auto_far_encoder_threshold_){
				movement_direction = down_direction_ * speed * auto_far_speed_ratio_;
			}
			else if (abs(encoder_count - encoder_count_) > auto_medium_encoder_threshold_) {
				movement_direction = down_direction_ * speed * auto_medium_speed_ratio_;
			}
			else {
				movement_direction = down_direction_ * speed * auto_near_speed_ratio_;
			}
		}
		
		// Move the climber
		controller_->Set(movement_direction, 0);
		return false;
	}
}

/**
 * \brief Sets the robot climber to a position provided by the argument.
 *
 * Reset and start the timer before calling this function.
 *
 * \param time amount of time to move the climber.
 * \param direction the direction to move the climber.
 * \param speed motor speed ratio.
 * \return true when the desired position is reached.
*/
bool Climber::Set(double time, Direction direction, float speed) {
	// Abort if climber not available
	if (!climber_enabled_ || timer_ == NULL)
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
				controller_->Set(0, 0);
				timer_->Stop();
				return true;
			}
		}
		// Check Min limit
		if (encoder_min_limit_ > 0 && direction == kDown) {
			if (encoder_count_ < encoder_min_limit_) {
				controller_->Set(0, 0);
				timer_->Stop();
				return true;
			}
		}
	}

	// Check to see if we've reached the proper height
	if ((time_left < time_threshold_) || (time_left < 0)) {
		controller_->Set(0, 0);
		timer_->Stop();
		return true;
	}
	// Continue moving the climber
	else {		
		if (direction == kUp) {
			directional_speed = up_direction_;
		}
		else {
			directional_speed = down_direction_;
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
		
		controller_->Set(directional_speed, 0);
		return false;
	}
}

/**
 * \brief Moves the climber until commanded otherwise.
 *
 * \param directional_speed the speed and direction to move the climber.
 * \param turbo true if the climber should move at 'turbo' speed.
*/
void Climber::Move(float directional_speed, bool turbo) {
	// Abort if climber not available
	if (!climber_enabled_)
		return;
	
	directional_speed = directional_speed * invert_multiplier_;
	
	// Set the encoder couting to match movement direction
	if (encoder_enabled_) {
		// Check the encoder position against the boundaries if boundaries enabled
		// Check Max limit
		if (encoder_max_limit_ > 0 && ((directional_speed * up_direction_) > 0)) {
			if (encoder_count_ > encoder_max_limit_) {
				directional_speed = 0.0;
			}
		}
		// Check Min limit
		if (encoder_min_limit_ > 0 && ((directional_speed * down_direction_) > 0)) {
			if (encoder_count_ < encoder_min_limit_) {
				directional_speed = 0.0;
			}
		}
	}
	
	if (turbo) {
		// Check if the requested direction is the same direction as Up
		if (directional_speed*up_direction_ >= 0.0f)
			directional_speed = directional_speed * turbo_up_speed_ratio_;
		else
			directional_speed = directional_speed * turbo_down_speed_ratio_;
	}
	else {
		// Check if the requested direction is the same direction as Up
		if (directional_speed*up_direction_ >= 0.0f)
			directional_speed = directional_speed * normal_up_speed_ratio_;
		else
			directional_speed = directional_speed * normal_down_speed_ratio_;
	}
	
	// Set the controller speed
	controller_->Set(directional_speed, 0);
}

