#include "WPILib.h"

#include "autoscript.h"
#include "climber.h"
#include "common.h"
#include "datalog.h"
#include "drivetrain.h"
#include "feeder.h"
#include "parameters.h"
#include "shooter.h"
#include "targeting.h"
#include "technojays.h"
#include "userinterface.h"


/**
 * \def GetMsecTime()
 * \brief A macro that returns the current processor time in milliseconds
 */
#define GetMsecTime()           (GetFPGATime()/1000)

/**
 * \brief Create and initialize the robot.
 *
 * Use the default parameter file "technojays.par" and logging is disabled.
*/
TechnoJays::TechnoJays() {
	Initialize("technojays.par", true);
}

/**
 * \brief Delete and clear all objects and pointers.
*/
TechnoJays::~TechnoJays() {
}

/**
 * \brief Initialize the robot.
 *
 * Create member objects, initialize default values, read parameters from the param file.
 *
 * \param parameters robot parameter file path and name.
 * \param logging_enabled true if logging is enabled.
*/
void TechnoJays::Initialize(const char * parameters, bool logging_enabled) {
	// Initialize public member variables

	// Initialize private member objects
	autoscript_ = NULL;
	climber_ = NULL;
	feeder_ = NULL;
	log_ = NULL;
	drive_train_ = NULL;
	parameters_ = NULL;
	shooter_ = NULL;
	targeting_ = NULL;
	timer_ = NULL;
	auto_shoot_timer_ = NULL;
	user_interface_ = NULL;
	current_target_ = ParticleAnalysisReport();
	current_target_.imageHeight = 0;
	current_target_.imageWidth = 0;

	// Initialize private parameters
	camera_boot_time_ = 30.0;
	initial_target_search_time_ = 1.5;
	auto_shooter_spinup_time_ = 1.5;
	auto_shooter_spindown_time_ = 0.5;
	auto_feeder_height_angle_ = 50.0;
	auto_climbing_angle_ = 20.0;
	period_ = 0.0;

	// Initialize private member variables
	log_enabled_ = false;
	detailed_logging_enabled_ = false;
	driver_turbo_ = false;
	scoring_turbo_ = false;
	current_command_complete_ = false;
	current_command_in_progress_ = false;
	autoscript_files_counter_ = 0;
	previous_scoring_dpad_y_ = 0.0;
	target_report_heading_ = 0.0;
	degrees_off_ = 0.0;
	current_target_vector_location_ = 0;
	auto_shoot_state_ = kFinished;
	aim_state_ = kFinished;
	auto_find_target_state_ = kFinished;
	auto_rapid_fire_state_ = kFinished;
	auto_cycle_target_state_ = kFinished;
	auto_feeder_height_state_ = kFinished;
	auto_climbing_prep_state_ = kFinished;
	
	// Disable the watchdog timer
	// Set this right away before we do anything else
	GetWatchdog().SetEnabled(false);

	// Create a new data log object
	log_ = new DataLog("technojays.log");

	// Enable logging if specified
	if (log_ != NULL && log_->file_opened_) {
		log_enabled_ = logging_enabled;
	} else {
		log_enabled_ = false;
	}

	// Create timer objects
	timer_ = new Timer();
	auto_shoot_timer_ = new Timer();

	// Attempt to read the parameters file
	strncpy(parameters_file_, parameters, sizeof(parameters_file_));

	LoadParameters();

	targeting_ = new Targeting("targeting.par", log_enabled_);
	autoscript_ = new AutoScript();
	climber_ = new Climber("climber.par", log_enabled_);
	drive_train_ = new DriveTrain("drivetrain.par", log_enabled_);
	feeder_ = new Feeder("feeder.par", log_enabled_);
	shooter_ = new Shooter("shooter.par", log_enabled_);
	user_interface_ = new UserInterface("userinterface.par", log_enabled_);
}

/**
 * \brief Loads the parameter file into memory, copies the values into local/member variables,
 * and creates and initializes objects using those values.
*/
bool TechnoJays::LoadParameters() {
	// Define and initialize local variables
	bool parameters_read = false; // This should default to false
	
	// Close and delete old objects
	SafeDelete(parameters_);
	
	// Attempt to read the parameters file
	parameters_ = new Parameters(parameters_file_);
	if (parameters_ != NULL && parameters_->file_opened_) {
		parameters_read = parameters_->ReadValues();
		parameters_->Close();
	}
	
	if (log_enabled_) {
		if (parameters_read)
			log_->WriteLine("TechnoJays parameters reloaded successfully\n");
		else
			log_->WriteLine("TechnoJays parameters failed to read\n");
	}
	
	// Set variables based on the parameters file
	if (parameters_read) {
		parameters_->GetValue("PERIOD", &period_);
		parameters_->GetValue("CAMERA_BOOT_TIME", &camera_boot_time_);
		parameters_->GetValue("INITIAL_TARGET_SEARCH_TIME", &initial_target_search_time_);
		parameters_->GetValue("AUTO_SHOOTER_SPINUP_TIME", &auto_shooter_spinup_time_);
		parameters_->GetValue("AUTO_SHOOTER_SPINDOWN_TIME", &auto_shooter_spindown_time_);
		parameters_->GetValue("AUTO_FEEDER_HEIGHT_ANGLE", &auto_feeder_height_angle_);
		parameters_->GetValue("AUTO_CLIMBING_ANGLE", &auto_climbing_angle_);
	}

	// Set the rate for the periodic methods
	// SetPeriod is part of the base class
	IterativeRobot::SetPeriod(period_);
	
	return parameters_read;
}

/**
 * \brief substitute for using the constructor in the class for consistency.
 *
 * Called when the robot is first turned on.  This method is only called once.
 * This is not used.  Initialize() is used instead.
*/
void TechnoJays::RobotInit() {
}

/**
 * \brief Prepares the robot for Disabled mode.
 *
 * Called when the robot is first disabled. This is called on a transition
 * from any other state.
 * E.g., Initializing sensors, resetting state/status and calculated variables,
 * or starting/restarting timers.
*/
void TechnoJays::DisabledInit() {
	IterativeRobot::SetPeriod(0);
	
	// Set the current state of the robot
	if (climber_ != NULL)
		climber_->SetRobotState(kDisabled);
	if (feeder_ != NULL)
		feeder_->SetRobotState(kDisabled);
	if (shooter_ != NULL)
		shooter_->SetRobotState(kDisabled);
	if (targeting_ != NULL)
		targeting_->SetRobotState(kDisabled);
	if (drive_train_ != NULL)
		drive_train_->SetRobotState(kDisabled);
	if (user_interface_ != NULL)
		user_interface_->SetRobotState(kDisabled);

	// Get the list of available autoscript files
	if (autoscript_ != NULL) {
		//autoscript_files_ = std::vector<std::string>();
		autoscript_files_.clear();
		int num_files_ = autoscript_->GetAvailableScripts(autoscript_files_);
		// Set the current autoscript file to the first one found and print on the screen
		if (num_files_ > 0) {
			autoscript_files_counter_ = 0;
			autoscript_file_name_ = autoscript_files_[autoscript_files_counter_];
			if (user_interface_ != NULL)
				user_interface_->OutputUserMessage(autoscript_file_name_.c_str(), true);
		}
	}
	
	// Read sensor values in all the objects
	if (shooter_ != NULL) {
		shooter_->ReadSensors();
	}
	if (climber_ != NULL) {
		climber_->ReadSensors();
	}
	if (drive_train_ != NULL) {
		drive_train_->ReadSensors();
	}
	
	// Reset and start a timer for camera initialization
	timer_->Stop();
	timer_->Reset();
	timer_->Start();
}

/**
 * \brief Performs tasks that need to be executed every iteration of the
 * control loop while in Disabled mode.
 *
 * Called continuously while the robot is disabled. Each time the program
 * returns from this function, it is immediately called again provided that the
 * state hasn’t changed.
 * This function should contain any maintenance code or routines that need to
 * run constantly.
*/
void TechnoJays::DisabledContinuous() {
	// Make sure no motors are moving (to prevent motor safety errors)
	if (drive_train_ != NULL) {
		drive_train_->Drive(0.0, 0.0, false);
	}
	if (climber_ != NULL) {
		climber_->Move(0.0, false);
	}	
	if (shooter_ != NULL) {
		if (shooter_->pitch_enabled_)
			shooter_->MovePitch(0.0, false);
		if (shooter_->shooter_enabled_)
			shooter_->Shoot(0);
	}

	// Initialize the targeting camera after a time delay
	// The camera must be configured before use, but it has a long
	//   bootup time, so a time delay is required.
	// Get the timer value
	double elapsed_time = timer_->Get();
	if (elapsed_time >= camera_boot_time_) {
		// Initialize the camera
		if (targeting_ != NULL && targeting_->camera_enabled_) {
			targeting_->InitializeCamera();
		}
		// Stop and reset the timer so that we don't keep trying to initialize over and over
		timer_->Stop();
		timer_->Reset();
	}
}

/**
 * \brief Performs tasks periodically during the Disabled mode.
 *
 * Called periodically during the Disabled mode based on a periodic timer for
 * the class.  If the period is 0, this function is syncronized with input
 * from the Driver Station.
 * 
 * This function should handle user-input during the disabled state before a
 * match starts.  E.g., Changing the autonomous routine.
*/
void TechnoJays::DisabledPeriodic() {
	DisabledContinuous();
	
	if (user_interface_ != NULL) {
		// Allow the user to cycle between the various autonomous programs while in Disabled mode
		if (user_interface_->GetButtonState(UserInterface::kDriver,UserInterface::kStart) == 1 &&
				user_interface_->ButtonStateChanged(UserInterface::kDriver,UserInterface::kStart)) {
			if (autoscript_ != NULL && !autoscript_files_.empty()) {
				autoscript_files_counter_++;
				if (autoscript_files_counter_ > (autoscript_files_.size()-1))
					autoscript_files_counter_ = 0;
				autoscript_file_name_ = autoscript_files_[autoscript_files_counter_];
				if (user_interface_ != NULL)
					user_interface_->OutputUserMessage(autoscript_file_name_.c_str(), true);
			}
		}
		// Update/store the current button state for driver controller
		user_interface_->StoreButtonStates(UserInterface::kDriver);
	}
}

/**
 * \brief Prepares the robot for Autonomous mode.
 *
 * Called when the robot enters the autonomous period for the first time. This is 
 * called on a transition from any other state.
 * 
 * This function should prepare the robot for autonomous mode.
 * E.g., Initializing sensors, resetting state/status and calculated variables,
 * or starting/restarting timers.
*/
void TechnoJays::AutonomousInit() {
	IterativeRobot::SetPeriod(period_);
	
	// Reset the timer in case the camera initialization didn't
	timer_->Stop();
	timer_->Reset();

	// Read the selected autonomous script file and get the first command
	if (!autoscript_file_name_.empty() && autoscript_file_name_.size() > 0) {
		autoscript_->Open(autoscript_file_name_.c_str());
		autoscript_->ReadScript();
		autoscript_->Close();
		current_command_complete_ = false;
		current_command_in_progress_ = false;
		current_command_ = autoscript_->GetNextCommand(); 
	}

	// Set the current state of the robot
	if (climber_ != NULL)
		climber_->SetRobotState(kAutonomous);
	if (feeder_ != NULL)
		feeder_->SetRobotState(kAutonomous);	
	if (shooter_ != NULL)
		shooter_->SetRobotState(kAutonomous);
	if (targeting_ != NULL)
		targeting_->SetRobotState(kAutonomous);
	if (drive_train_ != NULL)
		drive_train_->SetRobotState(kAutonomous);
	if (user_interface_ != NULL)
		user_interface_->SetRobotState(kAutonomous);
}

/**
 * \brief Performs tasks that need to be executed every iteration of the
 * control loop while in Autonomous mode.
 *
 * Called continuously while the in the autonomous part of the match. Each 
 * time the program returns from this function, it is immediately called again 
 * provided that the state hasn’t changed.
 * 
 * This function should contain the control loop for the automated routines.
 * E.g., Reading values from the sensors, or executing the selected
 * autonomous routine.
*/
void TechnoJays::AutonomousContinuous() {
	bool autoscript_finished = false;
	current_command_complete_ = false;
	
	// Read sensor values in all the objects
	if (shooter_ != NULL)
		shooter_->ReadSensors();
	if (drive_train_ != NULL)
		drive_train_->ReadSensors();
	if (climber_ != NULL)
		climber_->ReadSensors();
	
	// If autoscript is defined, execute the commands
	if (autoscript_ != NULL && !autoscript_file_name_.empty() && autoscript_file_name_.size() > 0) {
		// Verify that the current command is not invalid or the end
		if ((strncmp(current_command_.command, "invalid", 255) != 0) && (strncmp(current_command_.command, "end", 255) != 0)) {
			// Execute current autoscript command
			// General utilities
			if (strncmp(current_command_.command, "wait", 255) == 0) {
				if (current_command_.param1 == -9999)
					current_command_complete_ = true;
				else {
					if (!current_command_in_progress_) {
						timer_->Stop();
						timer_->Reset();
						timer_->Start();
						current_command_in_progress_ = true;
					}
					double time_left = 0.0;
					double elapsed_time = 999.0;
					// Get the timer value since we started moving
					elapsed_time = timer_->Get();
					// Calculate time left to move
					time_left = (double) current_command_.param1 - elapsed_time;
					if (time_left < 0) {
						timer_->Stop();
						current_command_complete_ = true;
					}
				}
			}
			// DriveTrain
			else if (strncmp(current_command_.command, "adjustheading", 255) == 0) {
				if (current_command_.param1 == -9999 || current_command_.param2 == -9999)
					current_command_complete_ = true;
				else {
					if (drive_train_->AdjustHeading(current_command_.param1, current_command_.param2))
						current_command_complete_ = true;
				}
			}
			else if (strncmp(current_command_.command, "drivedistance", 255) == 0) {
				if (current_command_.param1 == -9999 || current_command_.param2 == -9999)
					current_command_complete_ = true;
				else {
					if (drive_train_->Drive((double) current_command_.param1, current_command_.param2))
						current_command_complete_ = true;
				}
			}
			else if (strncmp(current_command_.command, "drivetime", 255) == 0) {
				if (current_command_.param1 == -9999 || current_command_.param2 == -9999 || current_command_.param3 == -9999)
					current_command_complete_ = true;
				else {
					if (!current_command_in_progress_) {
						drive_train_->ResetAndStartTimer();
						current_command_in_progress_ = true;
					}
					if (drive_train_->Drive((double) current_command_.param1, (Direction) current_command_.param2, current_command_.param3))
						current_command_complete_ = true;
				}
			}
			else if (strncmp(current_command_.command, "turnheading", 255) == 0) {
				if (current_command_.param1 == -9999 || current_command_.param2 == -9999)
					current_command_complete_ = true;
				else {
					if (drive_train_->Turn(current_command_.param1, current_command_.param2))
						current_command_complete_ = true;
				}
			}
			else if (strncmp(current_command_.command, "turntime", 255) == 0) {
				if (current_command_.param1 == -9999 || current_command_.param2 == -9999 || current_command_.param3 == -9999)
					current_command_complete_ = true;
				else {
					if (!current_command_in_progress_) {
						drive_train_->ResetAndStartTimer();
						current_command_in_progress_ = true;
					}
					if (drive_train_->Turn((double) current_command_.param1, (Direction) current_command_.param2, current_command_.param3))
						current_command_complete_ = true;
				}
			}
			// Shooter
			else if (strncmp(current_command_.command, "pitchposition", 255) == 0) {
				if (current_command_.param1 == -9999 || current_command_.param2 == -9999)
					current_command_complete_ = true;
				else {
					if (shooter_->SetPitch((int) current_command_.param1, current_command_.param2))
						current_command_complete_ = true;
				}
			}
			else if (strncmp(current_command_.command, "pitchtime", 255) == 0) {
				if (current_command_.param1 == -9999 || current_command_.param2 == -9999 || current_command_.param3 == -9999)
					current_command_complete_ = true;
				else {
					if (!current_command_in_progress_) {
						shooter_->ResetAndStartTimer();
						current_command_in_progress_ = true;
					}
					if (shooter_->SetPitch((double) current_command_.param1, (Direction) current_command_.param2, current_command_.param3))
						current_command_complete_ = true;
				}
			}
			else if (strncmp(current_command_.command, "pitchangle", 255) == 0) {
				if (current_command_.param1 == -9999 || current_command_.param2 == -9999)
					current_command_complete_ = true;
				else {
					if (shooter_->SetPitchAngle(current_command_.param1, current_command_.param2))
						current_command_complete_ = true;
				}
			}
			else if (strncmp(current_command_.command, "shoot", 255) == 0) {
				if (current_command_.param1 == -9999)
					current_command_complete_ = true;
				else {
					if (!current_command_in_progress_) {
						auto_shoot_state_ = kStep1;
						current_command_in_progress_ = true;
					}
					if (AutoShoot((int) current_command_.param1))
						current_command_complete_ = true;
				}
			}
			else if (strncmp(current_command_.command, "rapidfire", 255) == 0) {
				if (!current_command_in_progress_) {
					auto_rapid_fire_state_ = kStep1;
					current_command_in_progress_ = true;
				}
				if (AutoRapidFire())
					current_command_complete_ = true;
			}
			// Climber
			/*else if (strncmp(current_command_.command, "climberposition", 255) == 0) {
				if (current_command_.param1 == -9999 || current_command_.param2 == -9999)
					current_command_complete_ = true;
				else {
					if (climber_->Set((int) current_command_.param1, current_command_.param2))
						current_command_complete_ = true;
				}
			}
			else if (strncmp(current_command_.command, "climbertime", 255) == 0) {
				if (current_command_.param1 == -9999 || current_command_.param2 == -9999 || current_command_.param3 == -9999)
					current_command_complete_ = true;
				else {
					if (!current_command_in_progress_) {
						climber_->ResetAndStartTimer();
						current_command_in_progress_ = true;
					}
					if (climber_->Set((double) current_command_.param1, (Direction) current_command_.param2, current_command_.param3))
						current_command_complete_ = true;
				}
			}*/
			// Targeting
			else if (strncmp(current_command_.command, "findtarget", 255) == 0) {
				if (current_command_.param1 == -9999)
					current_command_complete_ = true;
				else {
					if (!current_command_in_progress_) {
						auto_find_target_state_ = kStep1;
						current_command_in_progress_ = true;
					}
					if (AutoFindTarget((Targeting::TargetHeight) current_command_.param1))
						current_command_complete_ = true;
				}
			}
			// Catchall
			else {
				current_command_complete_ = true;
			}
			
			// Get new command if current is finished
			if (current_command_complete_) {
				current_command_in_progress_ = false;
				current_command_ = autoscript_->GetNextCommand();
			}
		}
		else {
			autoscript_finished = true;
		}
	}
	else {
		autoscript_finished = true;
	}
	
	// If no autoscript or we're done, do nothing
	if (autoscript_finished) {
		if (drive_train_ != NULL) {
			drive_train_->Drive(0.0, 0.0, false);
		}
		if (climber_ != NULL) {
			climber_->Move(0.0, false);
		}	
		if (shooter_ != NULL) {
			if (shooter_->pitch_enabled_)
				shooter_->MovePitch(0.0, false);
			if (shooter_->shooter_enabled_)
				shooter_->Shoot(0);
		}
	}
}

/**
 * \brief Performs tasks periodically during the Autonomous mode.
 *
 * Called periodically during the autonomous part of the match based on a 
 * periodic timer for the class.
 * Unless the period is set to a value other than 0, this is never called!
 * 
 * This should be empty, unless there is a periodic behavior needed.  If
 * this is the case, the period must be set in AutonomousInit(), then
 * changed back to 0 in TeleopInit() and DisabledInit().
*/
void TechnoJays::AutonomousPeriodic() {
	AutonomousContinuous();
}

/**
 * \brief Prepares the robot for Teleop mode.
 *
 * Called when the robot enters the teleop period for the first time. This is
 * called on a transition from any other state.
 * 
 * This function should prepare the robot for user control instead of autonomous
 * mode.  E.g., Resetting state/status or calculated variables,
 * starting/restarting timers.
*/
void TechnoJays::TeleopInit() {
	IterativeRobot::SetPeriod(0);
	
	// Reset the timer in case the camera initialization didn't
	timer_->Stop();
	timer_->Reset();

	// Set the current state of the robot
	if (climber_ != NULL)
		climber_->SetRobotState(kTeleop);
	if (feeder_ != NULL)
		feeder_->SetRobotState(kTeleop);
	if (shooter_ != NULL)
		shooter_->SetRobotState(kTeleop);
	if (targeting_ != NULL)
		targeting_->SetRobotState(kTeleop);
	if (drive_train_ != NULL)
		drive_train_->SetRobotState(kTeleop);
	if (user_interface_ != NULL)
		user_interface_->SetRobotState(kTeleop);
}

/**
 * \brief Performs tasks that need to be executed every iteration of the
 * control loop while in Teleop mode.
 *
 * Called continuously while in the teleop part of the match. Each time the 
 * program returns from this function, it is immediately called again provided 
 * that the state hasn’t changed.
 * 
 * This function should contain code that does NOT depend on user input, or
 * that needs to run constantly.  E.g., Reading values from the sensors or
 * running the semi-autonomous functions when requested from TeleopPeriodic().
*/
void TechnoJays::TeleopContinuous() {
	// Read sensor values in all the objects
	if (shooter_ != NULL)
		shooter_->ReadSensors();
	if (drive_train_ != NULL)
		drive_train_->ReadSensors();
	if (climber_ != NULL)
		climber_->ReadSensors();

	// Log detailed data if enabled
	if (detailed_logging_enabled_) {
		if (shooter_ != NULL)
			shooter_->LogCurrentState();
		if (drive_train_ != NULL)
			drive_train_->LogCurrentState();
		if (climber_ != NULL)
			climber_->LogCurrentState();
	}
	
	// Perform any TeleOp Autonomous routines
	if (auto_rapid_fire_state_ != kFinished) {
		if (AutoRapidFire())
			auto_rapid_fire_state_ = kFinished;
	}
	if (auto_shoot_state_ != kFinished) {
		//log_->WriteLine("Calling AS in telecont\n");
		if (AutoShoot(100))
			auto_shoot_state_ = kFinished;
	}
	if (auto_find_target_state_ != kFinished) {
		if (AutoFindTarget(Targeting::kHigh))
			auto_find_target_state_ = kFinished;
	}
	if (auto_cycle_target_state_ != kFinished) {
		// Choose the next target
		if (auto_cycle_target_state_ == kStep1) {
			NextTarget();
			auto_cycle_target_state_ = kStep2;
			aim_state_ = kStep1;
		}
		// Aim at the target
		else if (auto_cycle_target_state_ == kStep2) {
			// Auto Aim at the target
			if (AimAtTarget()) {
				auto_cycle_target_state_ = kFinished;
				aim_state_ = kFinished;
			}
		} else {
			auto_cycle_target_state_ = kFinished;
		}
	}
	if (auto_feeder_height_state_ != kFinished) {
		if (AutoFeederHeight())
			auto_feeder_height_state_ = kFinished;
	}
	if (auto_climbing_prep_state_ != kFinished) {
		if (AutoClimbingPrep())
			auto_climbing_prep_state_ = kFinished;
	}
}

/**
 * \brief Performs tasks periodically during the Teleop mode.
 *
 * Called periodically during the teleoperation part of the match based on a 
 * periodic timer for the class.  If the period is 0, this function is
 * syncronized with input from the Driver Station.
 * 
 * This function should contain code that is dependent on user input, or code
 * that is not time critical.  E.g., Driving the robot, moving arms,
 * performing user requested semi-autonomous functions.
*/
void TechnoJays::TeleopPeriodic() {
	TeleopContinuous();
	
	if (user_interface_ != NULL) {
		float driver_left_y = 0.0;
		float driver_right_x = 0.0;
		float scoring_left_y = 0.0;
		float scoring_right_y = 0.0;
		float scoring_dpad_y = 0.0;

		// Get the values for the thumbsticks and dpads
		driver_left_y = user_interface_->GetAxisValue(UserInterface::kDriver, UserInterface::kLeftY);
		driver_right_x = user_interface_->GetAxisValue(UserInterface::kDriver, UserInterface::kRightX);
		scoring_left_y = user_interface_->GetAxisValue(UserInterface::kScoring, UserInterface::kLeftY);
		scoring_right_y = user_interface_->GetAxisValue(UserInterface::kScoring, UserInterface::kRightY);
		scoring_dpad_y = user_interface_->GetAxisValue(UserInterface::kScoring, UserInterface::kDpadY);

		// Log analog controls if detailed logging is enabled
		if (detailed_logging_enabled_) {
			log_->WriteValue("DriverLeftY", driver_left_y, true);
			log_->WriteValue("DriverRightX", driver_right_x, true);
			log_->WriteValue("DriverTurbo", user_interface_->GetButtonState(UserInterface::kDriver,
				UserInterface::kRightBumper), true);
			log_->WriteValue("ScoringLeftY", scoring_left_y, true);
			log_->WriteValue("ScoringRightY", scoring_right_y, true);
			log_->WriteValue("ScoringTurbo", user_interface_->GetButtonState(UserInterface::kScoring,
				UserInterface::kRightBumper), true);
			log_->WriteValue("Shooter", user_interface_->GetButtonState(UserInterface::kScoring,
				UserInterface::kLeftTrigger), true);
		}

		// Check for turbo mode
		if (user_interface_->GetButtonState(UserInterface::kDriver,
				UserInterface::kRightBumper) == 1) {
			driver_turbo_ = true;
		} else {
			driver_turbo_ = false;
		}
		if (user_interface_->GetButtonState(UserInterface::kScoring,
				UserInterface::kRightBumper) == 1) {
			scoring_turbo_ = true;
		} else {
			scoring_turbo_ = false;
		}
		
		// Check if a TeleOp Auto routine is requested
		// AutoShoot
		if (user_interface_->GetButtonState(UserInterface::kScoring, UserInterface::kRightTrigger) == 1
				&& user_interface_->ButtonStateChanged(UserInterface::kScoring, UserInterface::kRightTrigger)) {
			auto_rapid_fire_state_ = kFinished;
			auto_shoot_state_ = kStep1;
			memset(output_buffer_, 0, sizeof(output_buffer_));
			sprintf(output_buffer_, "AutoShoot..");
			user_interface_->OutputUserMessage(output_buffer_, true);
		}
		// Rapid Fire
		if (user_interface_->GetButtonState(UserInterface::kScoring,UserInterface::kX) == 1
				&& user_interface_->ButtonStateChanged(UserInterface::kScoring, UserInterface::kX)) {
			auto_shoot_state_ = kFinished;
			auto_rapid_fire_state_ = kStep1;
			memset(output_buffer_, 0, sizeof(output_buffer_));
			sprintf(output_buffer_, "Rapid Fire..");
			user_interface_->OutputUserMessage(output_buffer_, true);
		}
		// Find Targets
		if (user_interface_->GetButtonState(UserInterface::kScoring,UserInterface::kB) == 1 
				&& user_interface_->ButtonStateChanged(UserInterface::kScoring, UserInterface::kB)) {
			auto_cycle_target_state_ = kFinished;
			auto_feeder_height_state_ = kFinished;
			auto_climbing_prep_state_ = kFinished;
			auto_find_target_state_ = kStep1;
			memset(output_buffer_, 0, sizeof(output_buffer_));
			sprintf(output_buffer_, "Find Targets..");
			user_interface_->OutputUserMessage(output_buffer_, true);
		}
		// Next Target
		if (user_interface_->GetButtonState(UserInterface::kScoring,UserInterface::kY) == 1 
				&& user_interface_->ButtonStateChanged(UserInterface::kScoring, UserInterface::kY)) {
			auto_find_target_state_ = kFinished;
			auto_feeder_height_state_ = kFinished;
			auto_climbing_prep_state_ = kFinished;
			auto_cycle_target_state_ = kStep1;
		}
		// Auto Feed Height
		if (user_interface_->GetButtonState(UserInterface::kScoring,UserInterface::kA) == 1 
				&& user_interface_->ButtonStateChanged(UserInterface::kScoring, UserInterface::kA)) {
			auto_find_target_state_ = kFinished;
			auto_cycle_target_state_ = kFinished;
			auto_climbing_prep_state_ = kFinished;
			auto_feeder_height_state_ = kStep1;
			memset(output_buffer_, 0, sizeof(output_buffer_));
			sprintf(output_buffer_, "AutoFeedHeight..");
			user_interface_->OutputUserMessage(output_buffer_, true);
		}
		
		// Auto Climb Preparation (move shooter pitch to lowest setting)
		if (user_interface_->GetButtonState(UserInterface::kScoring, UserInterface::kStart) == 1 
				&& user_interface_->ButtonStateChanged(UserInterface::kScoring, UserInterface::kStart)) {
			auto_find_target_state_ = kFinished;
			auto_cycle_target_state_ = kFinished;
			auto_feeder_height_state_ = kFinished;
			auto_climbing_prep_state_ = kStep1;
			memset(output_buffer_, 0, sizeof(output_buffer_));
			sprintf(output_buffer_, "AutoClimbingPrep..");
			user_interface_->OutputUserMessage(output_buffer_, true);
		}
		
		// Manually control the robot
		// Abort any current or currently requested autonomous routines when manual controls are used.
		// When there isn't any user input and no autonomous routines are running, we still have to
		//   set the motors to not moving.
		// The motors need to be controlled each loop iteration, or else we get motor safety errors.
		
		// Climber
		if (scoring_right_y != 0.0) {
			// Even though these don't use the winch directly, they use the pitch
			// so we don't want them running if the winch is moving.
			auto_feeder_height_state_ = kFinished;
			auto_find_target_state_ = kFinished;
			auto_cycle_target_state_ = kFinished;
			auto_climbing_prep_state_ = kFinished;
			//TODO
			//if (climber_ != NULL && shooter_ != NULL && shooter_->PitchClearForClimbing()) {
			if (climber_ != NULL) {
				climber_->Move(scoring_right_y, scoring_turbo_);
			}
		}
		else {
			// Don't put the auto programs above for this one since they're not directly using the winch/climber
			// If we did, the climber/winch motor wouldn't get set to 0 while they ran and we'd get safety errors
			if (climber_ != NULL) {
				climber_->Move(0.0, false);
			}
		}
		
		// Shooter
		if (scoring_left_y != 0.0) {
			auto_find_target_state_ = kFinished;
			auto_cycle_target_state_ = kFinished;
			auto_feeder_height_state_ = kFinished;
			auto_climbing_prep_state_ = kFinished;
			if (shooter_ != NULL) {
				shooter_->MovePitch(scoring_left_y, scoring_turbo_);
			}
		}
		else if (auto_find_target_state_ == kFinished
				&& auto_cycle_target_state_ == kFinished
				&& auto_feeder_height_state_ == kFinished
				&& auto_climbing_prep_state_ == kFinished) {
			if (shooter_ != NULL) {
				shooter_->MovePitch(0.0, false);
			}
		}
		if (user_interface_->GetButtonState(UserInterface::kScoring,UserInterface::kLeftTrigger) == 1) {
			auto_shoot_state_ = kFinished;
			auto_rapid_fire_state_ = kFinished;
			if (shooter_ != NULL) {
				shooter_->Shoot(100);
			}
		}
		else if (auto_shoot_state_ == kFinished
				&& auto_rapid_fire_state_ == kFinished) {
			if (shooter_ != NULL) {
				shooter_->Shoot(0);
			}
		}
		
		// Feeder
		if (scoring_dpad_y != 0.0 && previous_scoring_dpad_y_ != scoring_dpad_y &&
				user_interface_->GetButtonState(UserInterface::kScoring,UserInterface::kLeftTrigger) == 1) {
			auto_shoot_state_ = kFinished;
			auto_rapid_fire_state_ = kFinished;
			if (feeder_ != NULL) {
				feeder_->SetPiston(true);
			}
		}
		else if (auto_shoot_state_ == kFinished
				&& auto_rapid_fire_state_ == kFinished) {
			if (feeder_ != NULL) {
				feeder_->SetPiston(false);
			}
		}

		// DriveTrain
		if (driver_left_y != 0.0 || driver_right_x != 0.0) {
			auto_find_target_state_ = kFinished;
			auto_cycle_target_state_ = kFinished;
			if (drive_train_ != NULL) {
				drive_train_->Drive(driver_left_y, driver_right_x, driver_turbo_);
			}
		}
		else if (auto_find_target_state_ == kFinished
				&& auto_cycle_target_state_ == kFinished) {
			if (drive_train_ != NULL) {
				drive_train_->Drive(0.0, 0.0, false);
			}
		}

		// Log current state of each object when diagnostics button (BACK) is pressed on driver
		if (user_interface_->GetButtonState(UserInterface::kDriver, UserInterface::kBack) == 1 
				&& user_interface_->ButtonStateChanged(UserInterface::kDriver, UserInterface::kBack)) {
			// Print title
			user_interface_->OutputUserMessage("Diagnostics", true);
			memset(output_buffer_, 0, sizeof(output_buffer_));
			if (drive_train_ != NULL) {
				drive_train_->LogCurrentState();
				drive_train_->GetCurrentState(output_buffer_);
				user_interface_->OutputUserMessage(output_buffer_, false);
			}
			if (shooter_ != NULL) {
				shooter_->LogCurrentState();
				shooter_->GetCurrentState(output_buffer_);
				user_interface_->OutputUserMessage(output_buffer_, false);
			}
			if (climber_ != NULL) {
				climber_->LogCurrentState();
				climber_->GetCurrentState(output_buffer_);
				user_interface_->OutputUserMessage(output_buffer_, false);
			}
		}
		
		// Toggle logging detailed mode when logging button (B) is pressed on driver
		if (user_interface_->GetButtonState(UserInterface::kDriver,
				UserInterface::kB) == 1 && user_interface_->ButtonStateChanged(
				UserInterface::kDriver, UserInterface::kB)) {
			if (detailed_logging_enabled_) {
				detailed_logging_enabled_ = false;
				user_interface_->OutputUserMessage("Logging disabled", false);
			} else {
				detailed_logging_enabled_ = true;
				user_interface_->OutputUserMessage("Logging enabled", false);
			}
		}

		// Update/store the current button state for both controllers
		user_interface_->StoreButtonStates(UserInterface::kDriver);
		user_interface_->StoreButtonStates(UserInterface::kScoring);
	}
}

void TechnoJays::PrintTargetInfo() {
	// Print target info
	if (user_interface_ != NULL && targeting_ != NULL) {
		memset(output_buffer_, 0, sizeof(output_buffer_));
		Targeting::TargetHeight target_height = targeting_->GetEnumHeightOfTarget(&current_target_);
		sprintf(output_buffer_, "Height: ");
		targeting_->GetStringHeightOfTarget(target_height, output_buffer_+8);
		user_interface_->OutputUserMessage(output_buffer_, true);
		sprintf(output_buffer_, "Distance: %4.2f", (float) targeting_->GetCameraDistanceToTarget(&current_target_));			
		user_interface_->OutputUserMessage(output_buffer_, false);
		sprintf(output_buffer_, "H-Angle: %4.2f", (float) targeting_->GetHorizontalAngleOfTarget(&current_target_));			
		user_interface_->OutputUserMessage(output_buffer_, false);
		sprintf(output_buffer_, "V-Angle: %4.2f", (float) targeting_->GetVerticalAngleOfTarget(&current_target_));			
		user_interface_->OutputUserMessage(output_buffer_, false);
	}
}

/**
 * \brief Steers the robot to face a target.
*/
bool TechnoJays::AimAtTarget() {
	// Abort if we don't have what we need
	if ((current_target_.imageWidth == 0 && current_target_.imageHeight == 0) || drive_train_ == NULL 
			|| shooter_ == NULL || targeting_ == NULL) {
		aim_state_ = kFinished;
		return true;
	}

	memset(output_buffer_, 0, sizeof(output_buffer_));
	sprintf(output_buffer_, "Aiming...");
	user_interface_->OutputUserMessage(output_buffer_, true);

	switch (aim_state_) {
	// Calculate heading adjustment
	case kStep1:
		degrees_off_ = targeting_->GetHorizontalAngleOfTarget(&current_target_);
		// Fall through to step 2
		aim_state_ = kStep2;
	// Adjust heading until aimed at target
	case kStep2:
		if (drive_train_->AdjustHeading(degrees_off_, 1.0)) {
			aim_state_ = kStep3;
		}
		break;
	case kStep3:
		degrees_off_ = targeting_->GetVerticalAngleOfTarget(&current_target_);
		// Fall through to step 4
		aim_state_ = kStep4;
	case kStep4:
		if (shooter_->SetPitchAngle(degrees_off_, 1.0)) {
			aim_state_ = kFinished;
			memset(output_buffer_, 0, sizeof(output_buffer_));
			sprintf(output_buffer_, "Finished.");
			user_interface_->OutputUserMessage(output_buffer_, false);
			return true;
		} else {
			break;
		}
	default:
		aim_state_ = kFinished;
		memset(output_buffer_, 0, sizeof(output_buffer_));
		sprintf(output_buffer_, "Finished.");
		user_interface_->OutputUserMessage(output_buffer_, false);
		return true;
	}

	return false;
}

/**
 * \brief Get a list of targets from the targeting module.
 *
 * This stores a copy of the target list locally, since the targeting
 * module continues searching.
*/
void TechnoJays::GetTargets() {
	// Abort if we don't have what we need
	if (targeting_ == NULL || !targeting_->camera_enabled_)
		return;

	// Erase old data
	if (!targets_report_.empty())
		targets_report_.clear();
		
	// Reset drive train sensors
	if (drive_train_ != NULL)
		drive_train_->ResetSensors();
	target_report_heading_ = 0.0;

	// Create a new empty target report
	current_target_ = ParticleAnalysisReport();
	current_target_.imageHeight = 0;
	current_target_.imageWidth = 0;
	
	// Search for targets
	if (!targeting_->GetTargets(targets_report_))
		return;

	// Store current robot heading
	if (drive_train_ != NULL)
		target_report_heading_ = drive_train_->GetHeading();
	
}

/**
 * \brief Selects the next target in the list of potential targets.
*/
void TechnoJays::NextTarget() {
	// Only cycle if we have more than 1 target
	if (targets_report_.size() > 1) {
		// Increment/cycle the target counter
		if ((current_target_vector_location_ + 1) >= targets_report_.size()) {
			current_target_vector_location_ = 0;
		} else {
			current_target_vector_location_++;
		}

		// Get the target report from the report of all targets
		ParticleAnalysisReport target = (targets_report_.at(
				current_target_vector_location_));

		// Copy the pointer to our class variable
		current_target_ = target;

		PrintTargetInfo();
	}
}

/**
 * \brief Select a target from the target list that is nearest the specified height.
 *
 * \param height the preferred target height to select out of the targets that were found.
*/
void TechnoJays::SelectTarget(Targeting::TargetHeight height) {
	// Abort if no targets found
	if (targets_report_.size() == 0)
		return;
	
	// Local target variables during search
	ParticleAnalysisReport target;
	Targeting::TargetHeight current_height = Targeting::kUnknown;
	
	// Loop through all detected targets
	for (unsigned i = 0; i < targets_report_.size(); i++) {
		// Get the current target and height
		target = (targets_report_.at(i));
		current_height = targeting_->GetEnumHeightOfTarget(&target);
		// If it matches, store the current target and return
		if (current_height == height) {
			current_target_ = target;
			current_target_vector_location_ = i;
			return;
		}
	}
	
	// If the expected was the low height and nothing found, choose lowest target found
	if (height == Targeting::kLow) {
		current_target_ = (targets_report_.at(0));
		current_target_vector_location_ = 0;
	}
	// Otherwise choose the highest target found
	else {
		current_target_vector_location_ = targets_report_.size() - 1;
		current_target_ = (targets_report_.at(current_target_vector_location_));
	}
}

/**
 * \brief Automatically finds a target of the specified height and aims the robot at it.
 *
 * \param height the preferred target height to select out of the targets that were found.
 * \return true when the operation is complete.
*/
bool TechnoJays::AutoFindTarget(Targeting::TargetHeight height) {
	switch (auto_find_target_state_) {
	// Get the target list and select the specified target
	case kStep1:
		GetTargets();
		auto_find_target_state_ = kStep2;
		break;
	case kStep2:
		SelectTarget(height);
		if (user_interface_ != NULL) {
			memset(output_buffer_, 0, sizeof(output_buffer_));
			sprintf(output_buffer_, "Finished.");
			user_interface_->OutputUserMessage(output_buffer_, false);
		}
		aim_state_ = kStep1;
		auto_find_target_state_ = kStep3;
		break;
	// Aim at the selected target
	case kStep3:
		if (AimAtTarget()) {
			aim_state_ = kFinished;
			auto_find_target_state_ = kFinished;
			return true;
		} else {
			break;
		}
	default:
		auto_find_target_state_ = kFinished;
		return true;
	}

	return false;
}

/**
 * \brief Automatically spins up the shooter and feeds a disc.
 *
 * \param power the amount of power as a percentage to shoot with.
 * \return true when the operation is complete.
*/
bool TechnoJays::AutoShoot(int power) {
	// Abort if we don't have what we need
	if (feeder_ == NULL || shooter_ == NULL || !feeder_->feeder_enabled_ || !shooter_->shooter_enabled_) {
		auto_shoot_state_ = kFinished;
		return true;
	}

	double time_left = 0.0;
	// Get the timer value since the last event
	double elapsed_time = auto_shoot_timer_->Get();

	// Spin up the shooter and keep it moving until we're done (regardless of what step)
	shooter_->Shoot(power);

	switch (auto_shoot_state_) {
	// Keep track of how long we've spun up the shooter
	case kStep1:
		auto_shoot_timer_->Stop();
		auto_shoot_timer_->Reset();
		auto_shoot_timer_->Start();
		elapsed_time = 0.0;
		auto_shoot_state_ = kStep2;
		// Fall through into kStep2
	// Pre-delay for the shooter to spinup
	case kStep2:
		// Calculate time left
		time_left = auto_shooter_spinup_time_ - elapsed_time;
		// If enough time has passed, feed a disc
		if (time_left <= 0.0) {
			auto_shoot_state_ = kStep3;
			auto_shoot_timer_->Stop();
			// Fall through to kStep3
		} else {
			break;
		}
	// Feed a disc into the shooter
	case kStep3:
		feeder_->SetPiston(true);
		auto_shoot_timer_->Reset();
		auto_shoot_timer_->Start();
		elapsed_time = 0.0;
		auto_shoot_state_ = kStep4;
		// Fall through to kStep4
	// Post-delay for the shooter to finish shooting
	case kStep4:
		// Calculate time left
		time_left = auto_shooter_spindown_time_ - elapsed_time;
		// If enough time has passed, we're done
		if (time_left <= 0.0) {
			auto_shoot_timer_->Stop();
			auto_shoot_timer_->Reset();
			// Retract feeder
			feeder_->SetPiston(false);
			// Stop spinning the shooter motor
			shooter_->Shoot(0);
			auto_shoot_state_ = kFinished;
			if (user_interface_ != NULL) {
				memset(output_buffer_, 0, sizeof(output_buffer_));
				sprintf(output_buffer_, "Finished.");
				user_interface_->OutputUserMessage(output_buffer_, false);
			}
			return true;
		} else {
			break;
		}
	default:
		shooter_->Shoot(0);
		auto_shoot_state_ = kFinished;
		if (user_interface_ != NULL) {
			memset(output_buffer_, 0, sizeof(output_buffer_));
			sprintf(output_buffer_, "Finished.");
			user_interface_->OutputUserMessage(output_buffer_, false);
		}
		return true;
	}

	return false;
}

/**
 * \brief Automatically sets the feeder to the proper angle to get discs from the feeder station.
 *
 * \return true when the operation is complete.
*/
bool TechnoJays::AutoFeederHeight() {
	// Abort if we don't have what we need
	if (shooter_ == NULL) {
		auto_feeder_height_state_ = kFinished;
		return true;
	}
	
	switch (auto_feeder_height_state_) {
	case kStep1:
		if (shooter_->SetPitchAngle(auto_feeder_height_angle_, 1.0)) {
			auto_feeder_height_state_ = kFinished;
			if (user_interface_ != NULL) {
				memset(output_buffer_, 0, sizeof(output_buffer_));
				sprintf(output_buffer_, "Finished.");
				user_interface_->OutputUserMessage(output_buffer_, false);
			}
			return true;
		} else {
			break;
		}
	default:
		auto_feeder_height_state_ = kFinished;
		if (user_interface_ != NULL) {
			memset(output_buffer_, 0, sizeof(output_buffer_));
			sprintf(output_buffer_, "Finished.");
			user_interface_->OutputUserMessage(output_buffer_, false);
		}
		return true;
	}

	return false;
}

/**
 * \brief Automatically set the pitch to the lowest angle to get it out of the way for climbing.
 *
 * \return true when the operation is complete.
*/
bool TechnoJays::AutoClimbingPrep() {
	// Abort if we don't have what we need
	if (shooter_ == NULL) {
		auto_climbing_prep_state_ = kFinished;
		return true;
	}
	
	switch (auto_climbing_prep_state_) {
	case kStep1:
		if (shooter_->SetPitchAngle(auto_climbing_angle_, 1.0)) {
			auto_climbing_prep_state_ = kFinished;
			if (user_interface_ != NULL) {
				memset(output_buffer_, 0, sizeof(output_buffer_));
				sprintf(output_buffer_, "Finished.");
				user_interface_->OutputUserMessage(output_buffer_, false);
			}
			return true;
		} else {
			break;
		}
	default:
		auto_climbing_prep_state_ = kFinished;
		if (user_interface_ != NULL) {
			memset(output_buffer_, 0, sizeof(output_buffer_));
			sprintf(output_buffer_, "Finished.");
			user_interface_->OutputUserMessage(output_buffer_, false);
		}
		return true;
	}

	return false;
}

/**
 * \brief Automatically spins up the shooter and feeds up to 4 discs.
 *
 * \return true when the operation is complete.
*/
bool TechnoJays::AutoRapidFire() {
	// Abort if we don't have what we need
	if (feeder_ == NULL || shooter_ == NULL || !feeder_->feeder_enabled_ || !shooter_->shooter_enabled_) {
		auto_rapid_fire_state_ = kFinished;
		return true;
	}
	
	double time_left = 0.0;
	// Get the timer value since the last event
	double elapsed_time = auto_shoot_timer_->Get();

	// Spin up the shooter and keep it moving until we're done (regardless of what step)
	shooter_->Shoot(100);
	
	switch (auto_rapid_fire_state_) {
	// Keep track of how long we've spun up the shooter
	case kStep1:
		auto_shoot_timer_->Stop();
		auto_shoot_timer_->Reset();
		auto_shoot_timer_->Start();
		elapsed_time = 0.0;
		auto_rapid_fire_state_ = kStep2;
		// Fall through into kStep2
	// Pre-delay for the shooter to spinup
	case kStep2:
		// Calculate time left
		time_left = auto_shooter_spinup_time_ - elapsed_time;
		// If enough time has passed, feed a disc
		if (time_left <= 0.0) {
			auto_rapid_fire_state_ = kStep3;
			auto_shoot_timer_->Stop();
			// Fall through to kStep3
		} else {
			break;
		}
	// Feed a disc into the shooter
	case kStep3:
		feeder_->SetPiston(true);
		auto_shoot_timer_->Reset();
		auto_shoot_timer_->Start();
		elapsed_time = 0.0;
		auto_rapid_fire_state_ = kStep4;
		// Fall through to kStep4
	// Post-delay for the shooter to finish shooting
	case kStep4:
		// Calculate time left
		time_left = auto_shooter_spindown_time_ - elapsed_time;
		// If enough time has passed, retract to shoot another
		if (time_left <= 0.0) {
			// Retract feeder
			feeder_->SetPiston(false);
			auto_shoot_timer_->Stop();
			auto_rapid_fire_state_ = kStep5;
			break;
		} else {
			break;
		}
	// Feed second disc into the shooter
	case kStep5:
		feeder_->SetPiston(true);
		auto_shoot_timer_->Reset();
		auto_shoot_timer_->Start();
		elapsed_time = 0.0;
		auto_rapid_fire_state_ = kStep6;
		// Fall through to kStep6
	// Post-delay for the shooter to finish shooting
	case kStep6:
		// Calculate time left
		time_left = auto_shooter_spindown_time_ - elapsed_time;
		// If enough time has passed, retract to shoot another
		if (time_left <= 0.0) {
			// Retract feeder
			feeder_->SetPiston(false);
			auto_shoot_timer_->Stop();
			auto_rapid_fire_state_ = kStep7;
			break;
		} else {
			break;
		}
	// Feed third disc into the shooter
	case kStep7:
		feeder_->SetPiston(true);
		auto_shoot_timer_->Reset();
		auto_shoot_timer_->Start();
		elapsed_time = 0.0;
		auto_rapid_fire_state_ = kStep8;
		// Fall through to kStep8
	// Post-delay for the shooter to finish shooting
	case kStep8:
		// Calculate time left
		time_left = auto_shooter_spindown_time_ - elapsed_time;
		// If enough time has passed, we're done
		if (time_left <= 0.0) {
			auto_shoot_timer_->Stop();
			auto_shoot_timer_->Reset();
			// Retract feeder
			feeder_->SetPiston(false);
			// Stop spinning the shooter motor
			shooter_->Shoot(0);
			auto_rapid_fire_state_ = kFinished;
			if (user_interface_ != NULL) {
				memset(output_buffer_, 0, sizeof(output_buffer_));
				sprintf(output_buffer_, "Finished.");
				user_interface_->OutputUserMessage(output_buffer_, false);
			}
			return true;
		} else {
			break;
		}
	default:
		shooter_->Shoot(0);
		auto_rapid_fire_state_ = kFinished;
		if (user_interface_ != NULL) {
			memset(output_buffer_, 0, sizeof(output_buffer_));
			sprintf(output_buffer_, "Finished.");
			user_interface_->OutputUserMessage(output_buffer_, false);
		}
		return true;
	}

	return false;
}

START_ROBOT_CLASS(TechnoJays);
