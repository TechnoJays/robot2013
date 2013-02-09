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

	// Initialize private member variables
	log_enabled_ = false;
	detailed_logging_enabled_ = false;
	driver_turbo_ = false;
	scoring_turbo_ = false;
	
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
	double period = 0.0;
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
		parameters_->GetValue("PERIOD", &period);
		parameters_->GetValue("CAMERA_BOOT_TIME", &camera_boot_time_);
		parameters_->GetValue("INITIAL_TARGET_SEARCH_TIME", &initial_target_search_time_);
		parameters_->GetValue("AUTO_SHOOTER_SPINUP_TIME", &auto_shooter_spinup_time_);
		parameters_->GetValue("AUTO_SHOOTER_SPINDOWN_TIME", &auto_shooter_spindown_time_);
	}

	// Set the rate for the periodic methods
	// SetPeriod is part of the base class
	IterativeRobot::SetPeriod(period);
	
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
	if (feeder_ != NULL)
		feeder_->ReadSensors();
	
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

	if (feeder_ != NULL)
		feeder_->ReadSensors();
		if (feeder_->GetPressureSwitchState())
			feeder_->SetCompressor(true);
		else
			feeder_->SetCompressor(false);
	
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
	
	// Read sensor values in all the objects
	if (shooter_ != NULL)
		shooter_->ReadSensors();
	if (drive_train_ != NULL)
		drive_train_->ReadSensors();
	if (climber_ != NULL)
		climber_->ReadSensors();
	if (feeder_ != NULL)
		feeder_->ReadSensors();	
		if (feeder_->GetPressureSwitchState())
			feeder_->SetCompressor(true);
		else
			feeder_->SetCompressor(false);
	
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
			// Climber
			else if (strncmp(current_command_.command, "climberposition", 255) == 0) {
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
			}
			// Targeting
			// TODO
			// Feeder
			// TODO
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
	if (feeder_ != NULL)
		feeder_->ReadSensors();
		if (feeder_->GetPressureSwitchState())
			feeder_->SetCompressor(true);
		else
			feeder_->SetCompressor(false);

	// Log detailed data if enabled
	if (detailed_logging_enabled_) {
		if (shooter_ != NULL)
			shooter_->LogCurrentState();
		if (drive_train_ != NULL)
			drive_train_->LogCurrentState();
		if (climber_ != NULL)
			climber_->LogCurrentState();
		if (feeder_ != NULL)
			feeder_->LogCurrentState();
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
	if (user_interface_ != NULL) {
		float driver_left_y = 0.0;
		float driver_right_x = 0.0;
		float scoring_left_y = 0.0;
		float scoring_right_y = 0.0;

		// Get the values for the thumbsticks and dpads
		driver_left_y = user_interface_->GetAxisValue(UserInterface::kDriver, UserInterface::kLeftY);
		driver_right_x = user_interface_->GetAxisValue(UserInterface::kDriver, UserInterface::kRightX);
		scoring_left_y = user_interface_->GetAxisValue(UserInterface::kScoring, UserInterface::kLeftY);
		scoring_right_y = user_interface_->GetAxisValue(UserInterface::kScoring, UserInterface::kRightY);

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

		// Manually control the robot
		// Abort any current or currently requested autonomous routines when manual controls are used.
		// When there isn't any user input and no autonomous routines are running, we still have to
		//   set the motors to not moving.
		// The motors need to be controlled each loop iteration, or else we get motor safety errors.
		// Climber
		if (scoring_right_y != 0.0) {
			if (climber_ != NULL) {
				climber_->Move(scoring_right_y, scoring_turbo_);
			}
		} else if (true) {
			if (climber_ != NULL) {
				climber_->Move(0.0, false);
			}
		}
		// Shooter
		if (scoring_left_y != 0.0) {
			if (shooter_ != NULL) {
				shooter_->MovePitch(scoring_left_y, scoring_turbo_);
			}
		} else if (true) {
			if (shooter_ != NULL) {
				shooter_->MovePitch(0.0, false);
			}
		}
		if (user_interface_->GetButtonState(UserInterface::kScoring,
				UserInterface::kLeftTrigger) == 1) {
			if (shooter_ != NULL) {
				shooter_->Shoot(100);
			}
		} else if (true) {
			if (shooter_ != NULL) {
				shooter_->Shoot(0);
			}
		}
		// DriveTrain
		if (driver_left_y != 0.0 || driver_right_x != 0.0) {
			if (drive_train_ != NULL) {
				drive_train_->Drive(driver_left_y, driver_right_x, driver_turbo_);
			}
		} else if (true) {
			if (drive_train_ != NULL) {
				drive_train_->Drive(0.0, 0.0, false);
			}
		}

		// Log current state of each object when diagnostics button (BACK) is pressed on driver
		if (user_interface_->GetButtonState(UserInterface::kDriver, UserInterface::kBack) == 1 
				&& user_interface_->ButtonStateChanged(UserInterface::kDriver, UserInterface::kBack)) {
			// Print title
			user_interface_->OutputUserMessage("Diagnostics", true);
			char output_buffer[22] = { 0 };
			if (drive_train_ != NULL) {
				drive_train_->LogCurrentState();
				drive_train_->GetCurrentState(output_buffer);
				user_interface_->OutputUserMessage(output_buffer, false);
			}
			if (shooter_ != NULL) {
				shooter_->LogCurrentState();
				shooter_->GetCurrentState(output_buffer);
				user_interface_->OutputUserMessage(output_buffer, false);
			}
			if (climber_ != NULL) {
				climber_->LogCurrentState();
				climber_->GetCurrentState(output_buffer);
				user_interface_->OutputUserMessage(output_buffer, false);
			}
			if (feeder_ != NULL) {
				feeder_->LogCurrentState();
				feeder_->GetCurrentState(output_buffer);
				user_interface_->OutputUserMessage(output_buffer, false);
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
			return true;
		} else {
			break;
		}
	default:
		shooter_->Shoot(0);
		auto_shoot_state_ = kFinished;
		return true;
	}

	return false;
}

START_ROBOT_CLASS(TechnoJays);
