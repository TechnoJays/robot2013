#ifndef TECHNOJAYS_H_
#define TECHNOJAYS_H_

#include "WPILib.h"
#include "common.h"
#include <string.h>
#include <vector>
#include "autoscript.h"
#include "targeting.h"


// Forward class definitions
class AutoScript;
class Climber;
class DataLog;
class DriveTrain;
class Feeder;
class Parameters;
class Shooter;
class Targeting;
class UserInterface;

/**
 * \class TechnoJays
 * \brief Main robot.
 */
class TechnoJays : public IterativeRobot {

public:	
	// Public methods
	TechnoJays();
	~TechnoJays();
	bool LoadParameters();
	void RobotInit();
	void DisabledInit();
	void DisabledPeriodic();
	void AutonomousInit();
	void AutonomousPeriodic();
	void TeleopInit();
	void TeleopPeriodic();

	// Public member variables
	vector<ParticleAnalysisReport> targets_report_;	///< particle reports of matching hoop targets
	
private:
	// Private enums
	// Store the current state of autonomous functions/state machines
	enum AutoState {
		kStep1,
		kStep2,
		kStep3,
		kStep4,
		kStep5,
		kStep6,
		kStep7,
		kStep8,
		kStep9,
		kStep10,
		kStep11,
		kStep12,
		kStep13,
		kStep14,
		kStep15,
		kFinished
	};
	
	// Private methods
	bool AimAtTarget();
	bool AutoFeederHeight();
	bool AutoClimbingPrep();
	bool AutoClimb();
	bool AutoFindTarget(Targeting::TargetHeight height);
	bool AutoRapidFire();
	bool AutoShoot(int power);
	void GetTargets();
	void Initialize(const char * parameters, bool logging_enabled);
	void NextTarget();
	void PrintTargetInfo();
	void SelectTarget(Targeting::TargetHeight height);
	
	
	// Private member objects
	AutoScript *autoscript_;				///< autoscript object used to load autonomous script from a file
	Climber *climber_;						///< controls the climbing winch to climb the pyramid
	DataLog *log_;							///< log object used to log data or status comments to a file
	DriveTrain *drive_train_;				///< controls the robot drive train to drive and turn
	Feeder *feeder_;						///< controls the feeder to feed discs to the shooter
	Parameters *parameters_;				///< parameters object used to load robot parameters from a file
	Shooter *shooter_;						///< controls the robot to shoot discs
	Targeting *targeting_;					///< finds and reports details about targets
	UserInterface *user_interface_;			///< gets input from the controllers and sends messages back to the DriverStation
	ParticleAnalysisReport current_target_;	///< contains information about the currently selected target from the camera
	Timer *timer_;							///< timer object used for misc timed functions
	Timer *auto_shoot_timer_;				///< timer object used for AutoShoot function
	
	// Private parameters
	double camera_boot_time_;				///< the amount of time required for the Axis camera to bootup
	double initial_target_search_time_;		///< the amount of time required for the Targeting module to search for targets
	float auto_shooter_spinup_time_;		///< the amount of time to spin up the shooter before feeding a disc	
	float auto_shooter_spindown_time_;		///< the amount of time to spin down the shooter after feeding a disc
	float auto_feeder_height_angle_;		///< the angle of the shooter required to set the feeder to the height for the feeder station
	float auto_climbing_angle_;				///< the angle of the shooter required to be out of the way for climbing
	int auto_climbing_encoder_count_;		///< the encoder count of the shooter required to be out of the way for climbing
	float auto_climb_backup_speed_;			///< the backup speed to drive while autoclimbing
	int auto_climb_headstart_encoder_count_;///< the encoder count of the shooter to have a headstart for auto climbing
	float auto_climb_winch_speed_;			///< the winch speed during auto climbing
	float auto_climb_winch_time_;			///< the winch duration during auto climbing
	double period_;							///< the period in seconds for the periodic loops
	
	// Private member variables
	float previous_scoring_dpad_y_;				///< the last known value of the Y axis on the scoring directional pad
	bool driver_turbo_;							///< true if the driver controller is requesting turbo mode
	bool scoring_turbo_;						///< true if the scoring controller is requesting turbo mode
	bool detailed_logging_enabled_;				///< true if detailed robot and driver details should be logged
	bool log_enabled_;							///< true if logging is enabled
	char parameters_file_[25];					///< path and filename of the parameter file to read
	char output_buffer_[22];					///< character buffer for outputting messages to the driver station LCD
	std::string autoscript_file_name_;			///< file name of the selected autoscript file for autonomous mode
	unsigned int autoscript_files_counter_;		///< counter of the current file selected in the autoscript_files_ vector
	std::vector<std::string> autoscript_files_;	///< vector of autoscript file names from the file system
	bool current_command_complete_;				///< true when an autonomous command finishes and the next should be executed
	bool current_command_in_progress_;			///< true when an autonomous command has already started
	autoscript_command current_command_;		///< the current autonomous command being executed
	float target_report_heading_;				///< the heading of the robot when the target report was generated
	double degrees_off_;						///< the number of degrees the robot is off from facing the selected target
	unsigned current_target_vector_location_;	///< the index in the particle report vector of the current target, used when cycling through targets
	bool auto_climb_pitch_finished_;			///< true when the pitch is finished moving during auto climb
	bool auto_climb_winch_finished_;			///< true when the winch is finished moving during auto climb
	AutoState auto_shoot_state_;				///< the current state of the AutoShoot autonomous function
	AutoState aim_state_;						///< the current state of the AimAtTarget function
	AutoState auto_find_target_state_;			///< the current state of the AutoFindTarget function
	AutoState auto_rapid_fire_state_;			///< the current state of the AutoRapidFire function
	AutoState auto_cycle_target_state_;			///< the current state of the AutoCycleTarget function
	AutoState auto_feeder_height_state_;		///< the current state of the AutoFeederHeight function
	AutoState auto_climbing_prep_state_;		///< the current state of the AutoClimbingPrep function
	AutoState auto_climb_state_;				///< the current state of the AutoClimb function
};

#endif
