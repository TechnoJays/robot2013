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
	void DisabledContinuous();
	void DisabledPeriodic();
	void AutonomousInit();
	void AutonomousContinuous();
	void AutonomousPeriodic();
	void TeleopInit();
	void TeleopContinuous();
	void TeleopPeriodic();
	bool AutoShoot(int power);

	// Public member variables
	vector<ParticleAnalysisReport> targets_report_;	///< particle reports of matching hoop targets
	
private:
	// Private enums
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
		kFinished
	};
	
	// Private methods
	bool AimAtTarget();
	bool AutoFindTarget(Targeting::TargetHeight height);
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
	
	// Private member variables
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
	double degrees_off_center_;					///< the number of degrees the robot is off from facing the selected target
	unsigned current_target_vector_location_;	///< the index in the particle report vector of the current target, used when cycling through targets
	AutoState auto_shoot_state_;				///< the current state of the AutoShoot autonomous function
	AutoState aim_state_;						///< the current state of the AimAtTarget function
	AutoState auto_find_target_state_;			///< the current state of the AutoFindTarget function
};

#endif
