#ifndef TECHNOJAYS_H_
#define TECHNOJAYS_H_

#include "WPILib.h"
#include "common.h"
#include <string.h>
#include <vector>
#include "autoscript.h"

// Forward class definitions
class AutoScript;
class DataLog;
class DriveTrain;
class Parameters;
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
		
private:
	// Private methods
	void Initialize(const char * parameters, bool logging_enabled);
	
	// Private member objects
	DriveTrain *drive_train_;				///< controls the robot drive train to drive and turn
	UserInterface *user_interface_;			///< gets input from the controllers and sends messages back to the DriverStation
	DataLog *log_;							///< log object used to log data or status comments to a file
	Parameters *parameters_;				///< parameters object used to load robot parameters from a file
	AutoScript *autoscript_;				///< autoscript object used to load autonomous script from a file
	
	// Private member variables
	bool driver_turbo_;							///< true if the driver controller is requesting turbo mode
	bool scoring_turbo_;						///< true if the scoring controller is requesting turbo mode
	bool detailed_logging_enabled_;				///< true if detailed robot and driver details should be logged
	bool log_enabled_;							///< true if logging is enabled
	char *parameters_file_;						///< path and filename of the parameter file to read
	std::string autoscript_file_name_;			///< file name of the selected autoscript file for autonomous mode
	unsigned int autoscript_files_counter_;		///< counter of the current file selected in the autoscript_files_ vector
	std::vector<std::string> autoscript_files_;	///< vector of autoscript file names from the file system
	bool current_command_complete_;				///< true when an autonomous command finishes and the next should be executed
	autoscript_command current_command_;		///< the current autonomous command being executed

};

#endif
