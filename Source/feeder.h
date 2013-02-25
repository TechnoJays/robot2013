#ifndef FEEDER_H_
#define FEEDER_H_

#include "common.h"

// Forward class definitions
class Compressor;
class DataLog;
class Parameters;
class Solenoid;

/**
 * \class Feeder
 * \brief Controls a robot feeder.
 * 
 * Provides a simple interface to control a robot feeder.
 */
class Feeder {

public:	
	// Public methods
	Feeder();
	Feeder(bool logging_enabled);
	Feeder(char * parameters);
	Feeder(char * parameters, bool logging_enabled);
	~Feeder();
	bool LoadParameters();
	void SetRobotState(ProgramState state);
	void SetLogState(bool state);
	void SetPiston(bool state);
	
	// Public member variables
	bool feeder_enabled_;			///< true if the entire feeder system is present and initialized
	bool compressor_enabled_;		///< true if the compressor is present and initialized
	bool solenoid_enabled_;			///< true if the solenoid is present and initialized

private:
	// Private methods
	void Initialize(char * parameters, bool logging_enabled);

	// Private member objects
	Compressor *compressor_;		///< compressor object to control the compressor
	DataLog *log_;				    ///< log object used to log data or status comments to a file
    Parameters *parameters_;	    ///< parameters object used to load feeder parameters from a file
	Solenoid *piston_;				///< solenoid to control the feeder piston
    
	// Private parameters

	// Private member variables
	bool log_enabled_;						///< true if logging is enabled
	char parameters_file_[25];				///< path and filename of the parameter file to read
	ProgramState robot_state_;				///< current state of the robot obtained from the field
};

#endif
