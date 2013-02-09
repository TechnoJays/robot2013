#ifndef FEEDER_H_
#define FEEDER_H_

#include "common.h"

// Forward class definitions
class DataLog;
class DigitalInput;
class Parameters;
class Relay;
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
	void ReadSensors();
	void SetRobotState(ProgramState state);
	void GetCurrentState(char * output_buffer);
	void LogCurrentState();
	void SetLogState(bool state);
    bool GetPressureSwitchState();
	void SetCompressor(bool state);
	void SetPiston(bool state);
	
	// Public member variables
	bool feeder_enabled_;			///< true if the entire feeder system is present and initialized
	bool pressure_switch_enabled_;	///< true if the pressure switch is present and initialized
	bool compressor_relay_enabled_;	///< true if the compressor relay is present and initialized
	bool solenoid_enabled_;			///< true if the solenoid is present and initialized

private:
	// Private methods
	void Initialize(char * parameters, bool logging_enabled);

	// Private member objects
	DataLog *log_;				    ///< log object used to log data or status comments to a file
	DigitalInput *pressure_switch_; ///< pressure switch that detects when the compressor should be turned on/off
    Parameters *parameters_;	    ///< parameters object used to load feeder parameters from a file
	Relay *compressor_power_;       ///< relay to turn the compressor on/off
	Solenoid *piston_;				///< solenoid to control the feeder piston
    
	// Private parameters

	// Private member variables
	unsigned int pressure_switch_state_;	///< status of the pressure switch
	bool log_enabled_;						///< true if logging is enabled
	char parameters_file_[25];				///< path and filename of the parameter file to read
	ProgramState robot_state_;				///< current state of the robot obtained from the field
};

#endif
