#include <cmath>
#include "WPILib.h"
#include "feeder.h"
#include "datalog.h"
#include "parameters.h"

/**
 * \brief Create and initialize a feeder.
 *
 * Use the default parameter file "feeder.par" and logging is disabled.
*/
Feeder::Feeder() {
	Initialize("feeder.par", false);
}

/**
 * \brief Create and initialize a feeder.
 *
 * Use the default parameter file "feeder.par" and enable/disable
 * logging based on the parameter.
 *
 * \param logging_enabled true if logging is enabled.
*/
Feeder::Feeder(bool logging_enabled) {
	Initialize("feeder.par", logging_enabled);
}

/**
 * \brief Create and initialize a feeder.
 *
 * Use the user specified parameter file and logging is disabled.
 *
 * \param parameters feeder parameter file path and name.
*/
Feeder::Feeder(char * parameters) {
	Initialize(parameters, false);
}

/**
 * \brief Create and initialize a feeder.
 *
 * Use the user specified parameter file and enable/disable
 * logging based on the parameter.
 *
 * \param parameters feeder parameter file path and name.
 * \param logging_enabled true if logging is enabled.
*/
Feeder::Feeder(char * parameters, bool logging_enabled) {
	Initialize(parameters, logging_enabled);
}

/**
 * \brief Delete and clear all objects and pointers.
*/
Feeder::~Feeder() {
	if (log_ != NULL) {
		log_->Close();
	}
	SafeDelete(log_);
	SafeDelete(parameters_);
	SafeDelete(compressor_);
    SafeDelete(piston_);
}

/**
 * \brief Initialize the Feeder object.
 *
 * Create member objects, initialize default values, read parameters from the param file.
 *
 * \param parameters feeder parameter file path and name.
 * \param logging_enabled true if logging is enabled.
*/
void Feeder::Initialize(char * parameters, bool logging_enabled) {
	// Initialize public member variables
	feeder_enabled_ = false;
	compressor_enabled_ = false;
	solenoid_enabled_ = false;

	// Initialize private member objects
	log_ = NULL;
	parameters_ = NULL;
	compressor_ = NULL;
    piston_ = NULL;
	
	// Initialize private parameters

    // Initialize private member variables
	log_enabled_ = false;
	robot_state_ = kDisabled;
	
	// Create a new data log object
	log_ = new DataLog("feeder.log");

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
bool Feeder::LoadParameters() {
	// Define and initialize local variables
	int pressure_switch_channel = -1;
	int compressor_relay_channel = -1;
	int solenoid_channel = -1;
	bool parameters_read = false;	// This should default to false
	
	// Close and delete old objects
	SafeDelete(parameters_);
	SafeDelete(compressor_);
    SafeDelete(piston_);
	
	// Attempt to read the parameters file
	parameters_ = new Parameters(parameters_file_);
	if (parameters_ != NULL && parameters_->file_opened_) {
		parameters_read = parameters_->ReadValues();
		parameters_->Close();
	}
	
	if (log_enabled_) {
		if (parameters_read)
			log_->WriteLine("Feeder parameters loaded successfully\n");
		else
			log_->WriteLine("Feeder parameters failed to read\n");
	}

	// Set feeder variables based on the parameters file
	if (parameters_read) {
		parameters_->GetValue("PRESSURE_SWITCH_CHANNEL", &pressure_switch_channel);
		parameters_->GetValue("COMPRESSOR_RELAY_CHANNEL", &compressor_relay_channel);
		parameters_->GetValue("SOLENOID_CHANNEL", &solenoid_channel);
	}
	
	// Check if the compressor is present/enabled
	if (pressure_switch_channel > 0 && compressor_relay_channel > 0) {
		compressor_ = new Compressor(pressure_switch_channel, compressor_relay_channel);
		if (compressor_ != NULL) {
			compressor_enabled_ = true;
		}
	}

	// Check if the solenoid is present/enabled
	if (solenoid_channel > 0) {
		piston_ = new Solenoid(solenoid_channel);
		if (piston_ != NULL) {
			solenoid_enabled_ = true;
		}
	}
	else {
		solenoid_enabled_ = false;
	}
	
	// Feeder is only enabled if everything is working
	//if (pressure_switch_enabled_ && compressor_relay_enabled_ && solenoid_enabled_)
	if (compressor_enabled_ && solenoid_enabled_)
		feeder_enabled_ = true;
	else
		feeder_enabled_ = false;
	
	if (log_enabled_) {
		if (compressor_enabled_) {
			log_->WriteLine("Compressor enabled\n");
		}
		else {
			log_->WriteLine("Compressor disabled\n");
		}
		if (solenoid_enabled_) {
			log_->WriteLine("Solenoid enabled\n");
		}
		else {
			log_->WriteLine("Solenoid disabled\n");
		}
		if (feeder_enabled_) {
			log_->WriteLine("Feeder enabled\n");
		}
		else {
			log_->WriteLine("Feeder disabled\n");
		}
	}
	
	return parameters_read;
}

/**
 * \brief Set the current state of the robot and perform any actions necessary during mode changes.
 *
 * \param state current robot state.
*/
void Feeder::SetRobotState(ProgramState state) {
	robot_state_ = state;

	// Make sure the compressor is running in every state
	if (compressor_enabled_) {
		if (!compressor_->Enabled())
			compressor_->Start();
	}

	if (state == kDisabled) {
	}
	else if (state == kTeleop) {
	}
	else if (state == kAutonomous) {
	}
}

/**
 * \brief Enable or disable logging for this object.
 *
 * \param state true if logging should be enabled.
*/
void Feeder::SetLogState(bool state) {
	if (state && log_ != NULL)
		log_enabled_ = true;
	else
		log_enabled_ = false;
}

/**
 * \brief Set the solenoid/piston on/off.
 *
 * \param state true if the solenoid is active
*/
void Feeder::SetPiston(bool state) {
	// Abort if feeder not available
	if (!feeder_enabled_ || !solenoid_enabled_)
		return;
	piston_->Set(state);
}
