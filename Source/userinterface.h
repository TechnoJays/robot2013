#ifndef USERINTERFACE_H_
#define USERINTERFACE_H_

#include "common.h"

// Forward class definitions
class DataLog;
class DriverStationLCD;
class Joystick;
class Parameters;

/**
 * \class UserInterface
 * \brief Robot interface to the DriverStation and joysticks.
 * 
 * Facilitates robot output to the DriverStation, and reading user input
 * from the joysticks/controllers.
 */
class UserInterface {

public:
	// Public enums and structs
	/**
	 * \enum JoystickAxis
	 * \brief An enumeration of all possible axis on the controller.
	 */
	enum JoystickAxis {
		kLeftX=1,	///< Left thumbstick, X axis
		kLeftY=2,	///< Left thumbstick, Y axis
		kRightX=3,	///< Right thumbstick, X axis
		kRightY=4,	///< Right thumbstick, Y axis
		kDpadX=5,	///< Directional pad, X axis
		kDpadY=6	///< Directoinal pad, Y axis
	};
	
	/**
	 * \enum JoystickButtons
	 * \brief An enumeration of all possible buttons on the controller.
	 */
	enum JoystickButtons {
		kX=1,
		kA=2,
		kB=3,
		kY=4,
		kLeftBumper=5,
		kRightBumper=6,
		kLeftTrigger=7,
		kRightTrigger=8,
		kBack=9,
		kStart=10
	};

	/**
	 * \enum UserControllers
	 * \brief An enumeration of the available controllers
	 */
	enum UserControllers {
		kDriver,	///< The driver controller handles all aspects of moving the robot
		kScoring	///< The scoring controller handles all aspects of scoring in a game
	};

	// Public methods
	UserInterface();
	UserInterface(bool logging_enabled);
	UserInterface(const char * parameters);
	UserInterface(const char * parameters, bool logging_enabled);
	~UserInterface();
	bool LoadParameters();
	void SetRobotState(ProgramState state);
	void SetLogState(bool state);
	bool ButtonStateChanged(int controller, int button);
	float GetAxisValue(int controller, int axis);
	int GetButtonState(int controller, int button);
	void OutputUserMessage(const char * message, bool clear);
	void StoreButtonStates(int controller);

private:
	// Private methods
	void Initialize(const char * parameters, bool logging_enabled);
	
	// Private member objects
	Joystick 			*controller_1_;							///< joystick object used to get button and axis states for controller 1
	Joystick 			*controller_2_;							///< joystick object used to get button and axis states for controller 2
	int					*controller_1_previous_button_state_;	///< array of last known button states for controller 1
	int	 				*controller_2_previous_button_state_;	///< array of last known button states for controller 2
	DriverStationLCD	*driver_station_lcd_;					///< driver station lcd object used to output text messages on the driver station screen
	DataLog 			*log_;									///< log object used to log data or status comments to a file
	Parameters 			*parameters_;							///< parameters object used to load UI parameters from a file
	
	// Private parameters
	int controller_1_buttons_;		///< number of buttons on controller 1
	int controller_2_buttons_;		///< number of buttons on controller 2
	float controller_1_dead_band_;	///< region (absolute value) of all axis on controller 1 that are ignored as potential error
	float controller_2_dead_band_;	///< region (absolute value) of all axis on controller 2 that are ignored as potential error
	
	// Private member variables
	int 	display_line_;		///< current text output line on the DriverStation
	bool 	log_enabled_;		///< true if logging is enabled
	char *parameters_file_;		///< path and filename of the parameter file to read
	ProgramState robot_state_;	///< current state of the robot obtained from the field
};

#endif
