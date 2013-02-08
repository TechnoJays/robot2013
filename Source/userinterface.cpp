#include <math.h>
#include "WPILib.h"
#include "userinterface.h"
#include "datalog.h"
#include "parameters.h"

/**
 * \brief Load the UI.
 * 
 * Use the default parameter file "userinterface.par" and logging is disabled.
*/
UserInterface::UserInterface() {
	Initialize("userinterface.par", false);
}

/**
 * \brief Load the UI.
 *
 * Use the default parameter file "userinterface.par" and enable/disable
 * logging based on the parameter.
 *
 * \param logging_enabled true if logging is enabled.
*/
UserInterface::UserInterface(bool logging_enabled) {
	Initialize("userinterface.par", logging_enabled);
}

/**
 * \brief Load the UI.
 * 
 * Use the user specified parameter file and logging is disabled.
 *
 * \param parameters user interface parameter file path and name.
*/
UserInterface::UserInterface(const char * parameters) {
	Initialize(parameters, false);
}

/**
 * \brief Load the UI.
 *
 * Use the user specified parameter file and enable/disable logging
 * based on the parameter.
 *
 * \param parameters user interface parameter file path and name.
 * \param logging_enabled true if logging is enabled.
*/
UserInterface::UserInterface(const char * parameters, bool logging_enabled) {
	Initialize(parameters, logging_enabled);
}

/**
 * \brief Delete and clear all objects and pointers.
*/
UserInterface::~UserInterface() {
	if (log_ != NULL) {
		log_->Close();
	}
	SafeDelete(parameters_);
	SafeDelete(controller_1_previous_button_state_);
	SafeDelete(controller_2_previous_button_state_);
	SafeDelete(driver_station_lcd_);
	SafeDelete(controller_1_);
	SafeDelete(controller_2_);
	SafeDelete(log_);
}

/**
 * \brief Initialize the UserInterface object.
 *
 * Create member objects, initialize default values, read parameters from the param file.
 *
 * \param parameters user interface parameter file path and name.
 * \param logging_enabled true if logging is enabled.
*/
void UserInterface::Initialize(const char * parameters, bool logging_enabled) {
	// Initialize private member objects
	driver_station_lcd_ = NULL;
	controller_1_ = NULL;
	controller_2_ = NULL;
	log_ = NULL;
	parameters_ = NULL;
	controller_1_previous_button_state_ = NULL;
	controller_2_previous_button_state_ = NULL;

	// Initialize private parameters
	controller_1_buttons_ = 4;
	controller_2_buttons_ = 4;
	controller_1_dead_band_ = 0.05;
	controller_2_dead_band_ = 0.05;
	
	// Initialize private member variables
	display_line_ = 0;
	log_enabled_ = false;
	robot_state_ = kDisabled;

	// Create a new data log object
	log_ = new DataLog("userinterface.log");
	// Get the Driver Station LCD object
	driver_station_lcd_ = DriverStationLCD::GetInstance();
	
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
 *
 * \return true if successful.
*/
bool UserInterface::LoadParameters() {
	// Define and initialize local variables
	int controller_1_port = 1;
	int controller_2_port = 2;
	int controller_1_axis = 2;
	int controller_2_axis = 2;
	bool parameters_read = false;
	
	// Close and delete old objects
	SafeDelete(parameters_);
	SafeDelete(controller_1_previous_button_state_);
	SafeDelete(controller_2_previous_button_state_);
	SafeDelete(controller_1_);
	SafeDelete(controller_2_);
	
	// Attempt to read the parameters file
	parameters_ = new Parameters(parameters_file_);
	if (parameters_ != NULL && parameters_->file_opened_) {
		parameters_read = parameters_->ReadValues();
		parameters_->Close();
	}
	
	if (log_enabled_) {
		if (parameters_read)
			log_->WriteLine("UserInterface parameters loaded successfully\n");
		else
			log_->WriteLine("UserInterface parameters failed to read\n");
	}
	
	// Set user interface variables based on the parameters file
	if (parameters_read) {
		parameters_->GetValue("CONTROLLER1_PORT", &controller_1_port);
		parameters_->GetValue("CONTROLLER2_PORT", &controller_2_port);
		parameters_->GetValue("CONTROLLER1_AXIS", &controller_1_axis);
		parameters_->GetValue("CONTROLLER2_AXIS", &controller_2_axis);
		parameters_->GetValue("CONTROLLER1_BUTTONS", &controller_1_buttons_);
		parameters_->GetValue("CONTROLLER2_BUTTONS", &controller_2_buttons_);
		parameters_->GetValue("CONTROLLER1_DEAD_BAND", &controller_1_dead_band_);
		parameters_->GetValue("CONTROLLER2_DEAD_BAND", &controller_2_dead_band_);
	}
	
	// Initialize previous button state arrays
	controller_1_previous_button_state_ = new int[controller_1_buttons_+1];
	controller_2_previous_button_state_ = new int[controller_2_buttons_+1];
	
	// Initialize controller objects
	controller_1_ = new Joystick(controller_1_port, controller_1_axis, controller_1_buttons_);
	controller_2_ = new Joystick(controller_2_port, controller_2_axis, controller_2_buttons_);

	// Store current button states
	StoreButtonStates(UserInterface::kDriver);
	StoreButtonStates(UserInterface::kScoring);
	
	return parameters_read;
}

/**
 * \brief Set the current state of the robot and perform any actions necessary during mode changes.
 *
 * \param state current robot state.
*/
void UserInterface::SetRobotState(ProgramState state) {
	robot_state_ = state;
}

/**
 * \brief Enable or disable logging for this object.
 *
 * \param state true if logging should be enabled.
*/
void UserInterface::SetLogState(bool state) {
	if (state)
		log_enabled_ = true;
	else
		log_enabled_ = false;
}

/**
 * \brief Check if the the button state for the specified controller/button has changed since the last "Store".
 *
 * \param controller the controller to read the button state from.
 * \param button the button ID to read the state from.
 * \return true if the button state has changed.
*/
bool UserInterface::ButtonStateChanged(int controller, int button) {
	// Get the current button state
	int current_state = GetButtonState(controller, button);
	int previous_state = 0;
	
	// Get the previous button state
	switch (controller) {
		case 0:
			if (controller_1_previous_button_state_ != NULL) {
				previous_state = controller_1_previous_button_state_[button];
			}
			break;
		case 1:
			if (controller_2_previous_button_state_ != NULL) {
				previous_state = controller_2_previous_button_state_[button];
			}
			break;
		default:
			break;
	}
	
	// Check if the button state has changed
	if (current_state != previous_state)
		return true;
	else
		return false;
}

/**
 * \brief Read the current axis value for the specified controller/axis.
 *
 * \param controller the controller to read the axis value from.
 * \param axis the axis ID to read.
 * \return the current position of the specified axis.
*/
float UserInterface::GetAxisValue(int controller, int axis) {
	float value = 0.0;
	
	// Get the current button state from the controller
	switch (controller) {
		case 0:
			value = controller_1_->GetRawAxis(axis);
			if (controller_1_ == NULL || fabs(value) < controller_1_dead_band_) {
				return 0.0;
			}
			return value;
		case 1:
			value = controller_2_->GetRawAxis(axis);
			if (controller_2_ == NULL || fabs(value) < controller_2_dead_band_) {
				return 0.0;
			}
			return value;
		default:
			return 0.0;
	}
}

/**
 * \brief Read the button state for the specified controller/button.
 *
 * \param controller the controller to read the button state from.
 * \param button the button ID to read the state from.
 * \return 1 if button is currently pressed.
*/
int UserInterface::GetButtonState(int controller, int button) {
	// Get the current button state from the controller
	switch (controller) {
		case 0:
			if (controller_1_ == NULL) {
				return 0;
			}
			return (controller_1_->GetRawButton(button));
		case 1:
			if (controller_2_ == NULL) {
				return 0;
			}
			return (controller_2_->GetRawButton(button));
		default:
			return 0;
	}
}

/**
 * \brief Displays a message on the User Messages window of the Driver Station.
 *
 * Automatically keeps track of the line numbering and clears when necessary.
 * Clearing can also be done manually using the clear parameter.
 *
 * \param message the text to display on the DriverStation.
 * \param clear true if the screen should be cleared prior to displaying the message.
*/
void UserInterface::OutputUserMessage(const char * message, bool clear) {
	if (driver_station_lcd_ == NULL) {
		return;
	}
	
	DriverStationLCD::Line current_display_line = DriverStationLCD::kUser_Line1;

	// If clear is specified, erase the user output screen and set line count to 0
	if (clear) {
		display_line_ = 0;
		driver_station_lcd_->Clear();
	}

	// Set the display line using the current line counter
	// When the max line # is reached, erase the screen and start at 0
	switch (display_line_) {
		case 0:
			current_display_line = DriverStationLCD::kUser_Line1;
			break;
		case 1:
			current_display_line = DriverStationLCD::kUser_Line2;
			break;
		case 2:
			current_display_line = DriverStationLCD::kUser_Line3;
			break;
		case 3:
			current_display_line = DriverStationLCD::kUser_Line4;
			break;
		case 4:
			current_display_line = DriverStationLCD::kUser_Line5;
			break;
		case 5:
			current_display_line = DriverStationLCD::kUser_Line6;
			break;
		default:
			current_display_line = DriverStationLCD::kUser_Line1;
			display_line_ = 0;
			driver_station_lcd_->Clear();
			break;
	}
	
	// Display the string on the User Messages section of DriverStation
	driver_station_lcd_->PrintfLine(current_display_line, message);
	driver_station_lcd_->UpdateLCD();
	display_line_++;
	if (log_enabled_) {
		log_->WriteValue("LCDOutput", message);
	}
}

/**
 * \brief Store the current button states for the specified controller.
 *
 * \param controller the controller to read the button states from.
*/
void UserInterface::StoreButtonStates(int controller) {
	int button_state = 0;
	int button_count = 0;
	
	// Get the total number of buttons for the controller
	switch (controller) {
		case 0:
			button_count = controller_1_buttons_;
			break;
		case 1:
			button_count = controller_2_buttons_;
			break;
		default:
			button_count = 0;
			break;
	}

	// Store the current state of each button for this controller
	// +1 is used in array indexing since #defines and GetRawButton() start at 1,
	//    and array starts at 0.
	for (int i=0; i<(button_count); i++) {			
		button_state = GetButtonState(controller, i+1);
		switch (controller) {
			case 0:
				if (controller_1_previous_button_state_ != NULL) {
					controller_1_previous_button_state_[i+1] = button_state;
				}
				break;
			case 1:
				if (controller_2_previous_button_state_ != NULL) {
					controller_2_previous_button_state_[i+1] = button_state;
				}
				break;
			default:
				break;
		}
	}
}
