#include "parameters.h"

/**
 * \brief Open a file with the mode "r" to read program parameters.
 *
 * \param path the path and filename of the parameter file.
*/
Parameters::Parameters(const char * path) {
	file_ = NULL;
	file_opened_ = false;
	Parameters::Open(path);
}

/**
 * \brief Open the default file "parameters.txt" with the mode "r" to read program parameters.
*/
Parameters::Parameters() {
	file_ = NULL;
	file_opened_ = false;
	Parameters::Open("parameters.txt");
}

/**
 * \brief Deletes the file object.
*/
Parameters::~Parameters() {
	SafeDelete(file_);
}

/**
 * \brief Open a file with the mode "r".
 *
 * \param path the path and filename of the parameter file.
 * \return true if successful.
*/
bool Parameters::Open(const char * path) {
	if (path == NULL) {
        file_opened_ = false;
		return false;
	}
	
	if ((file_ = fopen(path, "r")) <= 0) {
		/*printf("Error opening file = %s\n", strerror(errno));
		printf("file = %s\n", path);*/
		file_opened_ = false;
		return false;
	}
	else {
        file_opened_ = true;
		return true;
	}
}

/**
 * \brief Close the parameter file.
*/
void Parameters::Close() {
	if (file_ != NULL) {
		fclose(file_);
		file_opened_ = false;
	}
}

/**
 * \brief Read all parameter/value pairs from the file.
 *
 * Reads the entire parameter file and searches for NAME = VALUE pairs.
 * The pairs are stored in map variables depending on the type of VALUE.
 * All pairs are read as float type, then read as string type if float failed.
 *
 * \return true if successful.
*/
bool Parameters::ReadValues() {
	char buffer[256] = {0};
	char parameter[255] = {0};
	char value_string[255] = {0};
	char comment[255] = {0};
	float value_float = 0.0;
	int match_count = 0;

	// Clear old value_string
	number_parameters_.clear();
	string_parameters_.clear();
	
	if (file_ != NULL) {
		// Loop while there's data to read
		while (fgets(buffer, 255, file_) != NULL) {
			// Clear any previous values of the strings
			comment[0]=0;
			value_string[0]=0;
			// Try to format the value as a number
			// Also search for anything after the number as a comment
			match_count = sscanf(buffer, "%s = %f %[^\n]", parameter, &value_float, comment);
			if (match_count >= 2) {
				//std::cout << "Name: " << parameter << ", Value: " << value_float << ", Comment: " << comment << "\n";
				number_parameters_[parameter] = value_float;
				continue;
			}
			// Try to format the value as a string
			// Also search for anything after a '#' as a comment
			match_count = sscanf(buffer, "%s = %[^#\n] %[^\n]", parameter, value_string, comment);
			if (match_count >= 2) {
				// Remove any extra whitespace between the string value and the '#' if it exists
				if (strlen(value_string) > 0 && value_string[strlen(value_string)-1] == ' ') {
					value_string[strlen(value_string)-1] = 0;
				}
				//std::cout << "Name: " << parameter << ", Value: " << value_string << ", Comment: " << comment << "\n";
				string_parameters_[parameter] = value_string;
				continue;
			}
			// Could not match the line to any of the formats, return an error
			else {
				return false;
			}
		}
		return true;
	}
	else {
		return false;
	}
}

/**
 * \brief Get the matching text for the specified parameter.
 *
 * Searches the text parameters that were read from the file for the parameter name
 * specified.  If a match is found, it returns the text value associated
 * with the parameter name.
 *
 * \param parameter the name of the parameter.
 * \param value pointer to a character array to store the string into.
 * \return true if successful.
*/
bool Parameters::GetValue(const char * parameter, char * value) {
	string_parameters_iterator_ = string_parameters_.find(parameter);
	// If the iterator is not set to the end, then we found a match
	if (string_parameters_iterator_ != string_parameters_.end()) {
		std::string temp_string = string_parameters_iterator_->second;
		char * new_string = new char[temp_string.size() + 1];
		new_string[temp_string.size()] = 0;
		memcpy(new_string, temp_string.c_str(), temp_string.size());
		memcpy(value, new_string, strlen(new_string));
		SafeDelete(new_string);
		return true;
	}
	// Otherwise, no match found
	else {
		return false;
	}
}

/**
 * \brief Get the matching integer for the specified parameter.
 *
 * Searches the number parameters that were read from the file for the parameter name
 * specified.  If a match is found, it returns the value associated
 * with the parameter name.
 *
 * \param parameter the name of the parameter.
 * \param value pointer to an integer to store the parameter value.
 * \return true if successful.
*/
bool Parameters::GetValue(const char * parameter, int * value) {
	number_parameters_iterator_ = number_parameters_.find(parameter);
	// If the iterator is not set to the end, then we found a match
	if (number_parameters_iterator_ != number_parameters_.end()) {
		*value = (int) number_parameters_iterator_->second;
		return true;
	}
	// Otherwise, no match found
	else {
		return false;
	}
}

/**
 * \brief Get the matching float for the specified parameter.
 *
 * Searches the number parameters that were read from the file for the parameter name
 * specified.  If a match is found, it returns the value associated
 * with the parameter name.
 *
 * \param parameter the name of the parameter.
 * \param value pointer to a float to store the parameter value.
 * \return true if successful.
*/
bool Parameters::GetValue(const char * parameter, float * value) {
	number_parameters_iterator_ = number_parameters_.find(parameter);
	// If the iterator is not set to the end, then we found a match
	if (number_parameters_iterator_ != number_parameters_.end()) {
		*value = (float) number_parameters_iterator_->second;
		return true;
	}
	// Otherwise, no match found
	else {
		return false;
	}
}

/**
 * \brief Get the matching double for the specified parameter.
 *
 * Searches the number parameters that were read from the file for the parameter name
 * specified.  If a match is found, it returns the value associated
 * with the parameter name.
 *
 * \param parameter the name of the parameter.
 * \param value pointer to a double to store the parameter value.
 * \return true if successful.
*/
bool Parameters::GetValue(const char * parameter, double * value) {
	number_parameters_iterator_ = number_parameters_.find(parameter);
	// If the iterator is not set to the end, then we found a match
	if (number_parameters_iterator_ != number_parameters_.end()) {
		*value = (double) number_parameters_iterator_->second;
		return true;
	}
	// Otherwise, no match found
	else {
		return false;
	}
}
