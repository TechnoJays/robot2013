#ifndef PARAMETERS_H_
#define PARAMETERS_H_

#include <stdio.h>
#include <string>
#include <string.h>
#include <map>
#include "common.h"

/**
 * \class Parameters
 * \brief Reads parameter values from a file on disk into memory.
 * 
 * Provides a simple interface to read specific name/value pairs
 * from a file.
 */
class Parameters {

public:
	// Public methods
	Parameters();
	Parameters(const char * path);
	~Parameters();
	bool Open(const char * path);
	void Close();
	bool ReadValues();
	bool GetValue(const char * parameter, char * value);
	bool GetValue(const char * parameter, int * value);
	bool GetValue(const char * parameter, float * value);
	bool GetValue(const char * parameter, double * value);

	// Public member variables
	bool file_opened_;	///< true if the file is open

private:
	// Private typedefs
	/**
	 * \typedef std::map<std::string, std::string>
	 * \brief A map of name/value pairs for values that are strings.
	 */
	typedef std::map<std::string, std::string> StringMap;
	/**
	 * \typedef std::map<std::string, float>
	 * \brief A map of name/value pairs for values that are numbers.
	 */
	typedef std::map<std::string, float> NumberMap;

	// Private member objects
	FILE *file_;	///< the file to read parameters from
	
	// Private members variables
	StringMap string_parameters_;						///< contains parameters read from the file that are strings
	StringMap::iterator string_parameters_iterator_;	///< iterator of string parameters
	NumberMap number_parameters_;						///< contains parameters read from the file that are numerical
	NumberMap::iterator number_parameters_iterator_;	///< iterator of numerical parameters
};

#endif
