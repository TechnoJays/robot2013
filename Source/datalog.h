#ifndef DATALOG_H_
#define DATALOG_H_

#include <stdio.h>
#include "common.h"

/**
 * \class DataLog
 * \brief Writes log messages to a text file.
 * 
 * Automatically formats and writes various types of log messages to a log file.
 */
class DataLog {

public:	
	// Public methods
	DataLog(const char * path);
	DataLog(const char * path, const char * mode);
	DataLog();
	~DataLog();
	bool Open(const char * path);
	bool Open(const char * path, const char * mode);
	void Close();
	void WriteLine(const char * line, bool timestamp=false);
	void WriteValue(const char * parameter, const char * value, bool timestamp=false);
	void WriteValue(const char * parameter, int value, bool timestamp=false);
	void WriteValue(const char * parameter, float value, bool timestamp=false);
	void WriteValue(const char * parameter, double value, bool timestamp=false);

	// Public member variables
	bool file_opened_;	///< true if the output file is open

private:
	// Private member objects
	FILE *file_;	///< the file to write log data to
};

#endif
