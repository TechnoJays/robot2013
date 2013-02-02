#include "WPILib.h"
#include "datalog.h"

/**
 * \def GetMsecTime()
 * \brief A macro that returns the current processor time in milliseconds
 */
#define GetMsecTime()           (GetFPGATime()/1000)

/**
 * \brief Open a file with the mode "a+" for logging.
 *
 * \param path the path and filename of the log to open/create.
*/
DataLog::DataLog(const char * path) {
	file_ = NULL;
	file_opened_ = false;
	DataLog::Open(path);
}

/**
 * \brief Open a file with the specified mode for logging.
 *
 * \param path the path and filename of the log to open/create.
 * \param mode the file access mode.
*/
DataLog::DataLog(const char * path, const char * mode) {
	file_ = NULL;
	file_opened_ = false;
	DataLog::Open(path, mode);
}

/**
 * \brief Open the default file "datalog.txt" with the mode "w" for logging.
*/
DataLog::DataLog() {
	file_ = NULL;
	file_opened_ = false;
	DataLog::Open("datalog.txt", "w");
}

/**
 * \brief Delete the file object.
*/
DataLog::~DataLog() {
	SafeDelete(file_);
}

/**
 * \brief Open a file with the mode "w".
 *
 * \param path the path and filename of the log to open/create.
 * \return true if successful.
*/
bool DataLog::Open(const char * path) {
	if (path == NULL) {
        file_opened_ = false;
		return false;
	}
	
	file_ = fopen(path, "w");
	if (file_ != NULL) {
        file_opened_ = true;
		return true;
	}
	else {
        file_opened_ = false;
		return false;
	}
}

/**
 * \brief Open a file with the specified mode.
 *
 * \param path the path and filename of the file to open/create.
 * \param mode the file access mode.
 * \return true if successful.
*/
bool DataLog::Open(const char * path, const char * mode) {
	if ((path == NULL) || (mode == NULL)) {
        file_opened_ = false;
		return false;
	}

	file_ = fopen(path, mode);
	if (file_ != NULL) {
        file_opened_ = true;
		return true;
	}
	else {
        file_opened_ = false;
		return false;
	}

}

/**
 * \brief Close the file.
*/
void DataLog::Close() {
	if (file_ != NULL) {
		fclose(file_);
		file_opened_ = false;
	}
}

/**
 * \brief Write a null terminated line to the file.
 *
 * The line must also include the carriage return.
 *
 * \param line the text to write to the log file.
 * \param timestamp true if a timestamp should be prepended to the line.
*/
void DataLog::WriteLine(const char * line, bool timestamp) {
	if (file_ != NULL) {
		if (timestamp) {
			UINT32 time = GetMsecTime();
			fprintf(file_, "[%d] ", time);
		}
		fputs(line, file_);
		fflush(file_);
	}
}

/**
 * \brief Write a parameter/value (char*) pair to the log file.
 *
 * Carriage return automatically added, no null terminators required.
 *
 * \param parameter the label/name of the parameter.
 * \param value the text value of the parameter.
 * \param timestamp true if a timestamp should be prepended to the line.
*/
void DataLog::WriteValue(const char * parameter, const char * value, bool timestamp) {
	if ((file_ != NULL) && (parameter != NULL) && (value != NULL)) {
		if (timestamp) {
			UINT32 time = GetMsecTime();
			fprintf(file_, "[%d] ", time);
		}
		fprintf(file_, "%s = %s\n", parameter, value);
		fflush(file_);
	}
}

/**
 * \brief Write a parameter/value (int) pair to the log file.
 *
 * \param parameter the label/name of the parameter.
 * \param value the value of the parameter.
 * \param timestamp true if a timestamp should be prepended to the line.
*/
void DataLog::WriteValue(const char * parameter, int value, bool timestamp) {
	if ((file_ != NULL) && (parameter != NULL)) {
		if (timestamp) {
			UINT32 time = GetMsecTime();
			fprintf(file_, "[%d] ", time);
		}
		fprintf(file_, "%s = %d\n", parameter, value);
		fflush(file_);
	}	
}

/**
 * \brief Write a parameter/value (float) pair to the log file.
 *
 * \param parameter the label/name of the parameter.
 * \param value the value of the parameter.
 * \param timestamp true if a timestamp should be prepended to the line.
*/
void DataLog::WriteValue(const char * parameter, float value, bool timestamp) {
	if ((file_ != NULL) && (parameter != NULL)) {
		if (timestamp) {
			UINT32 time = GetMsecTime();
			fprintf(file_, "[%d] ", time);
		}
		fprintf(file_, "%s = %f\n", parameter, value);
		fflush(file_);
	}		
}

/**
 * \brief Write a parameter/value (double) pair to the log file.
 *
 * \param parameter the label/name of the parameter.
 * \param value the value of the parameter.
 * \param timestamp true if a timestamp should be prepended to the line.
*/
void DataLog::WriteValue(const char * parameter, double value, bool timestamp) {
	if ((file_ != NULL) && (parameter != NULL)) {
		if (timestamp) {
			UINT32 time = GetMsecTime();
			fprintf(file_, "[%d] ", time);
		}
		fprintf(file_, "%s = %f\n", parameter, value);
		fflush(file_);
	}	
}
