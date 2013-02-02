#ifndef AUTOSCRIPT_H_
#define AUTOSCRIPT_H_

#include <stdio.h>
#include <string>
#include <string.h>
#include <vector>
#include "common.h"

/**
 * Data structure to store the information for an autoscript command.
 */
struct autoscript_command {
	char command[255];	///< auto command
	float param1;		///< auto command parameter 1
	float param2;
	float param3;
	float param4;
	float param5;
	autoscript_command(char *c, float p1, float p2, float p3, float p4, float p5):
		param1(p1), param2(p2), param3(p3), param4(p4), param5(p5) {strncpy(command, c, 254);}
	autoscript_command():
		param1(-9999), param2(-9999), param3(-9999), param4(-9999), param5(-9999) {command[0]=0;}
};

/**
 * \class AutoScript
 * \brief Reads autonomous robot sequences from a file into memory..
 * 
 * Provides a simple interface to read specific name/value pairs
 * from a file.
 */
class AutoScript {

public:
	// Public methods
	AutoScript();
	AutoScript(const char * path);
	~AutoScript();
	bool Open(const char * path);
	void Close();
	bool ReadScript();
	int GetAvailableScripts(std::vector<std::string> &files);
	autoscript_command GetNextCommand();
	autoscript_command GetCommand(unsigned int command_index);

	// Public member variables
	bool file_opened_;	///< true if the file is open

private:
	// Private member objects
	FILE *file_;	///< the file to read parameters from
	
	// Private members variables
	std::vector<autoscript_command> autoscript_commands;		///< stores the command structures
	std::vector<autoscript_command>::iterator command_iterator;	///< iterator for the command vector
};

#endif
