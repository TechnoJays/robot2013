#include "autoscript.h"
#include <dirent.h>
#include <string.h>


/**
 * \brief Open a script file with the mode "r" to read auto commands.
 *
 * \param path the path and filename of the autoscript file.
*/
AutoScript::AutoScript(const char * path) {
	file_ = NULL;
	file_opened_ = false;
	AutoScript::Open(path);
}

/**
 * \brief Create a new AutoScript object, but wait for the file to be loaded using a call to Open().
*/
AutoScript::AutoScript() {
	file_ = NULL;
	file_opened_ = false;
}

/**
 * \brief Deletes the file object.
*/
AutoScript::~AutoScript() {
	SafeDelete(file_);
}

/**
 * \brief Open a script file with the mode "r".
 *
 * \param path the path and filename of the autoscript file.
 * \return true if successful.
*/
bool AutoScript::Open(const char * path) {
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
 * \brief Close the autoscript file.
*/
void AutoScript::Close() {
	if (file_ != NULL) {
		fclose(file_);
		file_opened_ = false;
	}
}

/**
 * \brief Read all autoscript commands from the file.
 *
 * Reads the entire autoscript file formatted as a comma separated value (CSV) file.
 * The commands are stored as structs in a vector.
 *
 * \return true if successful.
*/
bool AutoScript::ReadScript() {
	char buffer[256] = {0};		///< buffer for file reading
	char c[255] = {0};			///< command string buffer
	float p1, p2, p3, p4, p5;	///< command parameter values
	char *current_token;		///< string token pointer
	int param_index = 0;		///< number of parameters parsed per line

	// Clear out any old script data
	autoscript_commands.clear();
	command_iterator = autoscript_commands.begin();
	
	if (file_opened_ && file_ != NULL) {
		// Loop while there's data to read
		while (fgets(buffer, 255, file_) != NULL) {
			// Reset temporary variables
			c[0] = 0;
			p1 = -9999;
			p2 = -9999;
			p3 = -9999;
			p4 = -9999;
			p5 = -9999;
			
			// Split the current line by commas
			current_token = strtok(buffer, " ,");
			param_index = 0;
			while (current_token != NULL) {
				// Store each parameter one at a time into the temporary variables
				switch(param_index) {
					case 0:
						sscanf(current_token, "%s", c);
						CharArrayToLower(current_token);
						break;
					case 1:
						sscanf(current_token, "%f", &p1);
						break;
					case 2:
						sscanf(current_token, "%f", &p2);
						break;
					case 3:
						sscanf(current_token, "%f", &p3);
						break;
					case 4:
						sscanf(current_token, "%f", &p4);
						break;
					case 5:
						sscanf(current_token, "%f", &p5);
						break;
					default:
						break;
				}
				current_token = strtok(NULL, " ,");
				param_index++;
			}
			// If we read some parameters, store the command into the vector
			if (param_index > 0) {
				autoscript_command current_command(c, p1, p2, p3, p4, p5);
				autoscript_commands.push_back(current_command);
			}
		}

		// Create an iterator to the vector for later use
		command_iterator = autoscript_commands.begin();		
		return true;
	}
	else {
		return false;
	}
}

/**
 * \brief Get a list of AutoScript files in the current directory.
 *
 * Get a list of AutoScript files in the current directory.
 *
 * \param files a string vector of AutoScript file names.
 * \return the number of files found.
*/
int AutoScript::GetAvailableScripts(std::vector<std::string> &files) {
	int file_count_ = 0;
	DIR *dp;
	struct dirent *dirp;
	char *extension_ptr;
	char ext[10];

	// Open the current directory - '/' on the cRIO
	if((dp  = opendir(".")) == NULL) {
		return -1;
	}

	// Loop through all the files found, and add those with a '.as' extension to the autoscript file list
	while ((dirp = readdir(dp)) != NULL) {
		extension_ptr=strrchr(dirp->d_name,'.');
		if (extension_ptr != NULL) {
			ext[0] = 0;
			strncpy(ext, extension_ptr+1, sizeof(ext));
			if (strlen(ext) > 0 && strncmp(ext, "as", 10) == 0) {
				files.push_back(std::string(dirp->d_name));
				file_count_++;
			}
		}
	}
	
	// Close the current directory and return the number of '.as' files found
	closedir(dp);
	return file_count_;
}

/**
 * \brief Get the next autoscript command.
 *
 * Gets the next command from the autoscript file.
 *
 * \return autoscript_command with the next command.
*/
autoscript_command AutoScript::GetNextCommand() {
	// If we're not at the end of the iterator, retrieve and return the next command
	if (command_iterator != autoscript_commands.end()) {
		autoscript_command c = *command_iterator;
		command_iterator++;
		return c;
	}
	// Otherwise return an 'end' command
	else {
		autoscript_command end_command("end", -9999, -9999, -9999, -9999, -9999);
		return end_command;
	}
}

/**
 * \brief Get the specified autoscript command.
 *
 * Gets the a specified command from the autoscript file.
 *
 * \param command_index the index of the command to retrieve.
 * \return autoscript_command with the specified command.
*/
autoscript_command AutoScript::GetCommand(unsigned int command_index) {
	// If the index is valid in the vector, retrieve and return the specified command
	if (command_index < autoscript_commands.size()) {
		return autoscript_commands[command_index];
	}
	// Otherwise return an 'invalid' command
	else {
		autoscript_command end_command("invalid", -9999, -9999, -9999, -9999, -9999);
		return end_command;
	}
}
