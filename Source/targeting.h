#ifndef TARGETING_H_
#define TARGETING_H_

#include "Vision2009/VisionAPI.h" 
#include "common.h"

// Forward class definitions
class Parameters;
class DataLog;

/**
 * \class Targeting
 * \brief Finds and analyzes targets.
 * 
 * Provides a clean method of searching, filtering, and reporting
 * targets and their information.
 */
class Targeting {

public:	
	// Public methods
	Targeting();
	Targeting(bool logging_enabled);
	Targeting(const char * parameters);
	Targeting(const char * parameters, bool logging_enabled);
	~Targeting();
	bool LoadParameters();
	void SetRobotState(ProgramState state);
	void SetLogState(bool state);
	double GetAngleOfTarget(ParticleAnalysisReport *target);
	double GetCameraDistanceToTarget(ParticleAnalysisReport *target);
	double GetCameraHeightOfTarget(ParticleAnalysisReport *target);
	double GetFOVPercentageOfTarget(ParticleAnalysisReport *target);
	bool GetTargets(std::vector<ParticleAnalysisReport> &report);
	void InitializeCamera();
	bool StartSearching();
	bool StopSearching();

	// Public member variables
	bool camera_enabled_;		///< true if the camera is present

private:
	enum ThresholdType {
		kHSV,
		kHSL,
		kRGB
	};
	
	// Private methods
	static int CompareTargets(ParticleAnalysisReport t1, ParticleAnalysisReport t2);
	static int s_FindTargetsTask(Targeting *this_pointer);
	int FindTargetsTask();
	void GenerateFilename(char * prefix, char * suffix, int length, char * filename);
	void Initialize(const char * parameters, bool logging_enabled);

	// Private member objects
	Task find_targets_task_;							///< task object used to spawn the FindTargetsTask() function in a separate thread
	std::vector<ParticleAnalysisReport> *particle_report_;	///< vector of particle reports returned from the FindTargetsTask() function
	DataLog *log_;										///< log object used to log data or status comments to a file
	Parameters *parameters_;							///< parameters object used to load targeting parameters from a file

	// Private parameters
	char *camera_ip_address_;						///< the IP address of the camera we are connecting to
	float camera_view_angle_;						///< the viewing angle of the camera, used in various calculations
	int camera_resolution_;							///< the resolution of the images taken by the camera
	int frames_per_second_;							///< the FPS of the camera
	int color_level_;								///< the color level of the images taken by the camera
	int white_balance_;								///< the white balance mode of the camera
	int brightness_;								///< the brightness of hte images taken by the camera
	int compression_;								///< the compression percentage of the images taken by the camera
	int exposure_;									///< the exposure model of the camera
	double angle_of_target_offset_;					///< the offset that should be added to the 'degrees off target' calculation due to physical error in the robot
	int threshold_type_;							///< the type of image color threshold filter being used
	int threshold_plane_1_low_;						///< lower boundary for the RGB/HSL filter on plane 1
	int threshold_plane_1_high_;					///< upper boundary for the RGB/HSL filter on plane 1
	int threshold_plane_2_low_;						///< lower boundary for the RGB/HSL filter on plane 2
	int threshold_plane_2_high_;					///< upper boundary for the RGB/HSL filter on plane 2
	int threshold_plane_3_low_;						///< lower boundary for the RGB/HSL filter on plane 3
	int threshold_plane_3_high_;					///< upper boundary for the RGB/HSL filter on plane 3
	int particle_filter_filled_minimum_;			///< lower boundary for the particle filter (rectangle) percentage of the particle Area in relation to its Particle and Holes Area
	int particle_filter_filled_maximum_;			///< upper boundary for the particle filter (rectangle) percentage of the particle Area in relation to its Particle and Holes Area
	float target_rectangle_ratio_minimum_;			///< the lower boundary for a rectangle ratio
	float target_rectangle_ratio_maximum_;			///< the upper boundary for a rectangle ratio
	float target_rectangle_ratio_threshold_;		///< the rectangle ratio threshold used when identifying rectangles (high/medium/low)
	float target_rectangle_ratio_high_;				///< the rectangle ratio for the high height goal
	float target_rectangle_ratio_medium_;			///< the rectangle ratio for the medium height goal
	float target_rectangle_ratio_low_;				///< the rectangle ratio for the low height goal
	float target_rectangle_score_threshold_;		///< the lower threshold to accept from the target rectangle score
	
	// Private member variables
	int camera_horizontal_width_in_pixels_;	///< image width in pixels
	int camera_vertical_height_in_pixels_;	///< image height in pixels
	SEM_ID find_targets_semaphore_;			///< semaphore used to lock variables/objects that are shared between two threads
	bool sample_images_stored_;				///< true if 1 set of sample images have been stored
	bool camera_initialized_;				///< true if the camera is initialized
	bool log_enabled_;						///< true if logging is enabled
	char *parameters_file_;					///< path and filename of the parameter file to read
	ProgramState robot_state_;				///< current state of the robot obtained from the field
};

#endif
