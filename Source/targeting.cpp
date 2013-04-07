#include <cmath>
#include "WPILib.h"
#include "Vision/RGBImage.h"
#include "targeting.h"
#include "parameters.h"
#include "datalog.h"

/**
 * \def GetMsecTime()
 * \brief A macro that returns the current processor time in milliseconds
 */
#define GetMsecTime()           (GetFPGATime()/1000)

/**
 * \def PI
 * \brief the value of Pi to 8 decimal places.
 */
#define PI 3.141592653

/**
 * \brief Create and initialize the targeting system.
 *
 * Use the default parameter file "targeting.par" and logging is disabled.
*/
Targeting::Targeting()
	: find_targets_task_("findtargets", (FUNCPTR) s_FindTargetsTask)
{
	Initialize("targeting.par", false);
}

/**
 * \brief Create and initialize the targeting system.
 *
 * Use the default parameter file "targeting.par" and enable/disable
 * logging based on the parameter.
 *
 * \param logging_enabled true if logging is enabled.
*/
Targeting::Targeting(bool logging_enabled)
	: find_targets_task_("findtargets", (FUNCPTR) s_FindTargetsTask)
{

	Initialize("targeting.par", logging_enabled);
}

/**
 * \brief Create and initialize the targeting system.
 *
 * Use the user specified parameter file and logging is disabled.
 *
 * \param parameters targeting parameter file path and name.
*/
Targeting::Targeting(const char * parameters)
	: find_targets_task_("findtargets", (FUNCPTR) s_FindTargetsTask)
{

	Initialize(parameters, false);
}

/**
 * \brief Create and initialize the targeting system.
 *
 * Use the user specified parameter file and enable/disable
 * logging based on the parameter.
 *
 * \param parameters targeting parameter file path and name.
 * \param logging_enabled true if logging is enabled.
*/
Targeting::Targeting(const char * parameters, bool logging_enabled)
	: find_targets_task_("findtargets", (FUNCPTR) s_FindTargetsTask)
{

	Initialize(parameters, logging_enabled);
}

/**
 * \brief Delete and clear all objects and pointers.
*/
Targeting::~Targeting() {
	if (log_ != NULL) {
		log_->Close();
	}
	SafeDelete(log_);
	SafeDelete(parameters_);
	if (find_targets_task_.Verify()) {
		find_targets_task_.Stop();
	}
	SafeDelete(particle_report_);
}

/**
 * \brief Initialize the Targeting object.
 *
 * Create member objects, initialize default values, read parameters from the param file.
 *
 * \param parameters targeting parameter file path and name.
*/
void Targeting::Initialize(const char * parameters, bool logging_enabled) {
	// Create the task semaphore before doing anything
	find_targets_semaphore_ = semMCreate(SEM_Q_PRIORITY | SEM_INVERSION_SAFE | SEM_DELETE_SAFE);

	// Initialize public member variables
	camera_enabled_ = false;
		
	// Initialize private member objects
	log_ = NULL;
	parameters_ = NULL;
	CRITICAL_REGION(find_targets_semaphore_)
		particle_report_ = NULL;
	END_REGION
	
	// Initialize private parameters
	camera_view_angle_ = 43.5;
	camera_resolution_ = 2;
	frames_per_second_ = 5;
	color_level_ = 50;
	white_balance_ = 0;
	brightness_ = 100;
	compression_ = 30;
	exposure_ = 0;
	angle_of_target_horiztonal_offset_ = 0.0;
	angle_of_target_vertical_offset_ = 0.0;
	angle_of_target_distance_offset_ = 0.0;
	threshold_type_ = 0;
	threshold_plane_1_low_ = 0;
	threshold_plane_1_high_ = 50;
	threshold_plane_2_low_ = 50;
	threshold_plane_2_high_ = 255;
	threshold_plane_3_low_ = 0;
	threshold_plane_3_high_ = 50;
	particle_filter_filled_minimum_ = 35;
	particle_filter_filled_maximum_ = 65;
	target_rectangle_ratio_minimum_ = 1.0;
	target_rectangle_ratio_maximum_ = 3.2;
	target_rectangle_ratio_threshold_ = 0.4;
	target_rectangle_ratio_high_ = (62.0 / 20.0);
	target_rectangle_ratio_medium_ = (62.0 / 29.0);
	target_rectangle_ratio_low_ = (37.0 / 32.0);
	target_rectangle_score_threshold_ = 80.0;

	// Initialize private member variables
	log_enabled_ = false;
	camera_initialized_ = false;
	sample_images_stored_ = false;
	camera_horizontal_width_in_pixels_ = 0;
	camera_vertical_height_in_pixels_ = 0;
	robot_state_ = kDisabled;
	
	// Create a new data log object
	log_ = new DataLog("targeting.log");

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
bool Targeting::LoadParameters() {
	// Define and initialize local variables
	bool parameters_read = false;	// This should default to false
	int camera_present = 0;
	
	// Close and delete old objects
	SafeDelete(parameters_);
	
	// Attempt to read the parameters file
	parameters_ = new Parameters(parameters_file_);
	if (parameters_ != NULL && parameters_->file_opened_) {
		parameters_read = parameters_->ReadValues();
		parameters_->Close();
	}
	
	if (log_enabled_) {
		if (parameters_read)
			log_->WriteLine("Targeting parameters loaded successfully\n");
		else
			log_->WriteLine("Targeting parameters failed to read\n");
	}

	// Set targeting variables based on the parameters file
	if (parameters_read) {
		parameters_->GetValue("CAMERA_VIEW_ANGLE", &camera_view_angle_);
		parameters_->GetValue("CAMERA_PRESENT", &camera_present);
		parameters_->GetValue("CAMERA_RESOLUTION", &camera_resolution_);
		parameters_->GetValue("FRAMES_PER_SECOND", &frames_per_second_);
		parameters_->GetValue("COLOR_LEVEL", &color_level_);
		parameters_->GetValue("WHITE_BALANCE", &white_balance_);
		parameters_->GetValue("BRIGHTNESS", &brightness_);
		parameters_->GetValue("COMPRESSION", &compression_);
		parameters_->GetValue("EXPOSURE", &exposure_);
		parameters_->GetValue("ANGLE_OF_TARGET_HORIZONTAL_OFFSET", &angle_of_target_horiztonal_offset_);
		parameters_->GetValue("ANGLE_OF_TARGET_VERTICAL_OFFSET", &angle_of_target_vertical_offset_);
		parameters_->GetValue("ANGLE_OF_TARGET_DISTANCE_OFFSET", &angle_of_target_distance_offset_);
		parameters_->GetValue("THRESHOLD_TYPE", &threshold_type_);
		parameters_->GetValue("THRESHOLD_PLANE_1_LOW", &threshold_plane_1_low_);
		parameters_->GetValue("THRESHOLD_PLANE_1_HIGH", &threshold_plane_1_high_);
		parameters_->GetValue("THRESHOLD_PLANE_2_LOW", &threshold_plane_2_low_);
		parameters_->GetValue("THRESHOLD_PLANE_2_HIGH", &threshold_plane_2_high_);
		parameters_->GetValue("THRESHOLD_PLANE_3_LOW", &threshold_plane_3_low_);
		parameters_->GetValue("THRESHOLD_PLANE_3_HIGH", &threshold_plane_3_high_);
		parameters_->GetValue("PARTICLE_FILTER_FILLED_MINIMUM", &particle_filter_filled_minimum_);
		parameters_->GetValue("PARTICLE_FILTER_FILLED_MAXIMUM", &particle_filter_filled_maximum_);
		parameters_->GetValue("TARGET_RECTANGLE_RATIO_MINIMUM",&target_rectangle_ratio_minimum_);
		parameters_->GetValue("TARGET_RECTANGLE_RATIO_MAXIMUM",&target_rectangle_ratio_maximum_);
		parameters_->GetValue("TARGET_RECTANGLE_RATIO_THRESHOLD",&target_rectangle_ratio_threshold_);
		parameters_->GetValue("TARGET_RECTANGLE_RATIO_HIGH",&target_rectangle_ratio_high_);
		parameters_->GetValue("TARGET_RECTANGLE_RATIO_MEDIUM",&target_rectangle_ratio_medium_);
		parameters_->GetValue("TARGET_RECTANGLE_RATIO_LOW",&target_rectangle_ratio_low_);
		parameters_->GetValue("TARGET_RECTANGLE_SCORE_THRESHOLD",&target_rectangle_score_threshold_);
	}

	// Check if the camera is enabled or not
	if (camera_present)
		camera_enabled_ = true;
	else
		camera_enabled_ = false;

	if (log_enabled_) {
		if (camera_enabled_) {
			log_->WriteLine("Camera enabled\n");
		}
		else {
			log_->WriteLine("Camera disabled\n");
		}
	}

	camera_initialized_ = false;

	return parameters_read;
}

/**
 * \brief Set the current state of the robot and perform any actions necessary during mode changes.
 *
 * \param state current robot state.
*/
void Targeting::SetRobotState(ProgramState state) {
	robot_state_ = state;
	
	// Enable or disable target searching depending on robot state
	if (state == kDisabled) {
		StopSearching();
	}
	else if (state == kTeleop) {
		StartSearching();
	}
	else if (state == kAutonomous) {
		StartSearching();
	}	
}

/**
 * \brief Enable or disable logging for this object.
 *
 * \param state true if logging should be enabled.
*/
void Targeting::SetLogState(bool state) {
	if (state && log_ != NULL)
		log_enabled_ = true;
	else
		log_enabled_ = false;
}

/**
 * \brief Get the number of degrees the robot is off target.
 *
 * \param target pointer to a ParticleAnalysisReport of an image target.
 * \return the number of degrees off target.
*/
double Targeting::GetHorizontalAngleOfTarget(ParticleAnalysisReport *target) {
	double degrees_off_center = (-(camera_view_angle_ * ((target->imageWidth / 2.0) - target->center_mass_x)) / target->imageWidth) + angle_of_target_horiztonal_offset_;
	return degrees_off_center;
}

/**
 * \brief Get the vertical angle in degrees from the robot to the target.
 *
 * \param target pointer to a ParticleAnalysisReport of an image target.
 * \return the angle in degrees of the target.
*/
double Targeting::GetVerticalAngleOfTarget(ParticleAnalysisReport *target) {
	double distance = GetCameraDistanceToTarget(target) + angle_of_target_distance_offset_;
	double height = GetCameraHeightOfTarget(target);
	double angle = (atan(height/distance) * 180.0 / PI) + angle_of_target_vertical_offset_;
	return angle;
}

/**
 * \brief Get the estimated distance in feet to the provided particle target.
 *
 * \param target pointer to a ParticleAnalysisReport of an image target.
 * \return the number of feet away from the target.
*/
double Targeting::GetCameraDistanceToTarget(ParticleAnalysisReport *target) {
	int observed_target_width = target->boundingRect.width;
	double rectangle_width = 2.0 * camera_horizontal_width_in_pixels_ / observed_target_width;
	double distance = (rectangle_width / 2.0) / tan(((camera_view_angle_ * PI / 180.0) / 2.0));	
	return distance;
}

/**
 * \brief Get the estimated height in feet off the floor of the provided particle target.
 *
 * The height value is the center of mass for the target.
 *
 * \param target pointer to a ParticleAnalysisReport of an image target.
 * \return the height in feet of the target.
*/
double Targeting::GetCameraHeightOfTarget(ParticleAnalysisReport *target) {
	// Calculate the aspect ratio of the rectangle
	float rectangle_ratio = (float) target->boundingRect.width / (float) target->boundingRect.height;
	
	// Compare the rectangle aspect ratio to known goals
	if (fabs(rectangle_ratio - target_rectangle_ratio_high_) < target_rectangle_ratio_threshold_) {
		return 9.177083;	// 104 1/8 + (12/2) inches / 12" per foot
	}
	else if (fabs(rectangle_ratio - target_rectangle_ratio_medium_) < target_rectangle_ratio_threshold_) {
		return 8.2604167;	// 88 5/8 + (21/2) inches / 12" per foot
	}
	else if (fabs(rectangle_ratio - target_rectangle_ratio_low_) < target_rectangle_ratio_threshold_) {
		return 2.583;	// 19+(24/2) inches / 12" per foot
	}
	else {
		return 0.0;
	}
	
	// Old method, when the target width was known and constant
	// We know the actual height of the rectangle, so we can calculate how many pixels equal 1 inch
	// by dividing the rectangle's pixel height by the known height
	//double pixels_per_inch = target->boundingRect.height / (target_width_ * 12.0);
	// Calculate the height of the target from the bottom of the image to center mass
	// Determine the number of vertical pixels, and divide by the number of pixels per inch to get the height
	// This isn't the actual height since it goes from the bottom of the image and not from the floor
	//double height = ((camera_vertical_height_in_pixels_ - target->center_mass_y) / pixels_per_inch) / 12.0;
	//return height;
}

/**
 * \brief Get the height off the floor of the provided particle target as an enumeration.
 *
 * \param target pointer to a ParticleAnalysisReport of an image target.
 * \return the height as an enumeration.
*/
Targeting::TargetHeight Targeting::GetEnumHeightOfTarget(ParticleAnalysisReport *target) {
	double height = GetCameraHeightOfTarget(target);
	
	// Convert a height value to an enumeration
	if (height == 9.177083) {
		return Targeting::kHigh;
	}
	else if (height == 8.2604167) {
		return Targeting::kMedium;
	}
	else if (height == 2.583) {
		return Targeting::kLow;
	}
	else {
		return Targeting::kUnknown;
	}
}

/**
 * \brief Get the height off the floor of the provided value as an enumeration.
 *
 * \param height the height in feet.
 * \return the height as an enumeration.
*/
Targeting::TargetHeight Targeting::GetEnumHeightOfTarget(double height) {
	// Convert a height value to an enumeration
	if (height == 9.177083) {
		return Targeting::kHigh;
	}
	else if (height == 8.2604167) {
		return Targeting::kMedium;
	}
	else if (height == 2.583) {
		return Targeting::kLow;
	}
	else {
		return Targeting::kUnknown;
	}
}

/**
 * \brief Get the height off the floor of the provided value as a character string.
 *
 * \param target_height the height as an enumeration.
 * \param buffer the characeter array to contain the height as a character array.
*/
void Targeting::GetStringHeightOfTarget(TargetHeight target_height, char *buffer) {
	// Convert a height enumeration to a string
	switch(target_height) {
	case Targeting::kHigh:
		strncpy(buffer, "High", sizeof(buffer));
		break;
	case Targeting::kMedium:
		strncpy(buffer, "Medium", sizeof(buffer));
		break;
	case Targeting::kLow:
		strncpy(buffer, "Low", sizeof(buffer));
		break;
	default:
		strncpy(buffer, "Unknown", sizeof(buffer));
		break;			
	}
}

/**
 * \brief Get the amount off target as a percentage of the camera field of view.
 *
 * \param target pointer to a ParticleAnalysisReport of an image target.
 * \return the percentage off target as it relates to the image.
*/
double Targeting::GetFOVPercentageOfTarget(ParticleAnalysisReport *target) {
	return target->center_mass_x_normalized;
}

/**
 * \brief Gets the latest target report from the targeting system.
 *
 * \param report a reference to a vector of ParticleAnalysisReport that will contain the reports.
 * \return true if successful.
*/
bool Targeting::GetTargets(vector<ParticleAnalysisReport> &report) {
	// Abort if we don't have the camera
	if (!camera_enabled_) {
		return false;
	}

	// Block other threads from accessing the particle report while it's copied
	CRITICAL_REGION(find_targets_semaphore_)
	if (particle_report_ != NULL) {
		// Copy the report to the referenced vector passed in
		report.resize(particle_report_->size());
		report.assign(particle_report_->begin(), particle_report_->end());
		return true;
	}
	else {
		return false;
	}
	END_REGION
}

/**
 * \brief Sets the camera settings using the values from the parameter file.
*/
void Targeting::InitializeCamera() {
	// Only continue if we haven't already done this
	if (camera_enabled_ && !camera_initialized_) {
		// Get a reference to the camera
		AxisCamera &axis_camera = AxisCamera::GetInstance();
				
		// Store the resolution for use in calculations
		switch ((AxisCameraParams::Resolution_t) camera_resolution_) {
			case AxisCameraParams::kResolution_640x480:
				camera_horizontal_width_in_pixels_ = 640;
				camera_vertical_height_in_pixels_ = 480;
				break;
			case AxisCameraParams::kResolution_640x360:
				camera_horizontal_width_in_pixels_ = 640;
				camera_vertical_height_in_pixels_ = 360;
				break;
			case AxisCameraParams::kResolution_320x240:
				camera_horizontal_width_in_pixels_ = 320;
				camera_vertical_height_in_pixels_ = 240;
				break;
			case AxisCameraParams::kResolution_160x120:
				camera_horizontal_width_in_pixels_ = 160;
				camera_vertical_height_in_pixels_ = 120;
				break;
			default:
				camera_horizontal_width_in_pixels_ = 320;
				camera_vertical_height_in_pixels_ = 240;
				break;
		}
		
		// Set the camera settings
		axis_camera.WriteBrightness(brightness_);
		axis_camera.WriteColorLevel(color_level_);
		axis_camera.WriteCompression(compression_);
		axis_camera.WriteExposureControl((AxisCameraParams::Exposure_t) exposure_);
		axis_camera.WriteMaxFPS(frames_per_second_);
		axis_camera.WriteResolution((AxisCameraParams::Resolution_t) camera_resolution_);
		axis_camera.WriteWhiteBalance((AxisCameraParams::WhiteBalance_t) white_balance_);
		camera_initialized_ = true;
	}
}

/**
 * \brief Starts taking images searching for targets.
 *
 * Spawns a new task in the background so that the process won't interfere
 * with the main control loop.
 *
 * \return true if successful.
*/
bool Targeting::StartSearching() {
	// Abort if the camera is disabled
	if (!camera_enabled_)
		return false;
	
	// Initialize the camera if it hasn't been done yet
	if (!camera_initialized_) {
		InitializeCamera();
	}
	
	// Start or restart the task
	if (find_targets_task_.Verify()) {
		if (!find_targets_task_.Restart()) {
			return false;
		}
		return true;
	}
	else {
		if (!find_targets_task_.Start((int)this)) {
			return false;
		}
		return true;
	}
}

/**
 * \brief Stops taking images and searching for targets.
 *
 * Stops the background task.
 *
 * \return true if successful.
*/
bool Targeting::StopSearching() {
	if (!camera_enabled_)
			return false;
	
	// Stop the task
	if (find_targets_task_.Verify()) {
		return find_targets_task_.Stop();
	}
	else {
		return false;
	}
}

/**
 * \brief Compares two targets to see which is higher.
 *
 * Used to sort targets based on their height.
 *
 * (int t1,int t2) { return (t1<t2)
 * 0,5	1
 * 5,0	0
 * 5,5	0
 *
 * 0,5	1
 * 5,0	-1
 * 5,5	0
 *
 * \param t1 the first Target.
 * \param t2 the second Target.
 * \return (1, 0, or -1) for the heights of t1 > t2, t1 == t2, and t1 < t2.
*/
int Targeting::CompareTargets(ParticleAnalysisReport t1, ParticleAnalysisReport t2)
{
	// Ascending order
	if (t1.center_mass_y > t2.center_mass_y) return 1;
	if (t1.center_mass_y < t2.center_mass_y) return -1;
	// Descending order, e.g., 5,4,3,2,1
	/*if (t1.center_mass_y < t2.center_mass_y) return 1;
	if (t1.center_mass_y > t2.center_mass_y) return -1;*/
	return 0;
}

/**
 * \brief Static interface for the FindTargetsTask function.
 *
 * Static interface that will cause an instantiation if necessary.
 * This function is used so that the actual task function doesn't need
 * to be static.
 *
 * \param this_pointer a pointer to this object.
 * \return the result of the spawned task.
*/
int Targeting::s_FindTargetsTask(Targeting *this_pointer) {
	return this_pointer->FindTargetsTask();
}

/**
 * \brief Takes images from the camera and searches for targets.
 *
 * Repeatedly finds, filters, and stores matching targets.
 * Images are taken repeatedly from the camera as fast as the loop can execute.
 * The images are filtered for a specific color (green).
 * The images are then filtered to remove noise and false positives.
 * A particle report is generated from the images and the results are stored in 
 * a shared variable (which is protected from simultaneous access using a semaphore). 
 * 
 * \return 0 on success (but the task should never finish on it's own).
*/
int Targeting::FindTargetsTask() {
	// Loop repeatedly
	while (true) {
		// Get a reference to the camera
		AxisCamera &axis_camera = AxisCamera::GetInstance();
		
		try {
			// Only get an image if it's one we haven't processed yet
			if (axis_camera.IsFreshImage()) {
				// Create the image objects and image pointers used in the processing below
				ColorImage *image = new ColorImage(IMAQ_IMAGE_RGB);
				BinaryImage *color_filtered_image = NULL;
				BinaryImage *large_objects_image = NULL;
				BinaryImage *convex_hull_image = NULL;
				//BinaryImage *particle_filtered_image = NULL;

				// Get an image from the camera
				if (axis_camera.GetImage(image)) {
					
					// Create the HSL/RGB threshold filter object
					Threshold threshold_ = Threshold(threshold_plane_1_low_, threshold_plane_1_high_,
														 threshold_plane_2_low_, threshold_plane_2_high_,
													 threshold_plane_3_low_, threshold_plane_3_high_);
					
					// Create a particle filter criteria based on rectangle dimensions
					//ParticleFilterCriteria2 criteria[] = {{IMAQ_MT_BOUNDING_RECT_WIDTH, particle_filter_width_minimum_, particle_filter_width_maximum_, false, false},
					//									  {IMAQ_MT_BOUNDING_RECT_HEIGHT, particle_filter_height_minimum_, particle_filter_height_maximum_, false, false}};
					//ParticleFilterCriteria2 criteria[] = {{IMAQ_MT_AREA_BY_PARTICLE_AND_HOLES_AREA, particle_filter_filled_minimum_, particle_filter_filled_maximum_, false, false}};

					
					// If an image was acquired, filter it based on HSL or RGB color values
					if (image != NULL) {
						// Store the very first image taken by the camera (unfiltered)
						// This will be helpful during practice and competitions to diagnose issues
						if (!sample_images_stored_) {
							char filename[20] = {0};
							Targeting::GenerateFilename("/1_", ".bmp", 4, filename);
							image->Write(filename);
							sample_images_stored_ = true;
						}

						// Perform an HSV filter
						if ((Targeting::ThresholdType) threshold_type_ == kHSV)
							color_filtered_image = image->ThresholdHSV(threshold_);
						// Perform an HSL filter
						else if ((Targeting::ThresholdType) threshold_type_ == kHSL)
							color_filtered_image = image->ThresholdHSL(threshold_);
						// Perform an RGB filter
						else
							color_filtered_image = image->ThresholdRGB(threshold_);
					}

					// Remove small objects, leaving only the larger blobs
					if (color_filtered_image != NULL) {
						/*Targeting::GenerateFilename("/2_", ".bmp", 4, filename);
						color_filtered_image->Write(filename);*/
			
						large_objects_image = color_filtered_image->RemoveSmallObjects(false, 2);
						SafeDelete(color_filtered_image);
					}

					// Perform a convex hull to 'fill-in' the blobs
					if (large_objects_image != NULL) {
						/*Targeting::GenerateFilename("/3_", ".bmp", 4, filename);
						large_objects_image->Write(filename);*/
						
						convex_hull_image = large_objects_image->ConvexHull(false);
						SafeDelete(large_objects_image);
					}
					
						/*Targeting::GenerateFilename("/4_", ".bmp", 4, filename);
						large_objects_image->Write(filename);*/
						
						/*particle_filtered_image = large_objects_image->ParticleFilter(criteria, 2);
						SafeDelete(large_objects_image);
					}*/
					
					//if (particle_filtered_image != NULL) {
						/*Targeting::GenerateFilename("/3_", ".bmp", 4, filename);
						particle_filtered_image->Write(filename);*/
						
						/*convex_hull_image = particle_filtered_image->ConvexHull(false);
						SafeDelete(particle_filtered_image);
					}*/
					
					// Get a particle report from the image
					if (convex_hull_image != NULL) {
						/*Targeting::GenerateFilename("/5_", ".bmp", 4, filename);
						convex_hull_image->Write(filename);*/
			
						// Delete the old report and create a new one
						// Then filter out the bad targets based on user parameters
						CRITICAL_REGION(find_targets_semaphore_)
						SafeDelete(particle_report_);
						particle_report_ = convex_hull_image->GetOrderedParticleAnalysisReports();
						SafeDelete(convex_hull_image);
						
						// Filter out bad targets, and sort by height
						if (particle_report_ != NULL && particle_report_->size() > 0) {
							vector<ParticleAnalysisReport>::iterator target_iterator = particle_report_->begin();
							while (target_iterator != particle_report_->end()) {
								// Calculate rectangle ratio
								float rectangle_ratio = (float) target_iterator->boundingRect.width / (float) target_iterator->boundingRect.height;
								// Calculate rectangle score
								float rectangle_area = 	(float) target_iterator->boundingRect.width * (float) target_iterator->boundingRect.height;
								float rectangle_score = (target_iterator->particleArea / rectangle_area) * 100.0;
								/*printf("particle location: %i, %i\n", target_iterator->center_mass_x, target_iterator->center_mass_y);
								printf("particle ratio: %5.2f\n", rectangle_ratio);
								printf("particle score: %5.2f\n", rectangle_score);*/
								// Delete bad targets
								if (rectangle_ratio < target_rectangle_ratio_minimum_ || rectangle_ratio > target_rectangle_ratio_maximum_ || rectangle_score < target_rectangle_score_threshold_) {
									target_iterator = particle_report_->erase(target_iterator);
									//printf("Deleting particle\n");
								}
								else {
									target_iterator++;
								}
							}
							// sort the list of targets by height
							sort(particle_report_->begin(), particle_report_->end(), Targeting::CompareTargets);
						}
						END_REGION
					}
				}
				
				// Delete the first image pointer
				SafeDelete(image);
			}
		}
		catch (exception& e) {
			printf("Exception in Find Targets Task: %s\n", e.what());			
		}		
	}
	return 0;
}

/**
 * \brief Generates a random filename.
 *
 * \param prefix a character array containing the desired prefix of the filename.
 * \param suffix a character array containing the desired suffix of the filename.
 * \param length the number of random characters in the filename.
 * \param filename an empty character array that will contain the result.
*/
void Targeting::GenerateFilename(char * prefix, char * suffix, int length, char * filename) {
	// Acceptable characters for a filename
	static const char alphanum[] = "0123456789abcdefghijklmnopqrstuvwxyz";
	
	// Copy the filename prefix to the filename character pointer/array
	int prefix_length = strlen(prefix);
	strcpy(filename, prefix);
	
	// Generate the random part of the filename
	for (int i = prefix_length; i<(length + prefix_length); i++) {
		filename[i] = alphanum[rand() % (sizeof(alphanum) - 1)];
	}
	
	// Copy the suffix, if there is one
	for (int i = length + prefix_length; i < ((int) strlen(suffix) + length + prefix_length); i++) {
		filename[i] = suffix[i-length-prefix_length];
	}
	
	// Null terminate the filename character array 
	filename[length + prefix_length + strlen(suffix)] = 0;
}
