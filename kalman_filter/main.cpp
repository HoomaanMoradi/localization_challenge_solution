#include <stdio.h>
#include <stdlib.h>
#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glut.h>
#include <math.h>
#include <vector>
#include "robot_defs.h"
#include "controller.h"
#include "main.h"
#include<eigen3/Eigen/Dense>
#include <fstream>
#include <iostream>

// Define global variables and structures for robot state and parameters
RobotState pose;     // Current robot pose (position and orientation) measured by odometry
FieldLocation  landmarkLocations[NUM_LANDMARKS];     // Global coordinates of markers
RobotParams robotParams;                // Parameters for robot and sensors noises
FieldLocation poseGlob;                 // Global Pose of the robot obtained by odometry
FieldLocation poseGlobFig;               // Global Pose of the robot obtained by odometry for plots
MarkerObservation markerOutput;          //Pose of the robot with respect to the markers in the robot's body frame
FieldLocation markerPose;                //Global Pose of the robot obtained by markers with respect to the markers
FieldLocation markerPoseGlob;            //Global Pose of the robot obtained by markers with respect to the origins
FieldLocation markerPoseGlobFig[NUM_LANDMARKS];         //Global Pose of the robot obtained by each marker with respect to the origins for plots


double angle_fov;          // Robot's field of view angle


// Kalman Filter matrices
Eigen::Matrix3d P; // State covariance matrix
Eigen::Matrix3d Q; // Odometry noise covariance matrix
Eigen::Matrix2d R; // Markers observations noise covariance matrix
Eigen::Matrix3d A; // State transition matrix
Eigen::Matrix<double, 2, 3> H; // Measurement matrix

Eigen::Vector2d z_marker;   // Marker measurement vector in global frame
Eigen::Vector2d z_odo;      // Odometry measurement vector in global frame

// Function to initialize Kalman Filter matrices
void initializeMatrices() {

  // Initialize Q, R, A, H, P matrices with appropriate values
  Q << 0.4, 0, 0,
       0, 0.4, 0,
       0, 0, 0.01;

  R << 0.1, 0,
       0, 0.1;

  A << 1, 0, 0,
       0, 1, 0,
       0, 0, 1;

  H << 1, 0, 0,
       0, 1, 0;

  P << 1, 0, 0,
       0, 1, 0,
       0, 0, 1;
}


// Function to limit the angle within [-pi, pi]
void angLimit(double& ang) {
  while (ang > M_PI) ang -= 2. * M_PI;
  while (ang < -M_PI) ang += 2. * M_PI;
}


/**
 * getRobotPositionEstimate()
 * This function is called by the controller to retrieve the current 
 * robot position estimate. 
 */
void getRobotPositionEstimate(RobotState& estimatePosn)
{
  // Return the current estimated robot position
  estimatePosn.x = poseGlob.x;
  estimatePosn.y = poseGlob.y;
  estimatePosn.theta = pose.theta;
}

/**
 * motionUpdate()
 * This function is called every time the position of the robot is
 * updated. The argument passed is the relative change in position of the 
 * robot in local robot coordinates (observed by odometry model), which 
 * may be subject to noise (according to motion model parameters).
 */
void motionUpdate(RobotState delta)
{
    //Update the pose of the robot in the body frame of the robot originated at the inital Pose of the robot measured by odometry with sensor noise
    pose.x += delta.x + robotParams.odom_noise_translation_from_translation;
    pose.y += delta.y + robotParams.odom_noise_translation_from_translation;
    pose.theta += delta.theta + robotParams.odom_noise_rotation_from_rotation;
    
    // Limit the angle to the specified range
    angLimit(pose.theta);

    //Update the global Pose of the robot with respect to the origin measured by odometry
    poseGlob.x = pose.x*cos(pose.theta); 
    poseGlob.y = pose.y*sin(pose.theta);
    
    //Store the global Pose of the robot with respect to the origin measured by odometry for plots
    poseGlobFig.x = poseGlob.x; 
    poseGlobFig.y = poseGlob.y;

    // Update Kalman Filter prediction step
    P = A * P * A.transpose() + Q;
   }

/**
 * sensorUpdate()
 * This function is called every time the robot detects one or more
 * landmarks in its field of view. The argument passed contains all 
 * marker obervations (marker index and position of marker in robot 
 * coordinates) for the current frame.
 */
void sensorUpdate(std::vector<MarkerObservation> observations)
{
    // Loop through each observed marker and update the Kalman Filter
    for (const auto& observation : observations) {

        //Add noise to the observations obtained by markes
        markerOutput.distance  = observation.distance + robotParams.sensor_noise_distance;
        markerOutput.orientation  = observation.orientation + robotParams.sensor_noise_orientation;

        //Update the pose of the robot in the marker frame obtained by marker observations 
        markerPose.x = markerOutput.distance * cos(markerOutput.orientation)*cos(pose.theta)- markerOutput.distance * sin(markerOutput.orientation)*sin(pose.theta);
        markerPose.y = markerOutput.distance * cos(markerOutput.orientation )* sin(pose.theta) + markerOutput.distance * sin(markerOutput.orientation )* cos(pose.theta);
 
        //Update the pose of the robot in origin frame obtained by marker observations
        markerPoseGlob.x = landmarkLocations[observation.markerIndex].x - markerPose.x;
        markerPoseGlob.y = landmarkLocations[observation.markerIndex].y - markerPose.y;
  
        //Store the pose of the robot in the global frame obtained by marker observation for plots
        markerPoseGlobFig[observation.markerIndex].x = markerPoseGlob.x;
        markerPoseGlobFig[observation.markerIndex].y = markerPoseGlob.y;

        // Update the measurement vector of the markers
        z_marker << markerPoseGlob.x, markerPoseGlob.y;

        // Update the measurement vector of the markers odometry
        z_odo << poseGlob.x, poseGlob.y;

        // Compute Kalman gain
        Eigen::Matrix<double, 3, 2> K = P * H.transpose() * (H * P * H.transpose() + R).inverse();

        // Compute innovation
        Eigen::Vector3d innovation = K * (z_marker - z_odo);

        //Update the global robot position of odometry by Kalman Filter
        poseGlob.x += innovation(0);
        poseGlob.y += innovation(1);
     
       // Update Kalman Filter covariance matrix
       P = (Eigen::Matrix3d::Identity() - K * H) * P;
    }
}

/**
 * myinit()
 * Initialization function that takes as input the initial 
 * robot state (position and orientation), and the locations
 * of each landmark (global x,y coordinates).
 */
void myinit(RobotState robotState, RobotParams robotParams, 
            FieldLocation markerLocations[NUM_LANDMARKS])
{
    //Initialize the robot sates
    pose.x = robotState.x;
    pose.y = robotState.y;
    pose.theta = robotState.theta;
    poseGlob.x = robotState.x*cos(robotState.theta);
    poseGlob.y = robotState.y*sin(robotState.theta);

    // Assuming that the angles are given in degrees
    robotParams.angle_fov = robotParams.angle_fov * M_PI / 180;
    angLimit(robotParams.angle_fov);
    
    // Assuming that the angles are given in degrees
    robotParams.odom_noise_rotation_from_rotation =
    robotParams.odom_noise_rotation_from_rotation * M_PI / 180;

    //limit the angle within [-pi, pi]
    angLimit(robotParams.odom_noise_rotation_from_rotation);     

    robotParams.odom_noise_rotation_from_translation =
    robotParams.odom_noise_rotation_from_translation * M_PI / 180;
    angLimit(robotParams.odom_noise_rotation_from_translation);

    robotParams.sensor_noise_orientation =
    robotParams.sensor_noise_orientation * M_PI / 180;
    angLimit(robotParams.sensor_noise_orientation);
    
    // Initialize marker locations with respect to origin
    for (size_t i = 0; i < NUM_LANDMARKS; i++) {
       landmarkLocations[i].x = markerLocations[i].x;
       landmarkLocations[i].y = markerLocations[i].y;
   };
   
   // Initialize Kalman Filter matrices
   initializeMatrices();

}

/**
 * mydisplay()
 * This function is called whenever the display is updated. The controller
 * will draw the estimated robot position after this function returns.
 */
void mydisplay()
{
    // Write the sensor data as well as the Kalman Filter output in a text file for plots
    std::ofstream outputFile("Figures.txt", std::ios::app);
    if (outputFile.is_open()) {
        // Write header line if the file is empty
        if (outputFile.tellp() == 0) {
            outputFile << "poseodom.x poseodom.y markerPose1.x markerPose1.y markerPose2.x markerPose2.y markerPose3.x markerPose3.y markerPose4.x markerPose4.y PoseKalman.x PoseKalman.y" << std::endl;
        }
        
        // Append varibles values
        outputFile << poseGlobFig.x << " " << poseGlobFig.y <<
                " " << markerPoseGlobFig[1].x << " " << markerPoseGlobFig[1].y <<
                " " << markerPoseGlobFig[2].x << " " << markerPoseGlobFig[2].y <<
                " " << markerPoseGlobFig[3].x << " " << markerPoseGlobFig[3].y <<
                " " << markerPoseGlobFig[4].x << " " << markerPoseGlobFig[4].y <<
                " " << poseGlob.x << " " << poseGlob.y<< std::endl;
        
        outputFile.close();

    } else {
        std::cerr << "Unable to open file for writing!" << std::endl;
    }

}

/**
 * mykeyboard()
 * This function is called whenever a keyboard key is pressed, after
 * the controller has processed the input. It receives the ASCII value 
 * of the key that was pressed.
 *
 * Return value: 1 if window re-draw requested, 0 otherwise
 */
int mykeyboard(unsigned char key)
{
	
	return 0;
}


/**
 * Main entrypoint for the program.
 */
int main (int argc, char ** argv)
{
    // Initialize world, sets initial robot position
    // calls myinit() before returning
    runMainLoop(argc, argv);

    return 0;
}

