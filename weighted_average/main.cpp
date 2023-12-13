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
RobotState weightedPose;    // Weighted average of markers poses
FieldLocation  landmarkLocations[NUM_LANDMARKS];    // Global coordinates of markers
RobotParams robotParams;       // Parameters for robot and sensors noises
FieldLocation poseGlob;       // Global Pose of the robot obtained by odometry
MarkerObservation markerOutput;     //Pose of the robot with respect to the markers in the robot's body frame
FieldLocation markerPose;        //Global Pose of the robot obtained by markers with respect to the markers
FieldLocation markerPoseGlob;    //Global Pose of the robot obtained by markers with respect to the origins
FieldLocation markerPoseGlobFig[NUM_LANDMARKS];     //Global Pose of the robot obtained by each marker with respect to the origins for plots

bool flag;      // Flag indicating the estimation type (Weighted Average or corrected odometry)

double angle_fov;     // Robot's field of view angle

// Function to compute the weight based on the inverse of distance of the robot from markers
double computeWeight(const MarkerObservation& observation) {
    const double minDistance = 0.000000000001; // Avoid division by zero
    return 1.0 / std::max(minDistance, observation.distance);
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
  
  if (!flag) {                                //if at least we can have one observation from the markers
  estimatePosn.x = weightedPose.x;
  estimatePosn.y = weightedPose.y;
  } 
  else {                                      //if there is not any observation from the markers
  estimatePosn.x = poseGlob.x;               
  estimatePosn.y = poseGlob.y;
  }
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

    angLimit(pose.theta);
 
    //Update the global Pose of the robot with respect to the origin measured by odometry
    poseGlob.x = pose.x*cos(pose.theta); 
    poseGlob.y = pose.y*sin(pose.theta);

    flag = true;    //To update the odometry with the last Weighted Average result

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
    double totalWeight = 0.0;   //Sum of the weights

    //Pose of the robot in global frame based on Average Weight result
    weightedPose.x = 0.0;       
    weightedPose.y = 0.0;

    // Loop through each observed marker to obtain the Weighted Average
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
       
        double weight = computeWeight(observation);   //Calculate the weight of each observation
        totalWeight += weight;     // Accumulate the weightes

        ////Calculate the weightd pose of each observation
        weightedPose.x += weight * markerPoseGlob.x;  
        weightedPose.y += weight * markerPoseGlob.y;

        flag = false;   //To apply the Weighted Average to the estimation
    }

    if (totalWeight > 0.0) {
        
        // Update the estimation of the robot's pose based on weighted average
        weightedPose.x /= totalWeight;
        weightedPose.y /= totalWeight;

        // Update the robot's pose based on weighted average for the odometry
        poseGlob.x = weightedPose.x; 
        poseGlob.y = weightedPose.y;
        pose.x = poseGlob.x/(cos(pose.theta)+0.00001);
        pose.y = poseGlob.y/(sin(pose.theta)+0.00001);
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
    angLimit(robotParams.angle_fov);   //limit the angle within [-pi, pi]
    
    robotParams.odom_noise_rotation_from_rotation =
    robotParams.odom_noise_rotation_from_rotation * M_PI / 180;
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

}

/**
 * mydisplay()
 * This function is called whenever the display is updated. The controller
 * will draw the estimated robot position after this function returns.
 */
void mydisplay()
{
    // Write the sensor data as well as the Weighted Average result in a text file for plots
    std::ofstream outputFile("Figures.txt", std::ios::app);
    if (outputFile.is_open()) {
        // Write header line if the file is empty
        if (outputFile.tellp() == 0) {
            outputFile << "poseodom.x poseodom.y markerPose1.x markerPose1.y markerPose2.x markerPose2.y markerPose3.x markerPose3.y markerPose4.x markerPose4.y PoseAver.x PoseAver.y" << std::endl;
        }
        
        // Append varibles values
        outputFile << poseGlob.x << " " << poseGlob.y <<
                " " << markerPoseGlobFig[1].x << " " << markerPoseGlobFig[1].y <<
                " " << markerPoseGlobFig[2].x << " " << markerPoseGlobFig[2].y <<
                " " << markerPoseGlobFig[3].x << " " << markerPoseGlobFig[3].y <<
                " " << markerPoseGlobFig[4].x << " " << markerPoseGlobFig[4].y <<
                " " << weightedPose.x << " " << weightedPose.y << std::endl;
        
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

