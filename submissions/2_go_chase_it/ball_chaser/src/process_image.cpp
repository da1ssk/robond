#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>

// Define a global client that can request services
ros::ServiceClient client;

// This function calls the command_robot service to drive the robot in the specified direction
void drive_robot(float lin_x, float ang_z)
{
    ROS_INFO_STREAM("Driving the robot");
    
    // TODO: Request a service and pass the velocities to it to drive the robot
    ball_chaser::DriveToTarget srv;
    srv.request.linear_x = lin_x;
    srv.request.angular_z = ang_z;
    
    if (!client.call(srv))
        ROS_ERROR("Failed to call service command_robot");
      
}

// This callback function continuously executes and reads the image data
void process_image_callback(const sensor_msgs::Image img)
{
    int white_pixel = 255;

    // TODO: Loop through each pixel in the image and check if there's a bright white one
    // Then, identify if this pixel falls in the left, mid, or right side of the image
    // Depending on the white ball position, call the drive_bot function and pass velocities to it
    // Request a stop when there's no white ball seen by the camera
    
    static int last_seen = 0; // 0:left, 1:right
    float lin_x = 0.5;
    float ang_z = 0;
    const float new_ang_z = 1.5707 / 2.0; // 90 degrees
    
    for (int i = 0; i < img.height * img.step; i++) {
        if (img.data[i] == white_pixel) {
            // where is the ball: left/mid/right?
            int x = i % img.step;
            int oneThird = img.step / 3;
            
            if (x < oneThird) { // left
                ang_z = new_ang_z;
                last_seen = 0;
                printf("left");
            } else if (x < oneThird * 2) { // middle
                printf("middle");
            } else { // right
                ang_z = -new_ang_z;
                last_seen = 1;
                printf("right");
            }
            
            // call drive_bot
            drive_robot(lin_x, ang_z);
            
            // Wait for robot to settle
//            ros::Duration(0.25).sleep();
            return;
        }
    }
    
    // request a stop if no white pixel is found
//    drive_robot(0, 0);

    // keep searching the ball by rotating itself
    if (last_seen == 0) {
        lin_x = 0;
        ang_z = new_ang_z;
    } else {
        lin_x = 0;
        ang_z = -new_ang_z;
    }
    drive_robot(lin_x, ang_z);
}

int main(int argc, char** argv)
{
    // Initialize the process_image node and create a handle to it
    ros::init(argc, argv, "process_image");
    ros::NodeHandle n;

    // Define a client service capable of requesting services from command_robot
    client = n.serviceClient<ball_chaser::DriveToTarget>("/ball_chaser/command_robot");

    // Subscribe to /camera/rgb/image_raw topic to read the image data inside the process_image_callback function
    ros::Subscriber sub1 = n.subscribe("/camera/rgb/image_raw", 10, process_image_callback);

    // Handle ROS communication events
    ros::spin();

    return 0;
}
