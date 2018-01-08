#include <iostream>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv/cv.h>
#include <stdio.h>

#include <signal.h>         // Handling ctrl+C signal to close everything properly

// Manages message with oroclient
#include "std_msgs/String.h"
#include <sstream>


enum enCommand
{
    CMD_ADD_INST =1,
    CMD_ADD_PROP,
    CMD_FIND,
    CMD_REMOVE,
    LAST_CMD
};

static const uint32_t MY_ROS_QUEUE_SIZE = 1000;


/*------ Declaration des variables globales ------*/

static int gPrevNb;  
static CvHaarClassifierCascade *cascade;
static CvMemStorage *storage;
ros::Publisher gOroChatter_pub;


/*---------- Declaration des fonctions -----------*/

void detectFaces(IplImage *img);

/*------------------------------------------------*/


// Create a function which manages a clean "CTRL+C" command -> sigint command
void sigint_handler(int dummy)
{
    ROS_INFO("- detect-human-face is shutting down...\n");
    
    // Liberation de l'espace memoire
    cvDestroyWindow("Window-FT");
    cv::destroyWindow("Initial");
    cvReleaseHaarClassifierCascade(&cascade);
    cvReleaseMemStorage(&storage);
    
    ROS_INFO("\n\n... Bye bye !\n   -Manu\n");
    exit(EXIT_SUCCESS); // Shut down the program
}

// Explain who am i
void sayMyName()
{
    ros::Rate loop_rate(10); // Communicate slow rate

    std_msgs::String msg;
    std::stringstream ss;
    char enumCmd = 0;
    
    //ss << "add\n[kinect1 rdf:type VideoSensor, kinect1 rdfs:label \"Big brother\", kinect1 isIn Bedroom]\n#end#\n";
    enumCmd = (char)CMD_ADD_INST;
    ss << "BigBrother#"<<enumCmd<<"#kinect#VideoSensor";
    msg.data = ss.str();

    ROS_INFO("%s", msg.data.c_str());

    gOroChatter_pub.publish(msg);

    ros::spinOnce();
	  loop_rate.sleep();
	
	  ss.str("");
	  enumCmd = (char)CMD_ADD_PROP;
    ss << "BigBrother#"<< enumCmd <<"#kinect#isIn#Bedroom";
    msg.data = ss.str();

    ROS_INFO("%s", msg.data.c_str());

    gOroChatter_pub.publish(msg);

    ros::spinOnce();
	  loop_rate.sleep();
}

// Callback 
void imgcb(const sensor_msgs::Image::ConstPtr& msg)
{
    try
    {
        // Convert ROS images to cv::mat
        cv_bridge::CvImageConstPtr cv_ptr;
        cv_ptr = cv_bridge::toCvShare(msg);
        IplImage *img = new IplImage(cv_ptr->image); 

        cv::imshow("Initial", cv_ptr->image);
        cv::waitKey(1);  // Update screen
        
        /*********************************/
                
        // Creation d'une fenetre
        cvNamedWindow("Window-FT", 1);
        
        // Boucle de traitement 
        detectFaces(img);
        
        /*********************************/
        
        
    } catch (const cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }
}

int main(int argc, char* argv[])
{
    ROS_INFO("\n\n\t-- Launching Face detector --\n\n");
    
    gPrevNb = 0;
    
    // Chargement du classifieur
    const char *filename = "/home/dumont/catkin_ws/src/ros-openni-example-master/src/haarcascade_frontalface_alt.xml"; 
    cascade = (CvHaarClassifierCascade*)cvLoadHaarClassifierCascade( filename, cvSize(24, 24) );
    
    // Initialisation de l'espace memoire
    storage = cvCreateMemStorage(0);

    // ROS init
    ros::init(argc, argv, "detect_face_rgb");

    ros::NodeHandle nk; // Communicate with the kinect
    ros::NodeHandle nOroCl;  // Communicate wit oroclient
    
    // Override the signal interrupt from ROS
    signal(SIGINT, sigint_handler);
    
    ros::Subscriber sub = nk.subscribe("camera/rgb/image_color", MY_ROS_QUEUE_SIZE, imgcb);
    gOroChatter_pub = nOroCl.advertise<std_msgs::String>("oroChatter", 10);
   
	  usleep(500000); // necessary to be able to send data

    // Identify myself
    sayMyName();

    cv::namedWindow("Initial");
    
    ros::spin();
    
    
    cv::destroyWindow("Initial");

    ROS_INFO("\n\n... Bye bye !\n   -Manu\n");

    return EXIT_SUCCESS;
}

/*------------------------------------------------*/  
void detectFaces(IplImage *img)  
{  
    /*int i, nbFaces = 0;
    std_msgs::String msg;
    std::stringstream ss;
    
    // Face detection
    CvSeq *faces = cvHaarDetectObjects(img, cascade, storage, 1.2, 3, 0, cvSize(80,80));
    
    nbFaces = (faces?faces->total:0);
    
    // If there is more faces than previously
    if(nbFaces > gPrevNb)
    {
        std_msgs::String msg;
        std::stringstream ss;
        
        // Add new human in the ontologie
        for(i=gPrevNb; i < nbFaces; i++)
        {
            ss << "add\n[human"<< i <<" rdf:type Human, human rdfs:label \"human"<< i <<"\", kinect1 canSee human"<< i <<"]\n#end#\n";
            msg.data = ss.str();

            ROS_INFO("%s", msg.data.c_str());

            gOroChatter_pub.publish(msg);

            ros::spinOnce();
        }
        
    }
    // If there is less faces than previously
    else if (nbFaces < gPrevNb)
    {
        std_msgs::String msg;
        std::stringstream ss;
        
        // Removes the last ones
        for(i=nbFaces; i < gPrevNb; i++)
        {
            ss << "remove\n[human"<< i <<" rdf:type Human, kinect1 canSee human"<< i <<"]\n#end#\n";
            msg.data = ss.str();

            ROS_INFO("%s", msg.data.c_str());

            gOroChatter_pub.publish(msg);

            ros::spinOnce();
        }
    }
        
    // Draw rectangles over faces
    for(i=0; i<(faces?faces->total:0); i++)  
    {  
        CvRect *r = (CvRect*)cvGetSeqElem(faces, i);  
        cvRectangle(img, cvPoint(r->x, r->y), cvPoint(r->x + r->width, r->y + r->height), CV_RGB(255, 0, 0), 1, 8, 0);
    }

    // Update number of faces
    gPrevNb = (faces?faces->total:0);
    
    cvShowImage("Window-FT", img);  */
} 

