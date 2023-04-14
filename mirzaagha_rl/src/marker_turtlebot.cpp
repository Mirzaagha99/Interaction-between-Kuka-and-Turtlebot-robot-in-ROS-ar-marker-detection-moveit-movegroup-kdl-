#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/calib3d.hpp>

#include <actionlib/server/simple_action_server.h>
#include "mirzaagha_rl/markerTurtlebotAction.h"

class TurtlebotResearcher{

    private:
    cv::Ptr<cv::aruco::Dictionary> _dictionary;
    cv::Mat _img;
    ros::NodeHandle _nh;
    ros::Subscriber _cmrRdrDtr;
    actionlib::SimpleActionServer<mirzaagha_rl::markerTurtlebotAction> _as;
    mirzaagha_rl::markerTurtlebotFeedback _feedback;
    mirzaagha_rl::markerTurtlebotResult _result;


    public:
    TurtlebotResearcher()
    :_as(_nh, "marker_turtlebot", boost::bind(&TurtlebotResearcher::serverCb, this, _1), false)
    {
        //Returns one of the predefined dictionaries defined in PREDEFINED_DICTIONARY_NAME.
        _dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_1000);

        _cmrRdrDtr = _nh.subscribe("/turtlebot3_burger/camera/image_raw", 1, &TurtlebotResearcher::cameraReaderDetector, this);     
        //I initialize the values of the markerTurtlebot.action
        _feedback.ID=0;
	    _feedback.marker = 0;
        _result.ID=0;
        _as.start();
    }


    void cameraReaderDetector(const sensor_msgs::ImageConstPtr &ros_img)
    {       
            cv_bridge::CvImagePtr cv_ptr;
            // Convert topic to opencv data
            try{
                cv_ptr = cv_bridge::toCvCopy(ros_img, sensor_msgs::image_encodings::RGB8);
                _img = cv_ptr->image;
            }
            //in case of exception
            catch(cv_bridge::Exception &e){
                ROS_ERROR("marker_turtolbot detector:\n<<A cv_bridge exception occurred: %s>>", e.what());
                return;
            }
            std::vector<int> ids;
            std::vector< std::vector<cv::Point2f> > corners;
            
            cv::aruco::detectMarkers(_img, _dictionary, corners, ids);
            
            if(ids.size()!=0) 
            {
                _feedback.ID=ids[0];
                _feedback.marker=1;
            }
    }

    void serverCb(const mirzaagha_rl::markerTurtlebotGoalConstPtr &goal)
    {      
        if(!_as.isActive()) return;
        ros::Rate r(20); 
        bool success = false;      
        while(!success)
        { 
            if(!ros::ok())
            {
                _as.setAborted();
                return;
            }
            
            if(_as.isPreemptRequested()) 
            {      
                _as.setPreempted();
                return;
            }
           // it sends the feedback
           _as.publishFeedback(_feedback);
            //if the marker was found and ID equals the goal
            if(_feedback.marker!= 0 && (_feedback.ID== goal->find_marker) )
            {   
                success=true;
                _result.ID = _feedback.ID; 
                ROS_INFO("marker_turtlebot detector:\n<<Detection of marker %d happened successfully!>>", goal->find_marker);     
                //it sends the ID     
                _as.setSucceeded(_result); 
            }else 
            {
                ROS_INFO("marker_turtolbot detector:<<Marker streamed by the cam is %d, instead the desired marker is %d>>",_feedback.ID, goal->find_marker); 
            }
            // rate.sleep() is a thread sleep with a duration defined by a frequency.
            r.sleep();
        }
       
    }


};

int main(int argc, char **argv){
    ros::init(argc, argv, "researcher_turtlebot");
    ROS_INFO("Marker turtlebot detector!");
    TurtlebotResearcher detector;
    //ros::spin() blocks until ROS invokes a shutdown
    ros::spin();
    return 0;
}
