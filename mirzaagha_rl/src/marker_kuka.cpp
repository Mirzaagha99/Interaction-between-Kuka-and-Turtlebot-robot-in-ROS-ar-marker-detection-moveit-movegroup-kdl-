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
#include "mirzaagha_rl/markerKukaAction.h"

class KukaResearcher{

    private:
    cv::Ptr<cv::aruco::Dictionary> _dictionary;
    cv::Mat _cameraMatrix;
    cv::Mat _distCoeffs;
    cv::Mat _img;
    bool _success;
    int _index;
    ros::NodeHandle _nh;
    ros::Subscriber _cameraSub;
    ros::Subscriber _infoSub;
    actionlib::SimpleActionServer<mirzaagha_rl::markerKukaAction> _as;
    std::vector< std::vector<cv::Point2f> > _corners;
    mirzaagha_rl::markerKukaFeedback _feedback;
    mirzaagha_rl::markerKukaResult _result;


    public:
    KukaResearcher()
    :_as(_nh, "marker_kuka", boost::bind(&KukaResearcher::serverCb, this, _1), false),
    _cameraMatrix(3, 3, CV_64FC1), 
    _distCoeffs(1, 5, CV_64FC1), 
    _success(false),
    _index(0)
    {
        //Returns one of the predefined dictionaries defined in PREDEFINED_DICTIONARY_NAME.
        _dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_1000);

        _cameraSub = _nh.subscribe("/camera2/image_raw", 1, &KukaResearcher::imageCb, this);
        _infoSub = _nh.subscribe("/camera2/camera_info", 1, &KukaResearcher::infoCb, this);
        _feedback.found_marker.clear();
        _as.start();
    }

    void infoCb(const sensor_msgs::CameraInfoConstPtr &info_msg)
    {
        auto K_step= info_msg->K.begin();
        auto D_step= info_msg->D.begin();

        for(auto step= _cameraMatrix.begin<double>(); step!= _cameraMatrix.end<double>(); ++step)
        {
            *step= *K_step;
            ++K_step;
        }

        for(auto step= _distCoeffs.begin<double>(); step!= _distCoeffs.end<double>(); ++step)
        {
            *step= *D_step;
            ++D_step;
        }
    }

    void imageCb(const sensor_msgs::ImageConstPtr &ros_img)
    {
        if(!_success)
        {
            cv_bridge::CvImagePtr cv_ptr;
            try{
                cv_ptr = cv_bridge::toCvCopy(ros_img, sensor_msgs::image_encodings::RGB8);
                _img = cv_ptr->image;
            }
            catch(cv_bridge::Exception &e){
                ROS_ERROR("marker_kuka detector:\n<<There is an exception with cv: %s>>", e.what());
                return;
            }   
            std::vector<int> ids;
            cv::aruco::detectMarkers(_img, _dictionary, _corners, ids);
            if(ids.size()!=0) 
            {
                _index = 0;
                _feedback.ID=ids[_index];
                _feedback.marker=1;
            }
        }
    }
    void serverCb(const mirzaagha_rl::markerKukaGoalConstPtr &goal)
    {   if(!_as.isActive()) return;
        ros::Rate r(20);
        _success = false;
        ROS_INFO("marker_kuka detector:\n<<ID requested is: %d>>", goal->find_marker);
        while(!_success)
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

            _as.publishFeedback(_feedback);
            //if the marker was found and ID equals the goal
            if(_feedback.marker!= 0 && (_feedback.ID== goal->find_marker) )
            {
                _success = true;
                std::vector<cv::Vec3d> rot, trasl;		  
                double markerDim = 0.54;
                cv::aruco::estimatePoseSingleMarkers(_corners, markerDim, _cameraMatrix, _distCoeffs, rot, trasl);
                cv::Vec3d find_marker_trasl = trasl[_index]; 

                for(int i = 0; i < 3; i++)
                    _feedback.found_marker.push_back(find_marker_trasl[i]);

                _result.found_marker = _feedback.found_marker;
                ROS_INFO("marker_kuka detector:\n<<Detection of marker %d happened successfully!>>", goal->find_marker);  
                _as.setSucceeded(_result);   
                    
            }else ROS_INFO("marker_turtolbot detector:<<Marker streamed by the cam is %d, instead the desired marker is  %d>>",_feedback.ID, goal->find_marker); 

            r.sleep();
        }
    }

};

int main(int argc, char **argv){
    ros::init(argc, argv, "marker_kuka_");
    ROS_INFO("Marker kuka detector!");
    KukaResearcher detector;
    ros::spin();
    return 0;
}
