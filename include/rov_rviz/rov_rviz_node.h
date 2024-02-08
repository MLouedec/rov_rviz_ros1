// Purpose: Header file for the rov_rviz_node.cpp file
// This node can be used to visualize the ROVs in rviz from the rosbag file

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <mavros_msgs/OverrideRCIn.h>
#include <usbl/Usbl.h>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/WrenchStamped.h>
#include <Eigen/Dense>
#include <sstream>
#include <dynamic_reconfigure/server.h>
#include <rov_rviz/ViewerConfig.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Vector3Stamped.h>
#include "dynamical_model.hpp"

float sawtooth(float x){
    return 2*atan(tan(x/2));
}

void pose_yaw_offset(geometry_msgs::Pose& pose, float yaw_offset);

void set_color(visualization_msgs::Marker& marker,  const std::string& rov_frame);

class RovRviz
{
public:
    RovRviz(ros::NodeHandle &n){
        // node rate
        rate = 10; // Hz

        // get the name of the ROVs
        if(n.getParam("/rovA_name", rovA_name)){
            ROS_INFO("We are in the presence of: %s", rovA_name.c_str());
        }else{
            ROS_ERROR("Failed to get param 'rovA_name'");
        }
        if(n.getParam("/rovB_name", rovB_name)){
            ROS_INFO("We are in the presence of: %s", rovB_name.c_str());
        }else{
            ROS_ERROR("Failed to get param 'rovB_name'");
        }

        // get the cap of the usbl (the reference cap)
        if(n.getParam("/cap_usbl_deg", cap_usbl)) {
            ROS_INFO("Param cap usbl is: %f (deg)", cap_usbl);
            cap_usbl = cap_usbl*M_PI/180;
        }else{
            ROS_ERROR("Failed to get param 'cap_usbl'");
        }

        // Init kalman filters
        init_matrices();
        Eigen::VectorXd X0 = Eigen::VectorXd::Zero(4); // initial estimation
        EKF_rovA.init(X0, P0_, Qp_, Qm_, 1/rate);
        EKF_rovB.init(X0, P0_, Qp_, Qm_, 1/rate);

        // ------------------------------
        // subscribers and publishers
        // ------------------------------

        // usbl to get xy position
        ubsl = n.subscribe("/usbl", 1000, &RovRviz::ubslCallback,this);

        // mavros to get z and attitude
        sub_mavros_rovA = n.subscribe("/"+rovA_name+"/mavros/global_position/local", 1000, &RovRviz::mavros_rovA_Callback,this);
        sub_mavros_rovB = n.subscribe("/"+rovB_name+"/mavros/global_position/local", 1000, &RovRviz::mavros_rovB_Callback,this);

        // get the position of the target
        sub_target_rovA = n.subscribe("/"+rovA_name+"/coord_objectif", 1000, &RovRviz::target_rovA_Callback,this);
        sub_target_rovB = n.subscribe("/"+rovB_name+"/coord_objectif", 1000, &RovRviz::target_rovB_Callback,this);

        // get the motors commands
        sub_RC_rovA = n.subscribe("/"+rovA_name+"/mavros/rc/override", 1000, &RovRviz::RC_rovA_Callback,this);
        sub_RC_rovB = n.subscribe("/"+rovB_name+"/mavros/rc/override", 1000, &RovRviz::RC_rovB_Callback,this);

        // give the position of the last usbl measurement
        pub_marker_rovA_measurement = n.advertise<visualization_msgs::Marker>("/"+rovA_name+"/marker", 1000);
        pub_marker_rovB_measurement = n.advertise<visualization_msgs::Marker>("/"+rovB_name+"/marker", 1000);

        // give the estimated position of the ROVs (use the dynamical model of the ROV)
        pub_marker_rovA_estimation = n.advertise<visualization_msgs::Marker>("/"+rovA_name+"/estimation_marker", 1000);
        pub_marker_rovB_estimation = n.advertise<visualization_msgs::Marker>("/"+rovB_name+"/estimation_marker", 1000);

        // covariance ellipsoid from the kalman filter
        pub_marker_elliA = n.advertise<visualization_msgs::Marker>("/"+rovA_name+"/estimation_elli", 1000);
        pub_marker_elliB = n.advertise<visualization_msgs::Marker>("/"+rovB_name+"/estimation_elli", 1000);

        // give the position of the target of the ROVs
        pub_marker_rovA_target = n.advertise<visualization_msgs::Marker>("/"+rovA_name+"/target", 1000);
        pub_marker_rovB_target = n.advertise<visualization_msgs::Marker>("/"+rovB_name+"/target", 1000);

        // draw the triangle (with the targets of the ROVs)
        pub_triangle = n.advertise<visualization_msgs::Marker>("/triangle", 1000);

        // give the control input of the ROVs
        pub_control_input_rovA = n.advertise<geometry_msgs::TwistStamped>("/"+rovA_name+"/twist", 1000);
        pub_control_input_rovB = n.advertise<geometry_msgs::TwistStamped>("/"+rovB_name+"/twist", 1000);

        // TODO deprecated ???
//        pub_measured_position_rovA = n.advertise<geometry_msgs::PoseStamped>("/"+rovA_name+"/measured_position", 1000);
//        pub_measured_position_rovB = n.advertise<geometry_msgs::PoseStamped>("/"+rovB_name+"/measured_position", 1000);

//        pub_z_vector = n.advertise<geometry_msgs::Vector3Stamped>("/z_vector", 1000);
        ROS_INFO("rov_rviz_node initialized");
    }

    void kalman_predict(); // prediction for all the ROVs
    // The kalman corrections are called in the usbl callback

    void broadcast_tf_pose_i(const geometry_msgs::PoseStamped& msg, const std::string& rov_frame);
    void publish_marker_i(const std::string& rov_frame, const std::string& frame_id, float alpha, ros::Publisher& pub);
    void publish_control_input_i(const std::string& rov_frame);
    void publish_cov_ellipsoid(const std::string& rov_frame, const std::string& frame_id,
                          ros::Publisher& pub, const Eigen::MatrixXd& P,
                               const geometry_msgs::PoseStamped& pose_estimated);
//    void pose_estimation_prediction_i(geometry_msgs::PoseStamped& pose_estimated, geometry_msgs::TwistStamped& twist_estimated, geometry_msgs::WrenchStamped& wrench);

    void publish_robots_estimation();
    void publish_target();
    void publish_robots_measurement();
    void publish_control_input();
    void publish_triangle();
    void publish_z_vector();
    void publish_measured_position();

//    void pose_estimation_prediction();

    double rate; // rate of the node

    geometry_msgs::WrenchStamped control_input_rovA; // wrench of the ROVs
    geometry_msgs::WrenchStamped control_input_rovB;

    geometry_msgs::PoseStamped pose_measured_rovA; // measured pose of the ROV ROVs (only the x and y are measured)
    geometry_msgs::PoseStamped pose_measured_rovB; // the rest ( z yaw pitch roll) come from the measurement

    double mission_time; // ROS time during the mission (rosbag time is not the same), given by mavros

private:

    float cap_usbl; // heading of the usbl

    std::string rovA_name; // name of the ROVs used to name topics
    std::string rovB_name;

    ros::Publisher pub_marker_rovA_measurement; // publisher for the marker of the ROVs
    ros::Publisher pub_marker_rovB_measurement; 

    ros::Publisher pub_marker_rovA_estimation; // publisher for the estimation of the ROVs
    ros::Publisher pub_marker_rovB_estimation;

    ros::Publisher pub_marker_rovA_target; // publisher for the target of the ROVs 
    ros::Publisher pub_marker_rovB_target; 

    ros::Publisher pub_control_input_rovA; // publisher for the twist of the ROVs 
    ros::Publisher pub_control_input_rovB;

    ros::Publisher pub_triangle; // publisher for the triangle (with the targets of the ROVs)

    ros::Publisher pub_marker_elliA; // publisher for the covariance ellipsoid
    ros::Publisher pub_marker_elliB;

//    ros::Publisher pub_measured_position_rovA; // to draw the curves - separate the date of the usbl
//    ros::Publisher pub_measured_position_rovB;

//    ros::Publisher pub_z_vector; // to draw the curves to study the stability of the system

    ros::Subscriber ubsl; // (x,y) positions are given by the usbl

    ros::Subscriber sub_mavros_rovA; // depth and orientation of the ROVs
    ros::Subscriber sub_mavros_rovB; 

    ros::Subscriber sub_target_rovA; // pose of the targets of the ROVs
    ros::Subscriber sub_target_rovB;

    ros::Subscriber sub_RC_rovA; // RC commands of the ROVs
    ros::Subscriber sub_RC_rovB;

    geometry_msgs::PoseStamped pose_target_rovA; // pose of ROVs' target
    geometry_msgs::PoseStamped pose_target_rovB;

//    geometry_msgs::PoseStamped pose_estimated_rovA; // pose of the ROVs estimated by the observer
//    geometry_msgs::PoseStamped pose_estimated_rovB;

    geometry_msgs::TwistStamped twist_estimated_rovA; // twist of the ROVs estimated by the observer
    geometry_msgs::TwistStamped twist_estimated_rovB;

    tf::TransformBroadcaster br;

    // kalman filters ( contained the estimated position and the associated covariance matrices)
    ROV_ExtendedKalmanFilter EKF_rovA;
    ROV_ExtendedKalmanFilter EKF_rovB;

//    dynamic_reconfigure::Server<rov_rviz_ros1::ViewerConfig> cfg_server;
//    dynamic_reconfigure::Server<rov_rviz_ros1::ViewerConfig>::CallbackType cfg_f;

    void target_Callback(const geometry_msgs::PoseStamped& msg, const std::string& name);
    void mavros_Callback(const nav_msgs::Odometry& msg, const std::string& name);
    void ubslCallback(const usbl::Usbl& msg);

    void target_rovA_Callback(const geometry_msgs::PoseStamped& msg)
    {
        target_Callback(msg, rovA_name);
    }

    void target_rovB_Callback(const geometry_msgs::PoseStamped& msg)
    {
        target_Callback(msg, rovB_name);
    }

    void mavros_rovA_Callback(const nav_msgs::Odometry& msg)
    {
        mavros_Callback(msg, rovA_name);
    }

    void mavros_rovB_Callback(const nav_msgs::Odometry& msg)
    {
        mavros_Callback(msg, rovB_name);
    }

    void RC_rovA_Callback(const mavros_msgs::OverrideRCIn& msg)
    {
        RC_Callback(msg,control_input_rovA);
    }

    void RC_rovB_Callback(const mavros_msgs::OverrideRCIn& msg)
    {
        RC_Callback(msg,control_input_rovB);
    }

    void RC_Callback(const mavros_msgs::OverrideRCIn& msg, geometry_msgs::WrenchStamped& twist);

};