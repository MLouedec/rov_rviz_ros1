#include <rov_rviz/rov_rviz_node.h>

// ------------------------------
// tool function
// ------------------------------

void RovRviz::kalman_predict(){
    // call a prediction for all the Kalman filters
    double yawA = tf::getYaw(pose_measured_rovA.pose.orientation);
    double yawB = tf::getYaw(pose_measured_rovB.pose.orientation);

    // set as 0 if nan
    if(std::isnan(yawA)){
        yawA = 0;
    }
    if(std::isnan(yawB)){
        yawB = 0;
    }

    Eigen::VectorXd tauA = Eigen::VectorXd::Zero(2);
    tauA(0) = control_input_rovA.wrench.force.x;
    tauA(1) = control_input_rovA.wrench.force.y;

    Eigen::VectorXd tauB = Eigen::VectorXd::Zero(2);
    tauB(0) = control_input_rovB.wrench.force.x;
    tauB(1) = control_input_rovB.wrench.force.y;

    EKF_rovA.predict(tauA,yawA);
    EKF_rovB.predict(tauB,yawB);
}

void pose_yaw_offset(geometry_msgs::Pose& pose, float yaw_offset){
    // add the offset to the yaw of the pose
    tf::Quaternion q(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    m.setRPY(roll, pitch, yaw+yaw_offset);
    m.getRotation(q);
    pose.orientation.x = q.x();
    pose.orientation.y = q.y();
    pose.orientation.z = q.z();
    pose.orientation.w = q.w();
}

void set_color(visualization_msgs::Marker& marker,  const std::string& rov_frame){
    // set the color of the marker depending on the ROV name
    if(rov_frame.find("clyde")!=std::string::npos){
        marker.color.r = 1.0;
        marker.color.g = 0.5;
        marker.color.b = 0.0;
    }else if(rov_frame.find("blinky")!=std::string::npos) {
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
    }else if(rov_frame.find("inky")!=std::string::npos){
        marker.color.r = 0.0;
        marker.color.g = 0.3;
        marker.color.b = 1.0;
    }else{
        ROS_ERROR("Invalid name for ROV color: %s", rov_frame.c_str());
    }

    if(rov_frame.find("estimation")!=std::string::npos){
        marker.color.r += 0.2;
        marker.color.g += 0.2;
        marker.color.b += 0.2;
    }
}

// ------------------------------
// subscriber callback
// ------------------------------

void RovRviz::ubslCallback(const usbl::Usbl& msg)
{
    // Parse the message to the (x,y) position of the ROV (in the usbl frame)
    std_msgs::Header header = msg.header;
    float id = msg.beacon_ID;

    float range = msg.range;
    float azimuth = msg.azimuth;
    float elevation = msg.elevation;
    float x_local = range*cos(azimuth*M_PI/180)*cos(elevation*M_PI/180); // is is really different from the msg.position
    float y_local = range*sin(azimuth*M_PI/180)*cos(elevation*M_PI/180);

    float x_global = x_local*cos(cap_usbl) - y_local*sin(cap_usbl);
    float y_global = x_local*sin(cap_usbl) + y_local*cos(cap_usbl);
    Eigen::VectorXd Y(2);
    Y << x_global, y_global;

    if(id == 2){ // it is rovA
        pose_measured_rovA.header = header;
        pose_measured_rovA.pose.position.x = x_global;
        pose_measured_rovA.pose.position.y = y_global;

        // update the EKF
        EKF_rovA.correct(Y);
//        pose_estimated_rovA.header = header;
//        pose_estimated_rovA.pose.position.x = x_global;
//        pose_estimated_rovA.pose.position.y = y_global;
    } else if(id == 1){ // it is rovB
        pose_measured_rovB.header = header;
        pose_measured_rovB.pose.position.x = x_global;
        pose_measured_rovB.pose.position.y = y_global;

        // update the EKF
        EKF_rovB.correct(Y);
//
//        // update the estimation as well
//        pose_estimated_rovB.header = header;
//        pose_estimated_rovB.pose.position.x = x_global;
//        pose_estimated_rovB.pose.position.y = y_global;
    } else {
        ROS_ERROR("Invalid id for ROV id: %f", id);
    }
}

void RovRviz::mavros_Callback(const nav_msgs::Odometry& msg, const std::string& name)
{
    // Parse the message to the depth and orientation of the ROV
    std_msgs::Header header;
    geometry_msgs::Pose pose;
    header = msg.header;
    pose = msg.pose.pose;
    mission_time = msg.header.stamp.toSec();

    if(name == rovA_name){
        pose_measured_rovA.header = header;
        pose_measured_rovA.pose.orientation = pose.orientation;
        pose_yaw_offset(pose_measured_rovA.pose,-M_PI/2); // TODO why is there a 90 deg offset on the heading data ?
        pose_measured_rovA.pose.position.z = pose.position.z;

//        // same for the estimation
//        pose_estimated_rovA.header = header;
//        pose_estimated_rovA.pose.orientation = pose.orientation;
//        pose_yaw_offset(pose_estimated_rovA.pose,-M_PI/2); // TODO why is there a 90 deg offset on the heading data ?
//        pose_estimated_rovA.pose.position.z = pose.position.z;

    }else if(name == rovB_name){
        pose_measured_rovB.header = header;
        pose_measured_rovB.pose.orientation = pose.orientation;
        pose_measured_rovB.pose.position.z = pose.position.z;
        pose_yaw_offset(pose_measured_rovB.pose,-M_PI/2);  // TODO why is there a 90 deg offset on the heading data ?

//        // same for the estimation
//        pose_estimated_rovB.header = header;
//        pose_estimated_rovB.pose.orientation = pose.orientation;
//        pose_estimated_rovB.pose.position.z = pose.position.z;
//        pose_yaw_offset(pose_estimated_rovB.pose,-M_PI/2);  // TODO why is there a 90 deg offset on the heading data ?

    }else {
        ROS_ERROR("Invalid name for ROV: %s", name.c_str());
    }
}

void  RovRviz::target_Callback(const geometry_msgs::PoseStamped& msg, const std::string& name){
    float x_local = msg.pose.position.x;
    float y_local = msg.pose.position.y;
    float z = -msg.pose.position.z;
    float x_global = x_local;
    float y_global = y_local;

    if (name == rovA_name){
        pose_target_rovA = msg;
        pose_target_rovA.pose.position.x = x_global;
        pose_target_rovA.pose.position.y = y_global;
        pose_target_rovA.pose.position.z = z; // inverse the z axis
        pose_target_rovA.pose.orientation = msg.pose.orientation;
    }else if(name == rovB_name){
        pose_target_rovB = msg;
        pose_target_rovB.pose.position.x = x_global;
        pose_target_rovB.pose.position.y = y_global;
        pose_target_rovB.pose.position.z = z; // inverse the z axis
        pose_target_rovB.pose.orientation = msg.pose.orientation;
    }else {
        ROS_ERROR("Invalid name for ROV target: %s", name.c_str());
    }
}

void RovRviz::RC_Callback(const mavros_msgs::OverrideRCIn& msg, geometry_msgs::WrenchStamped& control_input){
    // Parse the message to the linear control_input of the ROV
    // a pwm of 500 is about 40N of thrust
    float k1= 40./500.;
    control_input.wrench.force.x = k1*(float)(msg.channels[4]-1500);
    control_input.wrench.force.y = -k1*(float)(msg.channels[5]-1500); // this channel is inverted
    control_input.wrench.force.z = k1*(float)(msg.channels[2]-1500);
    control_input.wrench.torque.z = k1*(float)(msg.channels[3]-1500);
    control_input.wrench.torque.y = k1*(float)(msg.channels[0]-1500);
    control_input.wrench.torque.x = k1*(float)(msg.channels[1]-1500);
}


// ------------------------------
// publisher related function
// ------------------------------

void RovRviz::publish_robots_measurement(){
    broadcast_tf_pose_i(pose_measured_rovA, rovA_name);
    broadcast_tf_pose_i(pose_measured_rovB, rovB_name);
    publish_marker_i(rovA_name, rovA_name, 1, pub_marker_rovA_measurement);
    publish_marker_i(rovB_name, rovB_name, 1, pub_marker_rovB_measurement);
}

void RovRviz::publish_robots_estimation(){
    geometry_msgs::PoseStamped pose_estimated_rovA;
    geometry_msgs::PoseStamped pose_estimated_rovB;

    pose_estimated_rovA = pose_measured_rovA;
    pose_estimated_rovB = pose_measured_rovB;

    pose_estimated_rovA.pose.position.x = EKF_rovA.X(0);
    pose_estimated_rovA.pose.position.y = EKF_rovA.X(1);
    pose_estimated_rovB.pose.position.x = EKF_rovB.X(0);
    pose_estimated_rovB.pose.position.y = EKF_rovB.X(1);

    broadcast_tf_pose_i(pose_estimated_rovA, "estimation_"+rovA_name);
    broadcast_tf_pose_i(pose_estimated_rovB, "estimation_"+rovB_name);
    publish_marker_i(rovA_name, "estimation_"+rovA_name, 0.5, pub_marker_rovA_estimation);
    publish_marker_i(rovB_name, "estimation_"+rovB_name, 0.5, pub_marker_rovB_estimation);

    // publish the covariance ellipsoid of the EKF
    publish_cov_ellipsoid(rovA_name, "map", pub_marker_elliA, EKF_rovA.P, pose_estimated_rovA);
    publish_cov_ellipsoid(rovB_name, "map", pub_marker_elliB, EKF_rovB.P, pose_estimated_rovB);
}

void RovRviz::publish_cov_ellipsoid(const std::string& rov_frame, const std::string& frame_id,
                                    ros::Publisher& pub, const Eigen::MatrixXd& P,
                                    const geometry_msgs::PoseStamped& pose_estimated){
    visualization_msgs::Marker marker_elli;
    marker_elli.header.frame_id = frame_id;
    marker_elli.header.stamp = ros::Time();
    marker_elli.ns = "/"+rov_frame;
    marker_elli.id = 0;
    marker_elli.type = visualization_msgs::Marker::SPHERE;
    marker_elli.action = visualization_msgs::Marker::ADD;

    // project P on two first dimensions
    // get the eigen values and eigen vectors of P
    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> es(P.block(0,0,2,2));
    Eigen::VectorXd eigen_values = es.eigenvalues();
    Eigen::MatrixXd eigen_vectors = es.eigenvectors();

    // get the angle of the fist eigenvector
    double theta = atan2(eigen_vectors(1,0),eigen_vectors(0,0));

    tf::Quaternion q;
    q.setRPY(0, 0, theta-M_PI/2); // rotation of the mesh
    marker_elli.pose.orientation.x = q.x();
    marker_elli.pose.orientation.y = q.y();
    marker_elli.pose.orientation.z = q.z();
    marker_elli.pose.orientation.w = q.w();

    marker_elli.pose.position.x = pose_estimated.pose.position.x;
    marker_elli.pose.position.y = pose_estimated.pose.position.y;
    marker_elli.pose.position.z = pose_estimated.pose.position.z;

    marker_elli.scale.x = sqrt(eigen_values(0));
    marker_elli.scale.y = sqrt(eigen_values(1));
    marker_elli.scale.z = 1.; // arbitrary value on the z axis (just to draw the hozizontal ellipsoid)
    marker_elli.color.a = 0.2;

    set_color(marker_elli, "estimation_"+rov_frame);
    pub.publish(marker_elli);
}

void RovRviz::publish_target(){
    broadcast_tf_pose_i(pose_target_rovA, "target_"+rovA_name);
    broadcast_tf_pose_i(pose_target_rovB, "target_"+rovB_name);
    publish_marker_i(rovA_name,"target_"+rovA_name,0.25,pub_marker_rovA_target);
    publish_marker_i(rovB_name,"target_"+rovB_name,0.25,pub_marker_rovB_target);
}

void RovRviz::broadcast_tf_pose_i(const geometry_msgs::PoseStamped& msg, const std::string& frame_name)
{
    tf::Transform transform;
    transform.setOrigin( tf::Vector3(msg.pose.position.x, msg.pose.position.y, msg.pose.position.z) );
    transform.setRotation( tf::Quaternion(msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w) );
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", frame_name));
}

void RovRviz::publish_marker_i(const std::string& rov_frame, const std::string& frame_id, float alpha, ros::Publisher& pub){
    visualization_msgs::Marker marker_rov;
    marker_rov.header.frame_id = frame_id;
    marker_rov.header.stamp = ros::Time();
    marker_rov.ns = "/"+rov_frame;
    marker_rov.id = 0;
    marker_rov.type = visualization_msgs::Marker::MESH_RESOURCE;
    marker_rov.action = visualization_msgs::Marker::ADD;
    marker_rov.mesh_resource = "package://rov_rviz_ros1/meshs/ROV_simple.stl";

    tf::Quaternion q;
    q.setRPY(0, 0, -M_PI/2); // rotation of the mesh
    marker_rov.pose.orientation.x = q.x();
    marker_rov.pose.orientation.y = q.y();
    marker_rov.pose.orientation.z = q.z();
    marker_rov.pose.orientation.w = q.w();

    marker_rov.pose.position.z = -0.1;

    marker_rov.scale.x = 0.01;
    marker_rov.scale.y = 0.01;
    marker_rov.scale.z = 0.01;
    marker_rov.color.a = alpha;

    set_color(marker_rov, frame_id);

    pub.publish(marker_rov);
}

void RovRviz::publish_control_input_i(const std::string& rov_frame){
    // publish the twist of the ROV
    if(rov_frame == rovA_name) {
        control_input_rovA.header.stamp = ros::Time::now();
        control_input_rovA.header.frame_id = rov_frame;
        pub_control_input_rovA.publish(control_input_rovA);
    }else if(rov_frame == rovB_name) {
        control_input_rovB.header.stamp = ros::Time::now();
        control_input_rovB.header.frame_id = rov_frame;
        pub_control_input_rovB.publish(control_input_rovB);
    } else {
        ROS_ERROR("Invalid name for ROV control_input: %s", rov_frame.c_str());
    }
}

void RovRviz::publish_control_input(){
    publish_control_input_i(rovA_name);
    publish_control_input_i(rovB_name);
}

void RovRviz::publish_triangle(){
    // create marker message
    visualization_msgs::Marker marker_triangle;
    marker_triangle.header.frame_id = "map";
    marker_triangle.header.stamp = ros::Time();
    marker_triangle.id = 0;
    marker_triangle.type = visualization_msgs::Marker::LINE_STRIP;
    marker_triangle.action = visualization_msgs::Marker::ADD;
    marker_triangle.scale.x = 0.1;
    marker_triangle.color.a = 1.0;
    marker_triangle.color.r = 0.0;
    marker_triangle.color.g = 1.0;
    marker_triangle.color.b = 0.0;

    marker_triangle.pose.orientation.w = 1.0;

    // add the points
    geometry_msgs::Point pA;
    pA.x = pose_target_rovA.pose.position.x;
    pA.y = pose_target_rovA.pose.position.y;
    pA.z = pose_target_rovA.pose.position.z;
    marker_triangle.points.push_back(pA);

    geometry_msgs::Point pB;
    pB.x = pose_target_rovB.pose.position.x;
    pB.y = pose_target_rovB.pose.position.y;
    pB.z = pose_target_rovB.pose.position.z;
    marker_triangle.points.push_back(pB);

    geometry_msgs::Point p;
    p.x = 0;
    p.y = 0;
    p.z = 0;
    marker_triangle.points.push_back(p);
    marker_triangle.points.push_back(pA);

    // publish the marker
    pub_triangle.publish(marker_triangle);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "rov_rviz_ros1");
    ros::NodeHandle n;
    RovRviz RR(n);
    ros::Rate loop_rate(RR.rate);

    while(ros::ok()){
        RR.kalman_predict();
//        RR.pose_estimation_prediction();
        RR.publish_robots_estimation();
        RR.publish_robots_measurement();
        RR.publish_target();
        RR.publish_control_input();
        RR.publish_triangle();
//        RR.publish_measured_position();
//        RR.publish_z_vector();
//        RR.publish_estimated_distance_error();
        ros::spinOnce();
        loop_rate.sleep();
    }
}

//void  RovRviz::pose_estimation_prediction(){
//    pose_estimation_prediction_i(pose_estimated_rovA, twist_estimated_rovA, control_input_rovA);
//    pose_estimation_prediction_i(pose_estimated_rovB, twist_estimated_rovB, control_input_rovB);
//}


//void RovRviz::pose_estimation_prediction_i(geometry_msgs::PoseStamped& pose_estimated, geometry_msgs::TwistStamped& twist_estimated, geometry_msgs::WrenchStamped& wrench){
//
//    // step 1 update the pose
//
//    // get the orientation matrix
//    Eigen::Quaterniond q(pose_estimated.pose.orientation.w, pose_estimated.pose.orientation.x,
//                         pose_estimated.pose.orientation.y, pose_estimated.pose.orientation.z);
//    Eigen::Matrix3d R = q.normalized().toRotationMatrix();
//
//    // convert twist from robot frame to world frame
//    Eigen::Vector3d speed_r(twist_estimated.twist.linear.x, twist_estimated.twist.linear.y, twist_estimated.twist.linear.z);
//    Eigen::Vector3d speed_w(R*speed_r);
//
//    // update pose with euler method
//    double dt = 1/rate;
//    pose_estimated.pose.position.x += speed_w(0)*dt;
//    pose_estimated.pose.position.y += speed_w(1)*dt;
//
//    // step 2 update the twist
//    double du = DL(0,0) + DNL(0,0)*abs(speed_r(0)); // linear damping matrix, diagonal matrix
//    double dv = DL(1,1) + DNL(1,1)*abs(speed_r(1));
//
//    double damping_u = -du*speed_r(0);
//    double damping_v = -dv*speed_r(1);
//
//    double accel_u = 1/m*(wrench.wrench.force.x+damping_u);
//    double accel_v = 1/m*(wrench.wrench.force.y+damping_v);
//
////    ROS_INFO("Hello ros named %s", rovA_name.c_str());
////    ROS_INFO("Wrench is: %f %f", wrench.wrench.force.x, wrench.wrench.force.y);
////    ROS_INFO("dampling is: %f %f", damping_u, damping_v);
////    ROS_INFO("pose_estimated: %f %f", pose_estimated.pose.position.x, pose_estimated.pose.position.y);
////    ROS_INFO("acceleration_r: %f %f", accel_u, accel_v);
//
//    // update the twist with euler method
//    twist_estimated.twist.linear.x += accel_u*dt;
//    twist_estimated.twist.linear.y += accel_v*dt;
//
////    ROS_INFO("twist_estimated: %f %f", twist_estimated.twist.linear.x, twist_estimated.twist.linear.y);
//}


//void RovRviz::publish_measured_position(){
//    pub_measured_position_rovA.publish(pose_measured_rovA);
//    pub_measured_position_rovB.publish(pose_measured_rovB);
//}
//void RovRviz::publish_z_vector(){
//    geometry_msgs::Vector3Stamped z_vector;
//    z_vector.header.frame_id = "map";
//    z_vector.header.stamp = ros::Time::now();
//
//    double pax = pose_measured_rovA.pose.position.x;
//    double pay = pose_measured_rovA.pose.position.y;
//    double pbx = pose_measured_rovB.pose.position.x;
//    double pby = pose_measured_rovB.pose.position.y;
//
////    double da = p_rovA.norm();
////    double db = p_rovB.norm();
//    double da = sqrt(pow(pax,2)+pow(pay,2));
//    double db = sqrt(pow(pbx,2)+pow(pby,2));
//
//    // norm of the pose fo the two targets
//    double dd = 0.5*sqrt(pow(pose_target_rovA.pose.position.x,2)+
//            pow(pose_target_rovA.pose.position.y,2)) +
//            0.5*sqrt(pow(pose_target_rovB.pose.position.x,2)+
//                     pow(pose_target_rovB.pose.position.y,2));
//
//    // amer of the ROVs
////    double tha = atan2(p_rovA[1],p_rovA[0]);
////    double thb = atan2(p_rovB[1],p_rovB[0]);
//    double tha = atan2(pay,pax);
//    double thb = atan2(pby,pbx);
//
//    z_vector.vector.x = da/dd-1; // distance error of the ROVa
//    z_vector.vector.y = db/dd-1; // distance error of the ROVb
//    z_vector.vector.z = abs(sawtooth(tha-thb))-M_PI/3; // amer between the two Rovs
//    pub_z_vector.publish(z_vector);
//}
//void RovRviz::publish_estimated_distance_error(){
//    // compute the distances between the ROVs and their targets
//    float distance_rovA = sqrt(pow(p_rovA[0]-pose_target_rovA.pose.position.x,2)+
//            pow(p_rovA[1]-pose_target_rovA.pose.position.y,2));
//
//    float distance_rovB = sqrt(pow(p_rovB[0]-pose_target_rovB.pose.position.x,2)+
//            pow(p_rovB[1]-pose_target_rovB.pose.position.y,2));
//
//    // publish the distances
//    std_msgs::Float64 msg_distance_rovA;
//    msg_distance_rovA.data = distance_rovA;
//    pub_estimated_distance_error_rovA.publish(msg_distance_rovA);
//
//    std_msgs::Float64 msg_distance_rovB;
//    msg_distance_rovB.data = distance_rovB;
//    pub_estimated_distance_error_rovB.publish(msg_distance_rovB);
//}