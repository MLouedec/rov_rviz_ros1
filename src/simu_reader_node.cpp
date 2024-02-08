//
// Created by morgan on 12/10/23.
//  read the json file of the ROV simulation to publish the data to the ROS system for a Rviz display
//

//#include <chrono>
//#include <string>
#include <fstream>
#include <jsoncpp/json/json.h>
//
//#include <rclcpp/rclcpp.hpp>
//#include <tf2_ros/transform_broadcaster.h>
//#include <tf2/LinearMath/Quaternion.h>
//#include <geometry_msgs/msg/transform_stamped.hpp>
//#include <visualization_msgs/msg/marker.hpp>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>


//using namespace std::chrono_literals;

class NodeDataReader{
public:
    NodeDataReader(ros::NodeHandle &nh){
        // get the filepath from the parameters
        std::string filepath;

        if(nh.getParam("/file_path", filepath)){
            ROS_INFO("Got param: %s", filepath.c_str());
        }else{
            ROS_ERROR("Failed to get param 'file_path'");
        }

        // print the filepath
        ROS_INFO("filepath: %s", filepath.c_str());

        // import the Json file and parse it
        std::ifstream file(filepath,std::ifstream::binary);
        Json::Value data;
        file >> data;

        N = data["header"]["N"].asInt(); // number of ROVs
        dt = data["header"]["dt"].asDouble(); // time step
        simu_data = data["simu_data"];
        k=0;
        k_max = simu_data.size();

        // ROV marker publisher
        pub_marker_clyde = nh.advertise<visualization_msgs::Marker>("/clyde/marker",10);
        pub_marker_blinky = nh.advertise<visualization_msgs::Marker>("/blinky/marker",10);

        marker.id = 0;
        marker.type = visualization_msgs::Marker::MESH_RESOURCE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.mesh_resource = "package://rov_rviz_ros1/meshs/ROV_simple.stl";
        marker.scale.x = 0.01;
        marker.scale.y = 0.01;
        marker.scale.z = 0.01;
        marker.color.a = 1.0;

        // time marker publisher
        pub_time_marker = nh.advertise<visualization_msgs::Marker>("/time_marker",10);

        t_init= ros::Time::now();
    }

    void read_and_publish(){
        if(k<k_max) { // if there is data to read
            // extract data from the json file
            ros::Duration t_simu = ros::Duration(simu_data[k]["t"].asDouble());
            Json::Value simu_data_i = simu_data[k]["L_eta"];
            ROS_INFO("Simu Time is : %f", t_simu.toSec());

            // wait for the simu time
            ros::Time t_ = ros::Time::now();
            ros::Duration t_wait = t_simu - (t_ - t_init);
            if (t_wait.toSec() > 0) {
                ROS_INFO("wait for %f", t_wait.toSec());
                ros::Duration(t_wait).sleep();
            }

            for (int j = 0; j < N; j++) { // for each ROV

                // send the tf transformations for each ROV
                std::string rov_frame = "unknown";
                if (j == 0) {
                    rov_frame = "blinky";
                } else if (j == 1) {
                    rov_frame = "clyde";
                }

                float roll = simu_data_i[j][3].asDouble();
                float pitch = simu_data_i[j][4].asDouble();
                float yaw = simu_data_i[j][5].asDouble();

                q.setRPY(roll, pitch, yaw);
                ts.setOrigin(tf::Vector3(simu_data_i[j][0].asDouble(), simu_data_i[j][1].asDouble(),
                                         simu_data_i[j][2].asDouble()));
                ts.setRotation(tf::Quaternion(q.x(), q.y(), q.z(), q.w()));
                br.sendTransform(tf::StampedTransform(ts, ros::Time::now(), "map", rov_frame));

                // display the marker of the ROVs
                marker.header.stamp = ros::Time::now();
                q.setRPY(0, 0, -M_PI/2);
                marker.pose.orientation.x = q.x();
                marker.pose.orientation.y = q.y();
                marker.pose.orientation.z = q.z();
                marker.pose.orientation.w = q.w();

                if(j==0){
                    marker.color.r = 1.0;
                    marker.color.g = 0.0;
                    marker.color.b = 0.0;
                    marker.header.frame_id = "blinky";
                    marker.ns = "/blinky";
                    pub_marker_blinky.publish(marker);
                }else{
                    marker.color.r = 1.0;
                    marker.color.g = 0.5;
                    marker.color.b = 0.0;
                    marker.header.frame_id = "clyde";
                    marker.ns = "/clyde";
                    pub_marker_clyde.publish(marker);
                }
            }

            // publish the time of the simulation
            visualization_msgs::Marker time_marker;
            time_marker.header.frame_id = "map";
            time_marker.header.stamp = ros::Time::now();
            time_marker.ns = "time";
            time_marker.id = 0;
            time_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
            time_marker.action = visualization_msgs::Marker::ADD;
            time_marker.pose.position.x = 0.0;
            time_marker.pose.position.y = 20.0;
            time_marker.pose.position.z = 1.0;
            time_marker.pose.orientation.x = 0.0;
            time_marker.pose.orientation.y = 0.0;
            time_marker.pose.orientation.z = 0.0;
            time_marker.pose.orientation.w = 1.0;
            time_marker.scale.x = 1.;
            time_marker.scale.y = 1.;
            time_marker.scale.z = 1.;
            time_marker.color.a = 1.0;
            time_marker.color.r = 1.0;
            time_marker.color.g = 1.0;
            time_marker.color.b = 1.0;
            time_marker.text = "Simulation Time: " + std::to_string(t_simu.toSec()) + " s";
            pub_time_marker.publish(time_marker);
            k += 1;
        }else{
            ROS_INFO("End of the simulation");
        }
    }

    double rate = 10.0; // Hz
private:
    tf::Quaternion q;
    tf::Transform ts;
    visualization_msgs::Marker marker;
    tf::TransformBroadcaster br;
    ros::Publisher pub_marker_clyde; // publisher for the marker of the ROV clyde
    ros::Publisher pub_marker_blinky; // publisher for the marker of the ROV blinky
    ros::Publisher pub_time_marker; // text marker publisher for the time of the simulation

    int N;
    double dt;
    int k; // number of time steps
    int k_max; // maximum number of time steps
    Json::Value simu_data;
    ros::Time t_init; // ros time of the initialization of the node
};

int main(int argc, char * argv[]){
    ros::init(argc, argv, "rov_rviz_ros1");
    ros::NodeHandle n;
    NodeDataReader NDR(n);
    ros::Rate loop_rate(NDR.rate);

    while(ros::ok()){
        NDR.read_and_publish();
        ros::spinOnce();
        loop_rate.sleep();
    }
}
