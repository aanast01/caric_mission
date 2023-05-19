/*
 * Copyright (C) 2012 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */

#include <thread>
#include <chrono>

#include <iostream>
#include <Eigen/Eigen>

#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/gazebo_client.hh>
#include <gazebo/gazebo_client.hh>
#include <gazebo/gazebo_config.h>

#include <gazebo/physics/World.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/physics/JointWrench.hh>

#include "rotors_comm/PPComTopology.h"

#include "ros/ros.h"
#include "visualization_msgs/Marker.h"
#include "std_msgs/ColorRGBA.h"

#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/conditional_removal.h>

#include "tcc/Stop.h"
#include "utility.h"

// Printout colors
#define KNRM "\x1B[0m"
#define KRED "\x1B[31m"
#define KGRN "\x1B[32m"
#define KYEL "\x1B[33m"
#define KBLU "\x1B[34m"
#define KMAG "\x1B[35m"
#define KCYN "\x1B[36m"
#define KWHT "\x1B[37m"
#define RESET "\033[0m"

using namespace std;
using namespace Eigen;

// Handles to ROS and Gazebo object
gazebo::physics::WorldPtr world = NULL;
ros::NodeHandlePtr nh_ptr;

// Topology and status checks
std::mutex topoMtx;
typedef nav_msgs::Odometry rosOdom;
int Nnodes = 0;
vector<string>  nodeName;
vector<string>  nodeRole;
vector<rosOdom> nodeOdom;
vector<string>  nodeStatus;
vector<bool>    nodeAlive;
MatrixXd        linkMat;

// Visualization
typedef visualization_msgs::Marker RosVizMarker;
typedef std_msgs::ColorRGBA RosVizColor;
typedef ros::Publisher RosPub;

struct VizAid
{
    RosVizColor    color  = RosVizColor();
    RosVizMarker   marker = RosVizMarker();
    ros::Publisher rosPub = RosPub();
};

// Predefined colors
VizAid vizAid;
RosVizColor los_color;
RosVizColor nlos_color;
RosVizColor dead_color;

// Map 
CloudXYZPtr globalMap(new CloudXYZ());
static pcl::KdTreeFLANN<PointXYZ> kdTreeglobalMap;

/////////////////////////////////////////////////
// Function is called everytime a message is received.
void ContactCallback(ConstContactsPtr &_msg)
{
    if (Nnodes == 0)
        return;

    topoMtx.lock();

    vector<bool> deadNode(Nnodes, false);
    for (int i = 0; i < _msg->contact_size(); ++i)
    {
        string col1 = _msg->contact(i).collision1();
        string col2 = _msg->contact(i).collision2();

        // Check if the node name shows up in collision, if it does, skip it
        for(int node_idx = 0; node_idx < Nnodes; node_idx++)
        {
            if (nodeRole[node_idx] == "manager")
                continue;

            bool colide_case1 = col1.find(nodeName[node_idx]) != std::string::npos
                                && col2.find("ground_plane") == std::string::npos;
            bool colide_case2 = col2.find(nodeName[node_idx]) != std::string::npos
                                && col1.find("ground_plane") == std::string::npos;
            bool on_air = nodeStatus[node_idx] == "on_air";

            // if (col1.find("gcs") != std::string::npos || col2.find("gcs") != std::string::npos)
            //     continue;

            // printf("Collision %s <-> %s. case1 %d. case2: %d. status: %s. %d\n",
            //         col1.c_str(), col2.c_str(), colide_case1, colide_case2, nodeStatus[node_idx].c_str(), on_air);

            if((colide_case1 || colide_case2) && on_air)
            {
                deadNode[node_idx] = true;
                printf("Node %d, %s (role %s) collides with %s.\n",
                        node_idx, nodeRole[node_idx].c_str(), nodeName[node_idx].c_str(),
                        colide_case1 ? col2.c_str() : col1.c_str());
                
                for(int k = 0; k < 1; k++)
                {
                    // auto force = _msg->contact(i).wrench(k).body_1_wrench().force();
                    // printf("wr %d: %6.3f. %6.3f. %6.3f. %s\n", k, force.x(), force.y(), force.z(), _msg->contact(i).world().c_str());
                    // force = _msg->contact(i).wrench(k).body_2_wrench().force();
                    // printf("wr %d: %6.3f. %6.3f. %6.3f. %s\n", k, force.x(), force.y(), force.z(), _msg->contact(i).world().c_str());
                    
                    // Command the drones to fall
                    tcc::Stop stop;
                    stop.request.message = KRED "Collision happens over " + nodeName[node_idx] + ". Control Stopped!" RESET;
                    ros::service::call("/" + nodeName[node_idx] + "/stop", stop);
                }

                // Set the state of the drone as dead
                nodeAlive[node_idx] = false;
            }
        }
    }

    topoMtx.unlock();
}

void PPComCallback(const rotors_comm::PPComTopology::ConstPtr &msg)
{
    static bool firstshot = true;

    topoMtx.lock();

    // printf(KGRN "PPComCallback\n" RESET);

    if (firstshot)
    {
        Nnodes      = msg->node_id.size();
        nodeName    = msg->node_id;
        nodeRole    = msg->node_role;
        nodeOdom    = msg->node_odom;
        nodeStatus  = vector<string>(Nnodes, "on_ground"); // Assuming that all drones are static on the ground
        nodeAlive   = vector<bool>(Nnodes, true); // Assuming that all nodes are initially alive, one only dies when colliding stuff
        linkMat     = -Eigen::MatrixXd::Ones(Nnodes, Nnodes);
        firstshot   = false;
    }

    // Update the states
    nodeOdom = msg->node_odom;
    assert(nodeOdom.size() == Nnodes);

    // Update the distance
    int range_idx = 0;
    for(int i = 0; i < Nnodes; i++)
    {
        for(int j = i+1; j < Nnodes; j++)
        {   
            linkMat(i,j) = msg->range[range_idx];
            linkMat(j,i) = linkMat(i,j);
            range_idx++;
        }
    }

    // Update the status
    for(int i = 0; i < Nnodes; i++)
    {
        Vector3d vel(nodeOdom[i].twist.twist.linear.x,
                     nodeOdom[i].twist.twist.linear.y,
                     nodeOdom[i].twist.twist.linear.z);
        
        if (nodeOdom[i].pose.pose.position.z > 0.1 && vel.norm() > 0.1)
            nodeStatus[i] = "on_air";

        if (nodeOdom[i].pose.pose.position.z < 0.1 && vel.norm() < 0.1)
            nodeStatus[i] = "on_ground";
    }

    topoMtx.unlock();

    // Update the link visualization
    vizAid.marker.points.clear();
    vizAid.marker.colors.clear();
    for(int i = 0; i < Nnodes; i++)
    {
        if (nodeOdom[i].pose.covariance[0] < 0.0)
            continue;

        for(int j = i+1; j < Nnodes; j++)
        {
            if (nodeOdom[j].pose.covariance[0] < 0.0)
                continue;
            
            // If either node is dead:
            if( !nodeAlive[i] || !nodeAlive[j] )
            {
                // printf("Node %s or %s is dead\n", nodeName[i], nodeName[j]);
                vizAid.marker.points.push_back(nodeOdom[i].pose.pose.position);
                vizAid.marker.colors.push_back(dead_color);
                vizAid.marker.points.push_back(nodeOdom[j].pose.pose.position);
                vizAid.marker.colors.push_back(dead_color);
                continue;
            }
            
            // If there is line of sight
            if (linkMat(i, j) > 0.0)
            {
                // printf("Node %d, %d has line of sight\n", i, j);
                vizAid.marker.points.push_back(nodeOdom[i].pose.pose.position);
                vizAid.marker.colors.push_back(los_color);
                vizAid.marker.points.push_back(nodeOdom[j].pose.pose.position);
                vizAid.marker.colors.push_back(los_color);
            }
            else
            {
                // printf("Node %d, %d lacks line of sight\n", i, j);
                vizAid.marker.points.push_back(nodeOdom[i].pose.pose.position);
                vizAid.marker.colors.push_back(nlos_color);
                vizAid.marker.points.push_back(nodeOdom[j].pose.pose.position);
                vizAid.marker.colors.push_back(nlos_color);
            }
        }
    }

    vizAid.rosPub.publish(vizAid.marker);
}

// Callback function for poses
void odomTimerCallback(const ros::TimerEvent& event)
{

}

// Callback function for pointcloud
void cloudTimerCallback(const ros::TimerEvent& event)
{
    // if (Nnodes == 0)
    //     return;
    
    // // Duplicate the tree of global map 
    // static vector<pcl::KdTreeFLANN<PointXYZ>> kdTreeglobalMapDup(Nnodes, kdTreeglobalMap);
    // static bool firstshot = true;
    // // if (firstshot)
    // // {
    // //     firstshot = false;
    // // }

    // // Get the node poses
    // topoMtx.lock();
    // vector<rosOdom> nodeOdom_ = nodeOdom;
    // topoMtx.unlock();

    // // Find the pointcloud seen within a radius by each node
    // for(int i = 1; i < 2; i++)
    // {
    //     // PointXYZ pos;
    //     // pos.x = nodeOdom[i].pose.pose.position.x;
    //     // pos.y = nodeOdom[i].pose.pose.position.y;
    //     // pos.z = nodeOdom[i].pose.pose.position.z;
        
    //     // vector<int> point_idx; vector<float> point_sq_dist;
    //     // kdTreeglobalMapDup[i].radiusSearch(pos, 25, point_idx, point_sq_dist);

    //     CloudXYZPtr localCloud(new CloudXYZ());

    //     // Copy all points belonging to the cloud_local
    //     pcl::ConditionAnd<PointXYZ>::Ptr cyl_cond (new pcl::ConditionAnd<PointXYZ>());

    //     Eigen::Matrix3f cylinderMatrixRad;
    //     cylinderMatrixRad << 1.0, 0.0, 0.0,
    //                          0.0, 1.0, 0.0,
    //                          0.0, 0.0, 0.0;
    //     float cylinderRad = -25*25; //radius² of cylinder
    //     pcl::TfQuadraticXYZComparison<PointXYZ>::Ptr cyl_comp1
    //         (new pcl::TfQuadraticXYZComparison<PointXYZ>
    //             (pcl::ComparisonOps::LT, cylinderMatrixRad, Eigen::Vector3f::Zero(), cylinderRad));
    //     cyl_comp1->transformComparison(myTf(nodeOdom_[i]).inverse().cast<float>().tfMat());
    //     cyl_cond->addComparison(cyl_comp1);

    //     Eigen::Matrix3f cylinderMatrixZPositive;
    //     cylinderMatrixZPositive << 0.0, 0.0, 0.0,
    //                                0.0, 0.0, 0.0,
    //                                0.0, 0.0, 0.0;
    //     float cylinderZpositive = 10.35;
    //     pcl::TfQuadraticXYZComparison<PointXYZ>::Ptr cyl_comp2
    //         (new pcl::TfQuadraticXYZComparison<PointXYZ>
    //             (pcl::ComparisonOps::GT, cylinderMatrixZPositive, Eigen::Vector3f(0.0, 0.0, 0.5), cylinderZpositive));
    //     cyl_comp2->transformComparison(myTf(nodeOdom_[i]).inverse().cast<float>().tfMat());
    //     cyl_cond->addComparison(cyl_comp2);

    //     Eigen::Matrix3f cylinderMatrixZNegative;
    //     cylinderMatrixZNegative << 0.0, 0.0, 0.0,
    //                                0.0, 0.0, 0.0,
    //                                0.0, 0.0, 0.0;
    //     float cylinderZNegative = -10.35;
    //     pcl::TfQuadraticXYZComparison<PointXYZ>::Ptr cyl_comp3
    //         (new pcl::TfQuadraticXYZComparison<PointXYZ>
    //             (pcl::ComparisonOps::LT, cylinderMatrixZNegative, Eigen::Vector3f(0.0, 0.0, 0.5), cylinderZNegative));    
    //     cyl_comp3->transformComparison(myTf(nodeOdom_[i]).inverse().cast<float>().tfMat());
    //     cyl_cond->addComparison(cyl_comp3);

    //     pcl::ConditionalRemoval<PointXYZ> condrem;
    //     condrem.setCondition(cyl_cond);
    //     condrem.setInputCloud(globalMap);
    //     condrem.setKeepOrganized (false);

    //     condrem.filter(*localCloud);

    //     static ros::Publisher localCloudPub = nh_ptr->advertise<sensor_msgs::PointCloud2>("/local_map", 1);

    //     // Publish the pointcloud
    //     Util::publishCloud(localCloudPub, *localCloud, ros::Time::now(), string("world"));
    // }
}

/////////////////////////////////////////////////
int main(int argc, char **argv)
{
    // Load gazebo
    gazebo::client::setup(argc, argv);

    // Create a gazebo node to subscribe to gazebo environments
    gazebo::transport::NodePtr node(new gazebo::transport::Node());
    node->Init();

    // Listen to Gazebo world_stats topic
    gazebo::transport::SubscriberPtr sub = node->Subscribe("/gazebo/default/physics/contacts", ContactCallback);

    // Create a ros node to subscribe to ros environment
    ros::init(argc, argv, "MissionManager");
    ros::NodeHandle nh("~");
    nh_ptr = boost::make_shared<ros::NodeHandle>(nh);

    // Subscribe to the ppcom topology
    printf(KGRN "Subscribing to ppcom_topology\n" RESET);
    ros::Subscriber ppcomSub = nh_ptr->subscribe("/gcs/ppcom_topology", 1, PPComCallback);

    // world = gazebo::physics::get_world("default");
    
    // Load the original pointcloud
    string pcd_file;
    nh_ptr->getParam("pcd_file", pcd_file);
    printf(KGRN "Global Map file: %s\n" RESET, pcd_file.c_str());
    pcl::io::loadPCDFile<PointXYZ>(pcd_file, *globalMap);
    kdTreeglobalMap.setInputCloud(globalMap);

    // Create a timer to publish the pointclouds at 10Hz
    double odom_rate = 10.0;
    nh_ptr->getParam("odom_rate", odom_rate);
    ros::Timer odom_timer = nh_ptr->createTimer(ros::Duration(1.0/odom_rate), odomTimerCallback);

    // Create a timer to publish the pointclouds at 10Hz
    double cloud_rate = 10.0;
    nh_ptr->getParam("cloud_rate", cloud_rate);
    ros::Timer cloud_timer = nh_ptr->createTimer(ros::Duration(1.0/cloud_rate), cloudTimerCallback);

    // Initialize visualization
    los_color.r  = 0.0; los_color.g  = 1.0;  los_color.b  = 0.5; los_color.a  = 1.0;
    nlos_color.r = 1.0; nlos_color.g = 0.65; nlos_color.b = 0.0; nlos_color.a = 0.5;
    dead_color.r = 0.2; dead_color.g = 0.2;  dead_color.b = 0.2; dead_color.r = 1.0;

    vizAid.rosPub = nh_ptr->advertise<RosVizMarker>("/topology_marker", 1);
    vizAid.marker.header.frame_id = "world";
    vizAid.marker.ns       = "loop_marker";
    vizAid.marker.type     = visualization_msgs::Marker::LINE_LIST;
    vizAid.marker.action   = visualization_msgs::Marker::ADD;
    vizAid.marker.pose.orientation.w = 1.0;
    vizAid.marker.lifetime = ros::Duration(0);
    vizAid.marker.id       = 0;

    vizAid.marker.scale.x = 0.15;
    vizAid.marker.scale.y = 0.15;
    vizAid.marker.scale.z = 0.15;

    vizAid.marker.color.r = 0.0;
    vizAid.marker.color.g = 1.0;
    vizAid.marker.color.b = 1.0;
    vizAid.marker.color.a = 1.0;

    vizAid.color = nlos_color;

    vizAid.marker.points.clear();
    vizAid.marker.colors.clear();

    ros::MultiThreadedSpinner spinner(0);
    spinner.spin();

    // Make sure to shut everything down.
    gazebo::client::shutdown();

}