//
// Created by jbs on 18. 6. 29.
//

#ifndef ASAP_ASAP_H
#define ASAP_ASAP_H

// CUSTOM HEADERS
#include "utils.h"




class ASAP{

private:
    // parameters for ASAP algorithm
    asap_ns::Params params;
    // linspace for azimuth(col) , elevation(row) set
    Eigen::VectorXf azim_set, elev_set;
    // target history
    TargetHistory target_history;
    // predicted target history
    TargetPrediction target_prediction;
    // octomap object
    octomap::OcTree*  octree_obj;
    // set of layers of candidate nodes
    asap_ns::LayerSet cur_layer_set;
    // current position of tracker
    geometry_msgs::Point cur_tracker_pos;
    // sequnce of viewpoints
    ViewSequence view_path;
    // graph for VP computation
    Graph g;
    // dictionary for vertex descriptor
    DescriptorMap descriptor_map;
    // color map for plotting the candidate node
    cv::Mat bgr[3];




public:
    // constructor destructor
    ASAP(asap_ns::Params);
    ~ASAP();

    // graph
    void graph_init();  // node add wtih current position of tracker
    asap_ns::Layer get_layer(geometry_msgs::Point,int);
    void add_layer(asap_ns::Layer); // add the layer and connect it with the last layer
    void graph_wrapping(); // add the last layer
    void solve_view_path(); // update view sequence of tracker by solving Dijkstra's shortest path algorithm
    MatrixXd castRay(geometry_msgs::Point,float,bool=false); // cast ray in octomap


    // ROS
    ros::NodeHandle nh; // getting parameters

    ros::Subscriber octomap_sub;
    ros::Subscriber states_sub;
    ros::Subscriber points_sub;

    ros::Publisher candidNodes_marker_pub; // points of local maximum in visibility matrix
    ros::Publisher pnts_pub; // points (clicked points)
    ros::Publisher path_pub; // view path
    ros::Publisher node_pub; // node marker publisher
    ros::Publisher edge_pub; // edge arrow publisher




    ros::ServiceServer solve_server; // server for solving view path

    // id
    std::string world_frame_id; // maybe world

    // rviz

    visualization_msgs::Marker marker; // marker for candidate nodes
    visualization_msgs::Marker pnt_marker; // marker for pnts
    visualization_msgs::Marker node_marker; // marker for nodes
    visualization_msgs::Marker edge_marker; // marker for edges
    visualization_msgs::MarkerArray arrow_array; // array of arrow
	int edge_id;

	



    /***  FUNCTIONS
     */

    // publish
    void marker_publish(); // marker (line list) for candidate nodes
    void points_publish(); // for the purpose of test (publish the received points)
    void path_publish();   // solution path publication



    // callback (subsrcibe)
    void points_callback(kiro_gui_msgs::PositionArray); // this callback function is for getting point from rviz
    void state_callback(const gazebo_msgs::ModelStates::ConstPtr&);
    bool solve_callback(asap::SolvePath::Request&,asap::SolvePath::Response&); // service callback
    void octomap_callback(const octomap_msgs::Octomap&);


    // flags
    bool state_callback_flag;
    bool octomap_callback_flag;


    // tracker name
    std::string tracker_name;
    // target name
    std::string target_name;



};



#endif //ASAP_ASAP_H
