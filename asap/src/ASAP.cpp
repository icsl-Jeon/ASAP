//
// Created by jbs on 18. 6. 29.
//


#include "ASAP.h"

using namespace asap_ns;

// constructor

ASAP::ASAP(Params params):nh("~"){
    // parameter parsing
    this->params=params;


    nh.getParam("world_frame_id",world_frame_id);

    std::cout<<"world frame id:"<<world_frame_id<<std::endl;
    // subscribe
    octomap_sub=nh.subscribe("/octomap_full",3,&ASAP::octomap_callback,this);
    states_sub=nh.subscribe("/gazebo/model_states",10,&ASAP::state_callback,this);
    points_sub=nh.subscribe("/search_position_array",10,&ASAP::points_callback,this);


    // advertise
    path_pub=nh.advertise<nav_msgs::Path>("view_sequence",3);
    pnts_pub=nh.advertise<visualization_msgs::Marker>("clicked_pnts",2);
    candidNodes_marker_pub=nh.advertise<visualization_msgs::Marker>("candidate_nodes",2);
    node_pub=nh.advertise<visualization_msgs::Marker>("nodes_in_layer",2);
    edge_pub=nh.advertise<visualization_msgs::MarkerArray>("edge_in_layer",2);


    // service
    solve_server = nh.advertiseService("solve_path",&ASAP::solve_callback,this);

    // candid nodes marker init
    marker.header.frame_id = world_frame_id;
    marker.header.stamp  = ros::Time::now();
    marker.ns = "candidate_nodes";
    marker.action = visualization_msgs::Marker::ADD;
    float scale=0.1;
    marker.pose.orientation.w = 1.0;
    marker.id = 0;
    marker.type = visualization_msgs::Marker::SPHERE_LIST;
    marker.scale.x = scale;
    marker.scale.y = scale;
    marker.scale.z = scale;
    marker.color.r = 1;
    marker.color.a = 0.5;

    // pnts marker init
    pnt_marker.header.frame_id = world_frame_id;
    pnt_marker.header.stamp  = ros::Time::now();
    pnt_marker.ns = "clicked_pnts";
    pnt_marker.action = visualization_msgs::Marker::ADD;
    float len=0.2;
    pnt_marker.pose.orientation.w = 1.0;
    pnt_marker.id = 0;
    pnt_marker.type = visualization_msgs::Marker::CUBE_LIST;
    pnt_marker.scale.x = len;
    pnt_marker.scale.y = len;
    pnt_marker.scale.z = len;
    pnt_marker.color.r = 1;
    pnt_marker.color.g = 0.3;
    pnt_marker.color.a = 0.8;

    // nodes marker init
    node_marker.header.frame_id = world_frame_id;
    node_marker.header.stamp  = ros::Time::now();
    node_marker.ns = "nodes";
    node_marker.action = visualization_msgs::Marker::ADD;
    node_marker.pose.orientation.w = 1.0;
    node_marker.id = 0;
    node_marker.type = visualization_msgs::Marker::POINTS;
    node_marker.scale.x = scale/2;
    node_marker.scale.y = scale/2;
    node_marker.scale.z = scale/2;
//    node_marker.color.r = 1;
    node_marker.color.a = 0.8;


    // edge marker init

	edge_id=0;
	edge_marker.header.frame_id = world_frame_id;
    edge_marker.header.stamp  = ros::Time::now();
    edge_marker.ns = "edges";
    edge_marker.action = visualization_msgs::Marker::ADD;
    edge_marker.pose.orientation.w = 1.0;
    edge_marker.id = edge_id;
    edge_marker.type = visualization_msgs::Marker::ARROW;
    edge_marker.scale.x = 0.01;
    edge_marker.scale.y = 0.02;
    edge_marker.scale.z = 0.02;
    edge_marker.color.b = 0.8;
    edge_marker.color.g = 0.8;
    edge_marker.color.r = 0;
    edge_marker.color.a = 0.5;


    // flags
    state_callback_flag= false;
    state_callback_flag= false;


    // target history
    target_history.header.frame_id=world_frame_id;

    // octomap
    this->octree_obj=new octomap::OcTree(0.1);

	

    // azimuth, elevation set constructing
    azim_set.setLinSpaced(params.N_azim,0,2*PI);
    elev_set.setLinSpaced(params.N_elev,params.elev_min,params.elev_max);
}


void ASAP::graph_init() {
    // graph init
    g=Graph();
    
    Vertex x0 = boost::add_vertex(std::string("x0"), g);
    descriptor_map.insert(make_pair("x0",x0));

    Layer base_layer;
    base_layer.nodes.push_back(CandidNode("x0",cur_tracker_pos,-1));
    base_layer.t_idx=0; // initial index t0
    cur_layer_set.push_back(base_layer);

    // clear the markers
    node_marker.points.clear();
    arrow_array.markers.clear();
	edge_id=0;
}

// I think it can be optimized more
MatrixXd ASAP::castRay(geometry_msgs::Point rayStartPnt, float ray_length,bool verbose ) {


    // initialize the elements
    MatrixXd castResultBinary(params.N_elev,params.N_azim);
    castResultBinary.setConstant(-1);

    // castRay w.r.t sampling azimuth and elevation

    if (octree_obj->size()) {
        printf("casting started with light distance %.4f\n",ray_length);

        bool ignoreUnknownCells =true;
        octomap::point3d light_start(float(rayStartPnt.x),float(rayStartPnt.y),float(rayStartPnt.z));

        // generate mesh
        // ray casting & update castResult and castResultBinary
        for (unsigned int ind_elev = 0; ind_elev < params.N_elev; ind_elev++)
            for (unsigned int ind_azim = 0; ind_azim < params.N_azim; ind_azim++) {

                octomap::point3d light_end; //endpoint of ray whether the ray was hit
                octomap::point3d light_dir(float (cos(elev_set[ind_elev])*cos(azim_set[ind_azim])),
                                  float(cos(elev_set[ind_elev]) * sin(azim_set[ind_azim])),
                                  float(sin(elev_set[ind_elev])));

                // if hit
                if(octree_obj->castRay(light_start, light_dir, light_end,
                                       ignoreUnknownCells, ray_length))
                    castResultBinary.coeffRef(ind_elev, ind_azim) = 1;
                else
                    // no hit = just tracking distance
                    castResultBinary.coeffRef(ind_elev,ind_azim)=0;
            }

        // print the cast result
        if (verbose)
            std::cout<<castResultBinary<<std::endl;
    }// if octree is not empty
    else
      printf("Octree has zero size\n");


    return castResultBinary;
}


Layer ASAP::get_layer(geometry_msgs::Point light_source,int t_idx){

    Params & param=this->params;
    int local_key=0;

    Layer layer;
    layer.t_idx=t_idx;

    // casting ray
    for(vector<float>::iterator it = param.tracking_ds.begin(); it!=param.tracking_ds.end();++it)
    {
        float cur_d=*it;
        MatrixXd binaryCast=castRay(light_source,cur_d);
		MatrixXd sdf = SEDT(binaryCast); // signed distance field

		std::cout<<"==============================="<<std::endl;
		std::cout<<binaryCast<<std::endl;
		std::cout<<"-------------------------------"<<std::endl;
        std::cout<<sdf<<std::endl;
		std::cout<<"==============================="<<std::endl;
        // normalization should be performed
        mat_normalize(sdf); // sdf normalized
        bool negative_rejection=true; // we exclude negative local extrema  
        vector<IDX> extrema = localMaxima(sdf,params.N_extrem,params.local_range);

        ROS_INFO("found extrema: %d",extrema.size());

        // insert N_extrem many nodes
        for(vector<IDX>::iterator it_idx = extrema.begin();it_idx!=extrema.end();it_idx++)
        {

            IDX idx=*it_idx;
            ROS_INFO("extrema: [%d, %d]",idx[0],idx[1]);

            float cur_azim=azim_set[idx(1)];
            float cur_elev=elev_set[idx(0)];
            ROS_INFO("azim,elev: [%f, %f]",cur_azim,cur_elev);


            ViewVector viewVector;
            viewVector.azim=cur_azim; viewVector.elev=cur_elev; viewVector.ray_length=cur_d;
            viewVector.centerPnt=light_source;

            // node insertion
            CandidNode candidNode;
//            candidNode.id="d"+to_string(d_key)+"_"+to_string(local_key);
            candidNode.id="t"+to_string(t_idx)+"_"+to_string(local_key);
            candidNode.position=viewVector.getEndPnt();
            candidNode.visibility=sdf(idx[0],idx[1]);
            layer.nodes.push_back(candidNode);

            local_key++;
        }
    }
    return layer;
}


/**
 * Time index
 * base : t0
 * layer1: t1
 * layer2: t2 ...
 */

void ASAP::add_layer(Layer layer) {


    // no layer
    if (cur_layer_set.empty())
        printf("graph has not been initialized yet");

    // only base layer exits
    else if (cur_layer_set.size()==1) {
        ROS_INFO("current tracker position: [%f, %f, %f]\n",cur_tracker_pos.x,cur_tracker_pos.y,cur_tracker_pos.z);
        for (vector<CandidNode>::iterator it = layer.nodes.begin(); it != layer.nodes.end(); it++) {


            // problem may occur
            octomap::point3d P1((cur_tracker_pos.x), cur_tracker_pos.y, cur_tracker_pos.z);
            octomap::point3d P2(it->position.x, it->position.y, it->position.z);

            node_marker.points.push_back(it->position);
            std_msgs::ColorRGBA c;
            float red=(bgr[2]).at<uchar>(layer.t_idx-1)/255.0;
            float green=(bgr[1]).at<uchar>(layer.t_idx-1)/255.0;
            float blue=(bgr[0]).at<uchar>(layer.t_idx-1)/255.0;
            c.r=red; c.g=green; c.b=blue; c.a=0.7;
            node_marker.colors.push_back(c);


            // assign vertex name to this candid node
            VertexName name = (it->id); // time info + distance info + local order info
            // register to graph

            Vertex v = boost::add_vertex(name, g); // add vertex corresponding to current node to graph
            descriptor_map.insert(make_pair(name, v));


            // connect edge
            float dist = P1.distance(P2);

            Weight w;
            if(it->visibility==it->visibility)
                w=dist+params.w_v*it->visibility;
            else
                w=dist+params.w_v*1;

            if (dist < params.max_interval_distance+2){
                boost::add_edge(descriptor_map["x0"], v, w, g);
                edge_marker.points.clear();
                edge_marker.points.push_back(cur_tracker_pos);
                edge_marker.points.push_back(it->position);
                edge_marker.id=edge_id++;
				arrow_array.markers.push_back(visualization_msgs::Marker(edge_marker));

            }

        }
        printf("---------connecting complete--------\n");
        // insert this layer
        cur_layer_set.push_back(layer);
    }
    // we connect last layer stacked in graph
    else {

        // register to graph
        for(auto it = layer.nodes.begin(),end=layer.nodes.end();it != end;it++){
            // assign vertex name to this candidnode
            VertexName name=(it->id); // time info + distance info + local order info
            Vertex v = boost::add_vertex(name, g); // add vertex corresponding to current node to graph
            descriptor_map.insert(make_pair(name,v));

            // marker construct
            node_marker.points.push_back(it->position);
            std_msgs::ColorRGBA c;
            float red=(bgr[2]).at<uchar>(layer.t_idx-1)/255.0;
            float green=(bgr[1]).at<uchar>(layer.t_idx-1)/255.0;
            float blue=(bgr[0]).at<uchar>(layer.t_idx-1)/255.0;
            c.r=red; c.g=green; c.b=blue; c.a=0.7;
            node_marker.colors.push_back(c);


        }

        // connect layer with toppest layer
        Layer prev_layer= cur_layer_set.back();

        for(auto it1 = prev_layer.nodes.begin(),end1=prev_layer.nodes.end();it1 !=end1;it1++)
            for(auto it2 = layer.nodes.begin(),end2=layer.nodes.end();it2 != end2;it2++)
            {
                octomap::point3d P1(it1->position.x,it1->position.y,it1->position.z);
                octomap::point3d P2(it2->position.x,it2->position.y,it2->position.z);
                double dist=P1.distance(P2);
//                printf("node distance: %.4f\n",dist);
                if (dist<params.max_interval_distance){
                    Weight w;
                    if(it2->visibility==it2->visibility)
                        w=P1.distance(P2)+params.w_v*it2->visibility;
                    else
                        w=P1.distance(P2)+params.w_v*1;


                    boost::add_edge(descriptor_map[it1->id], descriptor_map[it2->id], w, g);
					edge_marker.points.clear();
					edge_marker.id=edge_id++;
					edge_marker.points.push_back(it1->position);
                    edge_marker.points.push_back(it2->position);
                    arrow_array.markers.push_back(visualization_msgs::Marker(edge_marker));

                }
            }

        printf("---------connecting complete--------\n");
        cur_layer_set.push_back(layer);

    }
}


void ASAP::graph_wrapping() {

    // register vertex to graph
    Vertex xf = boost::add_vertex(std::string("xf"), g);
    descriptor_map.insert(make_pair("xf",xf));

    // last layer append
    Layer finishing_layer;
    CandidNode dummy_node;
    dummy_node.id="xf";
    finishing_layer.nodes.push_back(dummy_node);
    ROS_INFO("layer copy constructor");
    // connect all the nodes in the last layer with dummy node having assigning weight
    Layer prev_layer= cur_layer_set.back();
    for(auto it = prev_layer.nodes.begin(),end=prev_layer.nodes.end();it != end;it++){
        Weight w=10; // just dummy constant
        boost::add_edge(descriptor_map[it->id],descriptor_map["xf"], w, g);
    }

    cur_layer_set.push_back(finishing_layer);

}

void ASAP::solve_view_path() {
    // find path using Dijkstra algorithm and save the Path into member functions
    GraphPath graphPath=Dijkstra(g,descriptor_map["x0"],descriptor_map["xf"]);
    this->view_path=nav_msgs::Path();
	this->view_path.header.frame_id=world_frame_id;
    std::cout<<"solved path received"<<std::endl;
    for(auto it = graphPath.begin(),end=graphPath.end();it != end;it++){
        VertexName id=*it;

        std::cout<<"this id:"<<id<<" ";

        if (id == "x0"){
            geometry_msgs::PoseStamped poseStamped;
            poseStamped.pose.position=cur_tracker_pos;
            view_path.poses.push_back(poseStamped);
        }else if(id =="xf"){
            // skip : no insertion
        }else{
            int t_idx=int(id[1])-'0';
            int local_idx=int(id[3])-'0'; // let's limit the N_node per layer
            geometry_msgs::PoseStamped poseStamped;
            poseStamped.pose.position=cur_layer_set[t_idx].nodes[local_idx].position;
            view_path.poses.push_back(poseStamped);
        }
    }
    std::cout<<std::endl;

}


void ASAP::state_callback(const gazebo_msgs::ModelStates::ConstPtr& gazebo_msg) {

    std::vector<std::string> model_names=gazebo_msg->name;
    std::vector<geometry_msgs::Pose> pose_vector=gazebo_msg->pose;


    int tracker_idx=std::find(model_names.begin(),model_names.end(),this->tracker_name)-model_names.begin();
//    int tracker_idx=std::find(model_names.begin(),model_names.end(),this->tracker_name)-model_names.begin();


    //extract target state
    if (tracker_idx<model_names.size()) {
        cur_tracker_pos = pose_vector[tracker_idx].position;
        state_callback_flag = true;
    }
    else
        ROS_WARN_ONCE("specified tracker name was not found in gazebo");

//    //update for visualize
//    BBMarker.pose=targetPose;
//    BBMarker.header.stamp=ros::Time::now();
//
//    //extract tracker state
//    if (tracker_idx<model_names.size())
//        trackerPose=pose_vector[tracker_idx];
//    else
//        ROS_WARN("specified tracker name was not found in gazebo");


}


void ASAP::points_callback(kiro_gui_msgs::PositionArray positionArray) {

    // receive the target history
    int N=0;
    target_prediction.poses.clear();
    for (auto it = positionArray.positions.begin(),end=positionArray.positions.end();it != end;it++)
    {
        it->pose.position.z=0.5;
        target_prediction.poses.push_back(*it); N++;}

    ROS_INFO("%d points received",N);


    cv::Mat cvVec(1,N,CV_8UC1);
    cv::Mat colorVec;

    for(int i=0;i<N;i++)
        cvVec.at<uchar>(i)=i*(255/float(N-1));

    applyColorMap(cvVec,colorVec,COLORMAP_JET);
    split(colorVec,bgr);


}


bool ASAP::solve_callback(asap::SolvePath::Request& req,asap::SolvePath::Response& rep) {

    // Building graph

    graph_init();
    int t_idx=1;
    ROS_INFO("size of prediction pnts: %d",target_prediction.poses.size());
    for (auto it = target_prediction.poses.begin(),end=target_prediction.poses.end();it != end;it++,t_idx++)
    {

        Layer layer=get_layer(it->pose.position,t_idx);
        printf("------------------------------\n");
        ROS_INFO("found layer: %dth predicition",t_idx);
        add_layer(layer);
    }

    graph_wrapping();
    ROS_INFO("finished graph");


    // graph inspection

    IndexMap index = get(boost::vertex_index, g);
    std::cout << "vertices(g) = ";
    typedef boost::graph_traits<Graph>::vertex_iterator vertex_iter;
    std::pair<vertex_iter, vertex_iter> vp;
    for (vp = vertices(g); vp.first != vp.second; ++vp.first) {
        Vertex v = *vp.first;
        std::cout << index[v] <<  " ";
    }
    std::cout << std::endl;

    std::cout << "edges(g) = ";
    boost::graph_traits<Graph>::edge_iterator ei, ei_end;
    for (boost::tie(ei, ei_end) = edges(g); ei != ei_end; ++ei)
        std::cout << "(" << index[source(*ei, g)]
                  << "," << index[target(*ei, g)] << ") ";
    std::cout << std::endl;


    std::cout << "number of node markers: "<<node_marker.points.size()<<std::endl;


    solve_view_path();
    ROS_INFO("Dijkstra solved");

    return true;
}

void ASAP::octomap_callback(const octomap_msgs::Octomap & msg) {


    octomap_callback_flag=true;
    octomap::AbstractOcTree* octree;

    octree=octomap_msgs::fullMsgToMap(msg);
    //octree update
    this->octree_obj=(dynamic_cast<octomap::OcTree*>(octree));


//
//    // free node around target
//    point3d light_start(targetPose.position.x,targetPose.position.y,targetPose.position.z);
//
//
//    double thresMin = octree_obj->getClampingThresMin();
//
//
//    for (OcTree::leaf_bbx_iterator it = octree_obj->begin_leafs_bbx(freebox_min_point + light_start,
//                                                                    freebox_max_point + light_start),
//                 end = octree_obj->end_leafs_bbx(); it != end; ++it)
//        it->setLogOdds(octomap::logodds(thresMin));
//
//    octree_obj->updateInnerOccupancy();

}

void ASAP::marker_publish() {

    // marker construction
    marker.points.clear();
    for (auto layer_it = cur_layer_set.begin(),layer_end=cur_layer_set.end();layer_it != layer_end;layer_it++)
        for (auto node_it = layer_it->nodes.begin(),node_end=layer_it->nodes.end();node_it != node_end;node_it++)
            marker.points.push_back(node_it->position);

    // marker publish
    candidNodes_marker_pub.publish(marker);

    // node publish
    if (node_marker.points.size())
        ROS_INFO_ONCE("size of nodes : %d",node_marker.points.size());
    node_pub.publish(node_marker);

    // edge publish
    edge_pub.publish(arrow_array);

}



void ASAP::points_publish() {

    // marker construction
    pnt_marker.points.clear();
    for (auto it = target_prediction.poses.begin(),end=target_prediction.poses.end();it != end;it++)
        pnt_marker.points.push_back(it->pose.position);

    // marker publish
    pnts_pub.publish(pnt_marker);

}


void ASAP::path_publish() {
    path_pub.publish(view_path);
}


ASAP::~ASAP() {
    delete octree_obj;
}


