#include "ASAP.h"

int main(int argc,char **argv){

    ros::init(argc,argv,"solving_path_test_node");
    // parameter parsing
    ros::NodeHandle nh_private("~");
    asap_ns::Params params;

    params.azim_min=0; params.azim_max=2*PI;

    float tracking_d_min,tracking_d_max;
    int N_tracking_d;


    std::string tracker_name,target_name;

    nh_private.getParam("tracking_d_min", tracking_d_min);
    nh_private.getParam("tracking_d_max", tracking_d_max);
    nh_private.getParam("tracking_d_N",N_tracking_d);

    params.set_ds(tracking_d_min,tracking_d_max,N_tracking_d);

    nh_private.getParam("elev_min", params.elev_min);
    nh_private.getParam("elev_max", params.elev_max);

    nh_private.getParam("N_azim", params.N_azim);
    nh_private.getParam("N_elev",params.N_elev);
    nh_private.getParam("local_range",params.local_range);
    nh_private.getParam("N_extrema",params.N_extrem);

    nh_private.getParam("max_interval_distance",params.max_interval_distance);
    nh_private.getParam("w_v",params.w_v);
    nh_private.getParam("tracker_name",tracker_name);
    nh_private.getParam("target_name",target_name);




    printf("Parameters summary: \n");
    printf("------------------------------\n");
    printf("------------------------------\n");

    printf("sampling distance: [ ");
    for(int i=0;i<params.tracking_ds.size();i++)
        printf("%f ",params.tracking_ds[i]);
    printf(" ]\n");

    printf("azimuth range: [%f, %f] N: %d / elevation: [%f, %f] N: %d \n",params.azim_min,params.azim_max,
    params.N_azim,params.elev_min,params.elev_max,params.N_elev);

    printf("local maxima search in visibility matrix : search window = %d / N_extrema = %d\n",params.local_range,params.N_extrem);
    printf("max interval disance: %f\n",params.max_interval_distance);
    printf("weight for visibility in edge connection: %f \n",params.w_v);



    ASAP asap_obj(params);

    asap_obj.tracker_name=tracker_name;
    asap_obj.target_name=target_name;

    ROS_INFO("Always See and Picturing started");
    ros::Rate rate(20);

    // initialize
    ros::Duration(1.0).sleep();

    // main loop
    while(ros::ok()){

        if (asap_obj.octomap_callback_flag && asap_obj.state_callback_flag)
        {
            asap_obj.path_publish();
            asap_obj.marker_publish();
            asap_obj.points_publish();
        }

        ros::spinOnce();
        rate.sleep();
    }

    return 0;

}