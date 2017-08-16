#include "eis.h"
#include <cv_bridge/cv_bridge.h>
#include <iostream>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>

ros::Publisher image_pub;

bool is_show      = true;
bool is_eis_on    = true;
bool is_first_run = true;

Eis* m_eis;
Eigen::Quaterniond q_ec_tgt;
Eigen::Quaterniond q_bC;

void
imageProcessCallback( const sensor_msgs::ImageConstPtr& image_msg, const sensor_msgs::ImuConstPtr& imu_msg )
{
    cv::Mat image_in = cv_bridge::toCvCopy( image_msg, "mono8" )->image;

    Eigen::Quaterniond q_eb( imu_msg->orientation.w, imu_msg->orientation.x,
                             imu_msg->orientation.y, imu_msg->orientation.z );

    if ( is_first_run )
    {
        // q_eC is the initialized q_ec
        Eigen::Quaterniond q_ec_init = q_eb * q_bC;
        q_ec_tgt                     = q_ec_init;
        is_first_run                 = false;
    }
    else
    {
        Eigen::Quaterniond q_Cc;

        cv::Mat image_eis = m_eis->process( image_in, q_Cc );

        cv_bridge::CvImage outImage;
        outImage.header.stamp    = image_msg->header.stamp;
        outImage.header.frame_id = "eis";
        outImage.encoding        = "monon8";
        outImage.image           = image_eis;
        image_pub.publish( outImage );

        if ( is_show )
        {
            cv::imshow( "src", image_in );
            cv::imshow( "src", image_eis );
        }
    }
}

int
main( int argc, char** argv )
{
    ros::init( argc, argv, "eis_node" );
    ros::NodeHandle n( "~" );

    std::string camera_file;
    double angle_row;
    double angle_col;
    int max_size;

    n.getParam( "camera_file", camera_file );
    n.getParam( "angle_row", angle_row );
    n.getParam( "angle_col", angle_col );
    n.getParam( "max_size", max_size );
    n.getParam( "is_show", is_show );

    if ( max_size <= 0 )
    {
        std::cout << "#[ERROR] Error with output image Size." << std::endl;
        return 0;
    }

    m_eis = new Eis( camera_file, angle_row, angle_col, max_size );

    image_pub = n.advertise< sensor_msgs::Image >( "/eis_imega", 1 );

    message_filters::Subscriber< sensor_msgs::Image > sub_img( n, "/left_image", 2 );
    message_filters::Subscriber< sensor_msgs::Imu > sub_imu( n, "/imu", 2 );

    typedef message_filters::sync_policies::ExactTime< sensor_msgs::Image, sensor_msgs::Imu > SyncPolicy;
    //    typedef message_filters::sync_policies::ApproximateTime< sensor_msgs::Image,
    //    sensor_msgs::Imu > SyncPolicy;
    message_filters::Synchronizer< SyncPolicy > sync( SyncPolicy( 3 ), sub_img, sub_imu );

    sync.registerCallback( boost::bind( &imageProcessCallback, _1, _2 ) );

    if ( is_show )
    {
        cv::namedWindow( "src", CV_WINDOW_NORMAL );
        cv::namedWindow( "eis", CV_WINDOW_NORMAL );
    }

    while ( ros::ok( ) )
    {
        ros::spinOnce( );
    }

    std::cout << "[#INFO] EIS shut down." << std::endl;
    return 0;
}
