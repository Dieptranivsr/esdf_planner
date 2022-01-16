#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
// using namespace message_filters;

class Node
{
public:
    Node()
    {
        sub_1_.subscribe(nh_, "/camera/depth/image_rect_raw", 1);           //1
        sub_2_.subscribe(nh_, "/camera/color/image_raw", 1);
        sub_6_.subscribe(nh_, "/camera/depth/image_raw", 1);
        sub_3_.subscribe(nh_, "/mavros/local_position/pose", 1);            //2
        sub_4_.subscribe(nh_, "/mavros/local_position/odom", 1);
        sub_5_.subscribe(nh_, "/camera/pose", 1);

        sync_1_.reset(new Sync1(SyncPolicyImage(10), sub_1_, sub_2_));
        sync_1_->registerCallback(boost::bind(&Node::Imgcallback, this, _1, _2));

        sync_2_.reset(new Sync2(SyncPolicyImagePose(10), sub_1_, sub_3_));
        sync_2_->registerCallback(boost::bind(&Node::Posecallback, this, _1, _2));

        sync_3_.reset(new Sync3(SyncPolicyDepthPose(10), sub_2_, sub_3_));
        sync_3_->registerCallback(boost::bind(&Node::Depthcallback, this, _1, _2));

        sync_4_.reset(new Sync4(SyncPolicyDepthOdom(10), sub_2_, sub_4_));
        sync_4_->registerCallback(boost::bind(&Node::Odomcallback, this, _1, _2));

        sync_5_.reset(new Sync5(SyncPolicyPoseOdom(10), sub_3_, sub_4_));
        sync_5_->registerCallback(boost::bind(&Node::PoseOdomcallback, this, _1, _2));
/*
        sync_6_.reset(new Sync6(SyncPolicyCamPose(10), sub_6_, sub_5_));
        sync_6_->registerCallback(boost::bind(&Node::CamPosecallback, this, _1, _2));
*/
        //sync_7_.reset(new Sync7(SyncPolicyCamPose2(10), sub_6_, sub_1_));
        //sync_7_->registerCallback(boost::bind(&Node::CamPosecallback2, this, _1, _2));
    }

    void Imgcallback(const sensor_msgs::ImageConstPtr &in1, const sensor_msgs::ImageConstPtr &in2)
    {
        ROS_INFO("--------------------------------------------------------");
        ROS_INFO(" Img - Img || Synchronization successful");
    }
    
    void Posecallback(const sensor_msgs::ImageConstPtr &in1, const geometry_msgs::PoseStampedConstPtr &in2)
    {
        ROS_INFO(" Depth - Pose || Synchronization successful");
    }

    void Depthcallback(const sensor_msgs::ImageConstPtr &in1, const geometry_msgs::PoseStampedConstPtr &in2)
    {
        ROS_INFO(" Img - Pose || Synchronization successful");
    }

    void Odomcallback(const sensor_msgs::ImageConstPtr &in1, const nav_msgs::OdometryConstPtr &in2)
    {
        ROS_INFO(" Img - Odom || Synchronization successful");
    }

    void PoseOdomcallback(const geometry_msgs::PoseStampedConstPtr &in1, const nav_msgs::OdometryConstPtr &in2)
    {
        ROS_INFO(" Pose - Odom || Synchronization successful");
    }

    /*
    void CamPosecallback(const sensor_msgs::ImageConstPtr &in1, const geometry_msgs::PoseStampedConstPtr &in2)
    {
        std::cout << "|-------------------------------------------|" << std::endl;
        ROS_INFO(" Img - Camera Pose || Synchronization successful");
        std::cout << "depth: " << in1->header.frame_id << std::endl;
        std::cout << "camera pose: " << in2->header.frame_id << std::endl;
    }

    void CamPosecallback2(const sensor_msgs::ImageConstPtr &in1, const geometry_msgs::PoseStampedConstPtr &in2)
    {
        std::cout << "|------22222222222222222222222222222222--------|" << std::endl;
        ROS_INFO(" Img - Camera Pose || Synchronization successful");
        std::cout << "depth: " << in1->header.frame_id << std::endl;
        std::cout << "camera pose: " << in2->header.frame_id << std::endl;
    }*/

private:
    ros::NodeHandle nh_;
    message_filters::Subscriber<sensor_msgs::Image> sub_1_;
    message_filters::Subscriber<sensor_msgs::Image> sub_2_;
    message_filters::Subscriber<geometry_msgs::PoseStamped> sub_3_;
    message_filters::Subscriber<nav_msgs::Odometry> sub_4_;
    message_filters::Subscriber<geometry_msgs::PoseStamped> sub_5_;
    message_filters::Subscriber<sensor_msgs::Image> sub_6_;

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> SyncPolicyImage;
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, geometry_msgs::PoseStamped> SyncPolicyImagePose;
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, geometry_msgs::PoseStamped> SyncPolicyDepthPose;
    // nav_msgs::Odometry
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, nav_msgs::Odometry> SyncPolicyDepthOdom;
    // geometry_msgs::PoseStamped
    typedef message_filters::sync_policies::ApproximateTime<geometry_msgs::PoseStamped, nav_msgs::Odometry> SyncPolicyPoseOdom;
    //typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, geometry_msgs::PoseStamped> SyncPolicyCamPose;
    //typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image, geometry_msgs::PoseStamped> SyncPolicyCamPose2;

    typedef message_filters::Synchronizer<SyncPolicyImage> Sync1;
    typedef message_filters::Synchronizer<SyncPolicyImagePose> Sync2;
    typedef message_filters::Synchronizer<SyncPolicyDepthPose> Sync3;
    typedef message_filters::Synchronizer<SyncPolicyDepthOdom> Sync4;
    typedef message_filters::Synchronizer<SyncPolicyPoseOdom> Sync5;
    //typedef message_filters::Synchronizer<SyncPolicyCamPose> Sync6;
    //typedef message_filters::Synchronizer<SyncPolicyCamPose2> Sync7;
    boost::shared_ptr<Sync1> sync_1_;
    boost::shared_ptr<Sync2> sync_2_;
    boost::shared_ptr<Sync3> sync_3_;
    boost::shared_ptr<Sync4> sync_4_;
    boost::shared_ptr<Sync5> sync_5_;
    //boost::shared_ptr<Sync6> sync_6_;
    //boost::shared_ptr<Sync7> sync_7_;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "synchronizer");

    Node synchronizer;

    ros::spin();
}