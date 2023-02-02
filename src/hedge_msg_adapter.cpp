#include "ros/ros.h"
#include "marvelmind_nav/hedge_pos_ang.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "tf/transform_broadcaster.h"

class hedge_msg_adapter_node
{
public:
  hedge_msg_adapter_node() // Class constructorf
  {
    ros::NodeHandle nh_; // Public nodehandle for pub-sub
    ros::NodeHandle nh_private_("~"); // Private nodehandle for handling parameters

    // Init subscribers
    
    pos_ang_sub_ = nh_.subscribe("hedge_pos_ang", 10, &hedge_msg_adapter_node::pos_ang_callback, this);

    // Init publishers
    hedge_pose_pub_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("hedge_pose", 10, false);

    // You must provide the static transforms for these in a launch file!
    pose_out_.header.frame_id = "map";

    // Init covariances grabbed from the parameter server
    init_covariances(nh_private_);
  }

  

  void pos_ang_callback(const marvelmind_nav::hedge_pos_ang::ConstPtr& pos_ang_msg)
  {
    // Populate header
    pose_out_.header.stamp = ros::Time::now();

    // Populate position data
    pose_out_.pose.pose.position.x = pos_ang_msg->x_m;
    pose_out_.pose.pose.position.y = pos_ang_msg->y_m;
    pose_out_.pose.pose.position.z = pos_ang_msg->z_m;

    // Populate orientation data
    pose_out_.pose.pose.orientation = tf::createQuaternionMsgFromYaw(pos_ang_msg->angle);

    // Publish the geometry_msgs/PoseWithCovarianceStamped message
    hedge_pose_pub_.publish(pose_out_);
  }

  // Handy function for initialising covariance matrices from parameters
  void init_covariances(ros::NodeHandle &nh_private_)
  {
    // Create the vectors to store the covariance matrix arrays
    std::vector<double> pose_covar;

    // Grab the parameters and populate the vectors
    nh_private_.getParam("pose_covariance", pose_covar);

    // Iterate through each vector and populate the respective message fields
    for (int i = 0; i < pose_covar.size(); i++)
      pose_out_.pose.covariance[i] = pose_covar.at(i);
  }

protected:
  // Subscriber objects

  ros::Subscriber pos_ang_sub_;

  // Publisher objects
  ros::Publisher hedge_pose_pub_;


  // Message objects

  geometry_msgs::PoseWithCovarianceStamped pose_out_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "hedge_msg_adapter");

  hedge_msg_adapter_node adapter;

  ros::spin();
}
