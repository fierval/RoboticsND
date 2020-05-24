#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include "pick_objects/PointReached.h"

#include <tuple>
#include <vector>
#include <string>
#include <cmath>

using namespace std;

class PickupDropoff
{
public:
  static const string Pickup;
  static const string Dropoff;
  static const string Set;

  PickupDropoff()
  {
  }

  int init()
  {
    marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
    pickup_sub = n.subscribe<pick_objects::PointReached>("goal_state", 5, &PickupDropoff::GoalStateCallback, this);

    while (marker_pub.getNumSubscribers() < 1)
    {
      if (!ros::ok())
      {
        return 1;
      }
      ROS_WARN_ONCE("Please create a subscriber to the marker");
      sleep(1);
    }

    return 0;
  }

private:

  ros::NodeHandle n;
  ros::Publisher marker_pub;
  ros::Subscriber pickup_sub;

  const float min_dist = 0.2;

  void GoalStateCallback(const pick_objects::PointReached::ConstPtr &msg)
  {
    int32_t action;
    visualization_msgs::Marker marker;

    ROS_INFO("Reached: %s", msg->name.c_str());
    if(msg->name == "set")
    {
      action = visualization_msgs::Marker::ADD;
    }
    if(msg->name == "pickup")
    {
      action = visualization_msgs::Marker::DELETEALL;
    }
    else if (msg->name == "dropoff")
    {
      action = visualization_msgs::Marker::ADD;
    }
    else 
    {
      ROS_ERROR("Unknonw goal");
      action = visualization_msgs::Marker::DELETEALL;
    }

    marker = FillMarker(msg->x, msg->y, action);
    marker_pub.publish(marker);
  }

  visualization_msgs::Marker FillMarker(float x, float y, int32_t action)
  {
    visualization_msgs::Marker marker;

    marker.type = visualization_msgs::Marker::CUBE;
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker.ns = "add_markers";
    marker.id = 0;

    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    marker.action = visualization_msgs::Marker::ADD;

    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    marker.pose.position.x = x;
    marker.pose.position.y = y;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = 0.3;
    marker.scale.y = 0.3;
    marker.scale.z = 0.3;

    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.r = 0.0f;
    marker.color.g = 0.0f;
    marker.color.b = 1.0f;
    marker.color.a = 1.0;

    marker.lifetime = ros::Duration();
    marker.action = action;
    return marker;
  }
};

const string PickupDropoff::Pickup = "pickup";
const string PickupDropoff::Dropoff = "dropoff";
const string PickupDropoff::Set = "set";

int main(int argc, char **argv)
{
  ros::init(argc, argv, "add_markers");
  PickupDropoff pickupDropoff;

  if (pickupDropoff.init() != 0)
  {
    return 1;
  }

  ros::spin();
  return 0;
}
