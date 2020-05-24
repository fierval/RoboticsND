#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include "nav_msgs/Odometry.h"

#include <tuple>
#include <vector>
#include <string>
#include <cmath>

using namespace std;
typedef tuple<string, float, float, float> point_of_interest;
typedef vector<point_of_interest> pickup_dropoff_points;

class PickupDropoff
{
public:
  static const string pickup;
  static const string dropoff;

  PickupDropoff(pickup_dropoff_points &_poses) : poses(_poses)
  {
  }

  int init()
  {
    marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
    odom_sub = n.subscribe("odom", 5, &PickupDropoff::OdomCallback, this);
    
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

  void PublishPickup()
  {
    float x, y, w;
    tie(ignore, x, y, w) = poses[0];

    ROS_INFO("Setting pickup at x: %f, y: %f, w: %f", x, y, w);

    auto marker = FillMarker(x, y, w, visualization_msgs::Marker::ADD);
    marker_pub.publish(marker);
  }

private:
  // Pickup and dropoff locations
  pickup_dropoff_points poses;
  ros::NodeHandle n;
  ros::Publisher marker_pub;
  ros::Subscriber odom_sub;

  const float min_dist = 0.2;
  const float min_orient = 0.05;

  void OdomCallback(const nav_msgs::Odometry::ConstPtr &msg)
  {
    geometry_msgs::Point pos = msg->pose.pose.position;
    geometry_msgs::Quaternion orient = msg->pose.pose.orientation;

    for (auto poi : poses)
    {
      ManageMarker(poi, pos, orient);
    }
  }

  void ManageMarker(point_of_interest poi, geometry_msgs::Point pos, geometry_msgs::Quaternion orient)
  {
    string name;
    float x, y, w;
    tie(name, x, y, w) = poi;
    visualization_msgs::Marker marker;
    int32_t action;

    // we have reached the point of interest
    if (hypotf(pos.x - x, pos.y - y) <= min_dist && abs(w - orient.w) <= min_orient && abs(orient.x) <= min_orient && abs(orient.y) <= min_orient && abs(orient.z) <= min_orient)
    {
      if (name == pickup)
      {
        //if we are picking up - disappear the object
        action = visualization_msgs::Marker::DELETE;
      }
      else
      {
        action = visualization_msgs::Marker::ADD;
      }

      ROS_INFO_ONCE("Reached %s x: %f, y: %f, w: %f", name.c_str(), pos.x, pos.y, orient.w);
      marker = FillMarker(x, y, w, action);
      marker_pub.publish(marker);
    }
  }

  visualization_msgs::Marker FillMarker(float x, float y, float w, int32_t action)
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
    marker.pose.orientation.w = w;

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

const string PickupDropoff::pickup = "Pickup";
const string PickupDropoff::dropoff = "Dropoff";

int main(int argc, char **argv)
{
  pickup_dropoff_points poses{
      make_tuple(PickupDropoff::pickup, 3.0, 4.0, 1.0),
      make_tuple(PickupDropoff::dropoff, -5.0, 1.0, 1.0)};

  ros::init(argc, argv, "add_markers");
  PickupDropoff pickupDropoff(poses);

  if (pickupDropoff.init() != 0)
  {
    return 1;
  }

  pickupDropoff.PublishPickup();
  ros::spin();
  return 0;
}
