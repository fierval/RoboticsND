#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <tuple>
#include <vector>
#include <string>

using namespace std;

visualization_msgs::Marker FillMarker(float x, float y, float w)
{

  visualization_msgs::Marker marker;
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
  return marker;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "add_markers");
  ros::NodeHandle n;
  ros::Rate r(5);
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
  // Set our initial shape type to be a cube

  uint32_t shape = visualization_msgs::Marker::CUBE;
  vector<tuple<string, float, float, float>> poses {
    make_tuple("Pickup", 3.0, 4.0, 1.0), 
    make_tuple("Dropoff", -5.0, 1.0, 1.0)
    };

  visualization_msgs::Marker marker;
  marker.type = shape;

  // Publish the marker
  while (marker_pub.getNumSubscribers() < 1)
  {
    if (!ros::ok())
    {
      return 0;
    }
    ROS_WARN_ONCE("Please create a subscriber to the marker");
    sleep(1);
  }

  for (auto pos : poses)
  {
    string name;
    float x, y, w;
    tie(name, x, y, w) = pos;

    ROS_INFO("Publishing %s at (%f, %f), orientation: (0, 0, %f)", name.c_str(), x, y, w);

    marker = FillMarker(x, y, w);
    marker_pub.publish(marker);

    ROS_INFO("Waiting 5 sec...");
    ros::Duration(5.0).sleep();
    marker.action = visualization_msgs::Marker::DELETE;
    marker_pub.publish(marker);
  }
}
