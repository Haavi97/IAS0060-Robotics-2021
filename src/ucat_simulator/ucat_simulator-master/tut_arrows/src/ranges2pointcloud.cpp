#include <ros/ros.h>
#include <sensor_msgs/Range.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_listener.h>
#include <pcl_ros/point_cloud.h>

class Ranges2Pointcloud2
{
public:
  Ranges2Pointcloud2();
  void run();

private:
  std::vector<std::string> range_topics;
  std::vector<double> range_wh_ratios;
  std::vector<int> range_dots_per_heights;
  std::vector<bool> range_ovals;
  std::string frame_id;
  double rate, min_range, max_range;

  void rangeCallback(const sensor_msgs::Range::ConstPtr& data);
  void publishPointcloud2();

  ros::NodeHandle nh_,nh__;

  ros::Publisher points_pub_;
  std::vector<ros::Subscriber> ranges_subs_;

  tf::TransformListener listener;

  //ros::Time lastPublishedStamp;
  typedef std::vector<sensor_msgs::Range::ConstPtr> Ranges;
  typedef std::map<std::string, Ranges> RangesMap;
  typedef pcl::PointXYZ Point;
  typedef pcl::PointCloud<Point> PointCloud;
  RangesMap rangesMap;
};

Ranges2Pointcloud2::Ranges2Pointcloud2():
        nh_(), nh__("~"),
        frame_id("base_link"), rate(5), min_range(0.3), max_range(10)
{
  nh__.param("range_topics", range_topics, range_topics);
  nh__.param("range_wh_ratios", range_wh_ratios, range_wh_ratios);
  nh__.param("range_dots_per_heights", range_dots_per_heights, range_dots_per_heights);
  nh__.param("range_ovals", range_ovals, range_ovals);

  nh__.param("frame_id", frame_id, frame_id);
  nh__.param("rate", rate, rate);
  nh__.param("min_range", min_range, min_range);
  nh__.param("max_range", max_range, max_range);

  points_pub_ = nh_.advertise<PointCloud>("range_points", 1);

  for (int ii = 0; ii < range_topics.size(); ++ii) {
    if (range_wh_ratios.size()<=ii)
      range_wh_ratios.push_back(1.5);
    if (range_dots_per_heights.size()<=ii)
      range_dots_per_heights.push_back(9);
    if (range_ovals.size()<=ii)
      range_ovals.push_back(true);
    ROS_INFO("Subscribing to Range topic '%s'", range_topics.at(ii).c_str());
    ranges_subs_.push_back(nh_.subscribe(range_topics.at(ii), 10, &Ranges2Pointcloud2::rangeCallback, this));
  }
}

void Ranges2Pointcloud2::run()
{
  ros::Rate r(rate);
  while (ros::ok())
  {
    ros::spinOnce();

    publishPointcloud2();

    r.sleep();
  }
}

void Ranges2Pointcloud2::publishPointcloud2()
{
  ros::Time maxTime;
  std::vector<sensor_msgs::Range::ConstPtr> lastRanges;

  for (RangesMap::iterator iter = rangesMap.begin(); iter != rangesMap.end(); iter++) {
    sensor_msgs::Range::ConstPtr r =iter->second.back();
    lastRanges.push_back(r);
    if (r->header.stamp>maxTime)
      maxTime = r->header.stamp;
  }

  PointCloud::Ptr msg(new PointCloud);
  msg->header.frame_id = frame_id;
  msg->header.stamp = maxTime.toNSec();
  msg->height = 1;
  for (size_t ii = 0; ii < lastRanges.size(); ++ii) {
    if ((maxTime-lastRanges[ii]->header.stamp).toSec() < 1.5/rate
        && lastRanges[ii]->range >= min_range
        && lastRanges[ii]->range <= max_range
        )
    {
      try{
        tf::StampedTransform transform;
        listener.lookupTransform(frame_id, lastRanges[ii]->header.frame_id, ros::Time(0), transform);

        double fov_diag = lastRanges[ii]->field_of_view*0.5;
        double ratio = range_wh_ratios.at(ii);
        double fov_h = sqrt(fov_diag*fov_diag/(ratio*ratio+1.0));
        double fov_w = fov_h * ratio;
        //diag^2 = (ratio^2+1)*height^2
        //diag^2/(ratio^2+1) = height^2
        //height = sqrt(diag^2/(ratio^2+1))
        int height = range_dots_per_heights.at(ii);
        int width = height * ratio;
        height--;
        width--;
        double height_base = -height/2.0;
        double width_base = -width/2.0;
        double range = lastRanges[ii]->range;

        for(int w = 0; w<=width; ++w)
          for(int h = 0; h<=height; ++h)
          {
            double z = (2.0*(height_base+h)/height);
            double y = (2.0*(width_base+w)/width);

            if (range_ovals.at(ii)==0 || (z*z+y*y)<=1.05)
            {
              z*=fov_h;
              y*=fov_w;
              tf::Vector3 p_in(
                  cos(z)*cos(y)*range,
                  sin(y)*range,
                  sin(z)*range
                  );
              tf::Vector3 p_out = transform * p_in;
              msg->points.push_back(Point(p_out.x(), p_out.y(), p_out.z()));
            }
          }
      }
      catch (tf::TransformException ex){
        ROS_ERROR("%s",ex.what());
      }
    }
  }
  msg->width = msg->points.size();

  rangesMap.clear();

  if (msg->points.size()>0)
  {
    points_pub_.publish(msg);
  }
}

void Ranges2Pointcloud2::rangeCallback(const sensor_msgs::Range::ConstPtr& data)
{
  rangesMap[data->header.frame_id].push_back(data);
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "ranges2pointcloud");
  Ranges2Pointcloud2 ranges2Pointcloud;
  ranges2Pointcloud.run();
}
