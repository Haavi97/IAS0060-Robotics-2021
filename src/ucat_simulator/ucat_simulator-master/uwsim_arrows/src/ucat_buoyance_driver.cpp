#include "ros/ros.h"
#include "uwsim_physics_msgs/ForceCmd.h"
#include "geometry_msgs/Vector3Stamped.h"


class BuoyanceDriver
{
  class Channel
  {
  public:
    size_t force_idx;
    std::string frame_id;
    double min_force, max_force, max_speed; //N/s, N/s, N/s^2

    geometry_msgs::Vector3Stamped::Ptr lastForce;
    geometry_msgs::Vector3Stamped::Ptr lastForceCmdSent;
    geometry_msgs::Vector3Stamped::Ptr lastForceCmd;

    ros::Publisher pub;
    ros::Subscriber sub;

    double timeout;

    Channel(ros::NodeHandle & n_):
      min_force(0), max_force(1.1), max_speed(0.1), timeout(2)
    {
      n_.getParam("min_force",min_force);
      n_.getParam("max_force",max_force);
      n_.getParam("max_speed",max_speed);
      n_.getParam("timeout",timeout);
    }

    void buoyanceCmdCallback(const geometry_msgs::Vector3Stamped::Ptr& msg)
    {
      lastForceCmd = msg;
    }

    void publish(uwsim_physics_msgs::ForceCmd & cmd)
    {
      if (lastForce && (cmd.header.stamp - lastForce->header.stamp).toSec()<timeout)
      {
        geometry_msgs::Vector3Stamped v;
        v.header.stamp = lastForce->header.stamp;
        v.header.frame_id = frame_id;
        v.vector = lastForce->vector;
        pub.publish(v);

        if (lastForceCmd)
        {
          if( lastForceCmd->vector.z < min_force)
            lastForceCmd->vector.z = min_force;
          else
            if( lastForceCmd->vector.z > max_force)
              lastForceCmd->vector.z = max_force;
          if (fabs(lastForceCmd->vector.z - lastForce->vector.z)<0.000001)
          {
            lastForceCmdSent.reset();
            lastForceCmd.reset();
          }
        }

        if (lastForceCmd)
        {
          if (!lastForceCmdSent)
            lastForceCmdSent = lastForce;

          double dT = (ros::Time::now() - lastForceCmdSent->header.stamp).toSec();
          if (dT<=0) dT = 0.001;
          double oldF = lastForceCmdSent->vector.z;
          double F = lastForceCmd->vector.z;
          double speed = (F - oldF)/dT;
          if (fabs(speed)>max_speed)
            speed = speed >= 0 ? max_speed : -max_speed;
          F = oldF + speed*dT;

          cmd.name.push_back(lastForce->header.frame_id);
          geometry_msgs::Vector3 v;
          v.z = F;
          cmd.force.push_back(v);
          cmd.duration.push_back(-1);

          lastForceCmdSent->header = cmd.header;
          lastForceCmdSent->vector = v;
        }
      }
      //else
      //  ROS_WARN("lastForce (%d) is null or ", (lastForce?1:0));
    }
  };

  double rate;

  ros::NodeHandle n,n_;

  ros::Subscriber buoyance_sub;
  ros::Publisher buoyance_cmd_pub;

  std::vector<boost::shared_ptr<Channel> > channels;

  void buoyanceCallback(const uwsim_physics_msgs::ForceCmdConstPtr& msg);

public:
  void run();
  BuoyanceDriver();
};

void BuoyanceDriver::buoyanceCallback(const uwsim_physics_msgs::ForceCmdConstPtr& msg)
{
  for (size_t ii = 0; ii<channels.size(); ++ii)
  {
    int force_idx = channels.at(ii)->force_idx;
    geometry_msgs::Vector3Stamped::Ptr lastForce = channels.at(ii)->lastForce;
    if (force_idx < msg->force.size())
    {
      lastForce.reset(new geometry_msgs::Vector3Stamped);
      lastForce->header = msg->header;
      lastForce->header.frame_id = force_idx < msg->name.size() ? msg->name.at(force_idx):"";
      lastForce->vector = msg->force.at(force_idx);
    }
    else
      lastForce.reset();

    channels.at(ii)->lastForce = lastForce;
  }
}

BuoyanceDriver::BuoyanceDriver():
        n_("~"), rate(10)
{
  n_.getParam("rate",rate);

  buoyance_sub = n.subscribe("buoyance_force", 1, &BuoyanceDriver::buoyanceCallback, this);
  buoyance_cmd_pub = n.advertise<uwsim_physics_msgs::ForceCmd>("buoyance_force_cmd", 1);

  std::vector<std::string> topics, frame_ids;
  std::vector<int> force_idxs;

  n_.getParam("topics", topics);
  n_.getParam("force_idxs", force_idxs);
  n_.getParam("frame_ids", frame_ids);

  for (size_t ii = 0; ii<topics.size(); ++ii)
  {
    std::string topic_pub = topics.at(ii);
    std::string topic_sub = topic_pub+std::string("_cmd");
    if (ii>=force_idxs.size())
      ROS_WARN("ii>=force_idxs.size()");
    int force_idx = force_idxs.at(ii);
    std::string frame_id = frame_ids.size()>ii?frame_ids.at(ii):"";

    boost::shared_ptr<Channel> c(new Channel(n_));
    c->pub = n.advertise<geometry_msgs::Vector3Stamped>(topic_pub, 1);
    c->sub = n.subscribe(topic_sub, 1, &Channel::buoyanceCmdCallback, c.get());
    c->frame_id = frame_id;
    c->force_idx = force_idx;
    channels.push_back(c);
  }
}

void BuoyanceDriver::run()
{
  ros::Rate loop_rate(rate);
  while (ros::ok())
  {
    ros::spinOnce();
    uwsim_physics_msgs::ForceCmd cmd;
    cmd.header.stamp = ros::Time::now();

    for (size_t ii = 0; ii<channels.size(); ++ii)
      channels.at(ii)->publish(cmd);

    buoyance_cmd_pub.publish(cmd);

    loop_rate.sleep();
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "buoyance_driver");

  boost::shared_ptr<BuoyanceDriver> buoyance_driver(new BuoyanceDriver);
  buoyance_driver->run();

  return 0;
}
