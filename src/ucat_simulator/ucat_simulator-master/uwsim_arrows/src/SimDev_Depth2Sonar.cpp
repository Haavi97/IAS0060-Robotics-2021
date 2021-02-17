//"DirectionalReceiver" example, SimulatedDevice_DirectionalReceiver.cpp

#include "uwsim_arrows/SimDev_Depth2Sonar.h"
#include "uwsim/SceneBuilder.h"
#include "sensor_msgs/Range.h"
#include <iostream>
#include <vector>
#include <vector>


std::vector<boost::shared_ptr<ROSInterface> > SimDev_Depth2Sonar_Factory::getInterface(
    ROSInterfaceInfo & rosInterface, std::vector<boost::shared_ptr<SimulatedIAUV> > & iauvFile)
{
  std::vector<boost::shared_ptr<ROSInterface> > ifaces;

  for (size_t i = 0; i < iauvFile.size(); ++i)
    for (size_t d = 0; d < iauvFile[i]->camview.size(); ++d)
      if (iauvFile[i]->camview[d].name == rosInterface.targetName)
      {
        VirtualCamera * cam = &(iauvFile[i]->camview[d]);

        std::string divisor = rosInterface.values["divisor"];
        std::string publishAll = rosInterface.values["publishAll"];
        if (publishAll.length()==0) publishAll = "0";
        ifaces.push_back(
            boost::shared_ptr<ROSInterface>(
                new SimDev_Depth2Sonar_ROSPublisher(cam, rosInterface.topic, rosInterface.rate, rosInterface.infoTopic,
                                                    boost::lexical_cast<int>(divisor),
                                                    rosInterface.values.at("frameId"),
                                                    boost::lexical_cast<int>(publishAll)!=0)
            )
        );
      }
  if (ifaces.size() == 0)
    ROS_WARN("Returning empty ROS interface for device %s...", rosInterface.targetName.c_str());

  return ifaces;
}

void SimDev_Depth2Sonar_ROSPublisher::createPublisher(ros::NodeHandle &nh)
{
  ROS_INFO("SimDev_Depth2Sonar_ROSPublisher on topic %s", topic.c_str());
  pub_ = nh.advertise<sensor_msgs::Range>(topic, 1);
  if (infoTopic.length()>0)
    image_pub = nh.advertise<sensor_msgs::Image>(infoTopic, 1);

}

void SimDev_Depth2Sonar_ROSPublisher::publish()
{
  sensor_msgs::Range msg;
  msg.header.stamp = getROSTime();
  if (dev != NULL && dev->depthTexture!=NULL)
  {
    osg::ref_ptr < osg::Image > osgimage;
    osgimage = dev->depthTexture;
    msg.header.frame_id = this->frameId;
    int w, h, d, step;
    w = osgimage->s();
    h = osgimage->t();
    d = osgimage->getTotalSizeInBytes();
    step = d/h;
    double fov_y, aspect, near, far;
    dev->textureCamera->getProjectionMatrixAsPerspective(fov_y, aspect, near, far);
    double fov_x = fov_y * aspect;
    double fov = sqrt(pow(fov_x,2)+pow(fov_y,2));
    //ROS_INFO("fov_y=%.4f, aspect=%.4f, fov_x=%.4f, fov=%.4f",fov_y, aspect, fov_x, fov);
    double a = far / (far - near);
    double b = (far * near) / (near - far);
    msg.max_range = far;
    msg.min_range = near;
    msg.field_of_view = fov*M_PI/180.0;
    msg.range = msg.max_range;

    int w_, h_, d_, step_;
    w_= w/mult;
    h_= h/mult;
    d_=w_*h_;
    step_ = w_ * sizeof(float);

    if (publishAll && pub_all.size()==0)
      for(int ii=0;ii<h_; ++ii)
        for(int jj=0;jj<w_; ++jj)
        {
          std::stringstream ss;
          ss<<topic<<"_"<<jj<<"_"<<ii;
          pub_all.push_back(this->nh_.advertise<sensor_msgs::Range>(ss.str(), 1));
        }

    std::vector<float> minimized;
    minimized.resize(d_, 0);
    std::vector<int> minimizedCnt;
    minimizedCnt.resize(d_, 0);

    for(int r=0; r < h;++r){
          float *data = (float *)(osgimage->data()+r*step);
          for(int c=0; c < w;++c){
            float Z = (data[c]);
            float D = b / (Z - a);
            //converting depth to range
            double x = (c - dev->cx) * D / dev->fx;
            double y = (r - dev->cy) * D / dev->fy;
            D = sqrt(pow(x,2)+pow(y,2)+pow(D,2));
//            if (D<=near)
//              D = -INFINITY;
//            else if (D>=far)
//              D = INFINITY;
            if (isfinite(D))
            {
              int r_ = (r/mult);
              int c_ = (c/mult);
              int i = (h_-r_-1)*w_ + c_;
              minimized[i]+=D;
              minimizedCnt[i]++;
            }
          }
    }
    bool img = (infoTopic.length()>0);
    sensor_msgs::Image m;
    if (img)
    {
      m.header.stamp = msg.header.stamp;
      m.header.frame_id = msg.header.frame_id;
      m.width = w_;
      m.height = h_;
      m.step = step_;
      m.encoding = std::string("32FC1");
      m.data.resize(d_*sizeof(float));
    }

    float D = 0;
    std::vector<float> cells;
    for(int i=0; i < d_;++i)
    {
      D = minimizedCnt[i]>0? minimized[i]/minimizedCnt[i]:NAN;
      if (isfinite(D) && D<msg.range)
      {
        msg.range = D;
      }
      if (publishAll)
        cells.push_back(D);
      if (img)
        ((float*)(m.data.data()))[i] = D;
    }

    if (img)
      image_pub.publish(m);

    if (publishAll)
      for(int ii=0;ii<h_; ++ii)
        for(int jj=0;jj<w_; ++jj)
        {
          std::stringstream ss;
          ss<<msg.header.frame_id<<"_"<<jj<<"_"<<ii;
          sensor_msgs::Range msg_;
          msg_.header.stamp = msg.header.stamp;
          msg_.header.frame_id = ss.str();
          msg_.max_range = far;
          msg_.min_range = msg.min_range;
          msg_.field_of_view = msg.field_of_view/sqrt(w_*w_+h_*h_);
          //msg_.range = msg_.max_range;
          int idx = w_*ii+jj;
          msg_.range = cells.at(idx);
          pub_all.at(idx).publish(msg_);
          //pub_all.push_back(nh.advertise<sensor_msgs::Range>(ss.str(), 1));
        }
  }
  pub_.publish(msg);
}
