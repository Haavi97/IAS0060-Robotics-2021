//"DirectionalReceiver" example, SimulatedDevice_DirectionalReceiver.cpp

#include "uwsim_arrows/SimDev_DirectionalReceiver.h"
#include <osg/MatrixTransform>
#include <osg/PositionAttitudeTransform>
#include "uwsim/SceneBuilder.h"
#include <iostream>

SimDev_DirectionalReceiver::SimDev_DirectionalReceiver(SimDev_DirectionalReceiver_Config * cfg, osg::Node * root,
                                                       osg::Node * trackNode) :
    SimulatedDevice(cfg)
{
  //make this virtual ray track the node
  node_tracker = new IntersectorUpdateCallback(cfg->range, cfg->angle, cfg->visible, root, cfg->numLines);
  node_tracker->sensitiveTo = cfg->sensitiveTo;
  trackNode->setUpdateCallback(node_tracker);
  trackNode->asGroup()->addChild(node_tracker->geode);
  trackNode->asGroup()->addChild(node_tracker->coneGeode);
}

SimulatedDeviceConfig::Ptr SimDev_DirectionalReceiver_Factory::processConfig(const xmlpp::Node* node,
                                                                             ConfigFile * config)
{
  SimDev_DirectionalReceiver_Config * cfg = new SimDev_DirectionalReceiver_Config(getType());
  xmlpp::Node::NodeList list = node->get_children();
  double tmp3[3];
  double tmp;
  int tmpint;
  cfg->numLines = 4;
  unsigned int tmpuint;
  std::string tmpstring;
  for (xmlpp::Node::NodeList::iterator iter = list.begin(); iter != list.end(); ++iter)
  {
    const xmlpp::Node* child = dynamic_cast<const xmlpp::Node*>(*iter);
    if (child->get_name() == "relativeTo")
      config->extractStringChar(child, cfg->relativeTo);
    else if (child->get_name() == "sensitiveTo")
    {
      config->extractStringChar(child, tmpstring);
      boost::trim(tmpstring);
      cfg->sensitiveTo.push_back(tmpstring);
    }
    else if (child->get_name() == "range")
    {
      config->extractFloatChar(child, tmp);
      cfg->range = tmp;
    }
    else if (child->get_name() == "angle")
    {
      config->extractFloatChar(child, tmp);
      cfg->angle = tmp;
    }
    else if (child->get_name() == "visible")
    {
      config->extractIntChar(child, tmpint);
      cfg->visible = tmpint;
    }
    else if (child->get_name() == "numberOfLines")
    {
      config->extractUIntChar(child, tmpuint);
      cfg->numLines = tmpuint;
    }
    else if (child->get_name() == "position")
    {
      config->extractPositionOrColor(child, tmp3);
      for (int i = 0; i < 3; i++)
        cfg->position[i] = tmp3[i];
    }
    else if (child->get_name() == "orientation")
    {
      config->extractOrientation(child, tmp3);
      for (int i = 0; i < 3; i++)
        cfg->orientation[i] = tmp3[i];
    }
  }
  return SimulatedDeviceConfig::Ptr(cfg);
}

bool SimDev_DirectionalReceiver_Factory::applyConfig(SimulatedIAUV * auv, Vehicle &vehicleChars,
                                                     SceneBuilder *sceneBuilder, size_t iteration)
{
  if (iteration > 0)
    return true;
  // find all DirectionalReceivers
  for (size_t i = 0; i < vehicleChars.simulated_devices.size(); ++i)
    if (vehicleChars.simulated_devices[i]->getType() == this->getType())
    {
      // Get the configuration from XML
      SimDev_DirectionalReceiver_Config * cfg =
          dynamic_cast<SimDev_DirectionalReceiver_Config *>(vehicleChars.simulated_devices[i].get());
      // Find parent node
      int nodeIndex = -1;
      for (int j = 0; j < vehicleChars.nlinks && nodeIndex < 0; j++)
        if (vehicleChars.links[j].name == cfg->relativeTo)
          nodeIndex = j;
      // Add transform according to position and orientation
      if (nodeIndex >= 0 && cfg)
      {
        osg::ref_ptr<osg::Transform> vMr = (osg::Transform*)new osg::PositionAttitudeTransform;
        vMr->asPositionAttitudeTransform()->setPosition(
            osg::Vec3d(cfg->position[0], cfg->position[1], cfg->position[2]));
        vMr->asPositionAttitudeTransform()->setAttitude(
            osg::Quat(cfg->orientation[2], osg::Vec3d(0, 0, 1), cfg->orientation[1], osg::Vec3d(0, 1, 0),
                      cfg->orientation[0], osg::Vec3d(1, 0, 0)));
        auv->urdf->link[nodeIndex]->asGroup()->addChild(vMr);
        // Create new device from the configuration
//				SimDev_DirectionalReceiver::Ptr device(
//						new SimDev_DirectionalReceiver(cfg,
//								sceneBuilder->scene->localizedWorld, vMr));
//				auv->devices->all.push_back(device);
        auv->devices->all.push_back(
            SimDev_DirectionalReceiver::Ptr(
                new SimDev_DirectionalReceiver(cfg, sceneBuilder->scene->localizedWorld, vMr)));
      }
      else
      OSG_FATAL<< "SimDev_DirectionalReceiver device '"
      << vehicleChars.simulated_devices[i]->name
      << "' inside robot '" << vehicleChars.name
      << "' has failed, discarding..." << std::endl;
    }
  return true;
}

std::vector<boost::shared_ptr<ROSInterface> > SimDev_DirectionalReceiver_Factory::getInterface(
    ROSInterfaceInfo & rosInterface, std::vector<boost::shared_ptr<SimulatedIAUV> > & iauvFile)
{
  std::vector<boost::shared_ptr<ROSInterface> > ifaces;
  for (size_t i = 0; i < iauvFile.size(); ++i)
    for (size_t d = 0; d < iauvFile[i]->devices->all.size(); ++d)
      if (iauvFile[i]->devices->all[d]->getType() == this->getType()
          && iauvFile[i]->devices->all[d]->name == rosInterface.targetName)
      {
        ifaces.push_back(
            boost::shared_ptr<ROSInterface>(
                new SimDev_DirectionalReceiver_ROSPublisher(
                    dynamic_cast<SimDev_DirectionalReceiver*>(iauvFile[i]->devices->all[d].get()), rosInterface.topic,
                    rosInterface.rate)));
        //rosInterface.values are for new and non-standard xml configurations, but it looks like currently existing rosInterface fields are enough...
        //below is just an example how to get values, alternatively you can use rosInterface.values["name"]
        //for(std::map<std::string,std::string>::iterator it = rosInterface.values.begin() ;it != rosInterface.values.end();++it)
        //	ROS_INFO("rosInterface.values[%s]='%s'",  it->first.c_str(), it->second.c_str());
      }
  if (ifaces.size() == 0)
    ROS_WARN("Returning empty ROS interface for device %s...", rosInterface.targetName.c_str());
  return ifaces;
}

void SimDev_DirectionalReceiver_ROSPublisher::createPublisher(ros::NodeHandle &nh)
{
  ROS_INFO("SimDev_DirectionalReceiver_ROSPublisher on topic %s", topic.c_str());
  pub_ = nh.advertise<std_msgs::Bool>(topic, 10);
}

void SimDev_DirectionalReceiver_ROSPublisher::publish()
{
  std_msgs::Bool msg;
  if (dev != NULL)
    msg.data = dev->node_tracker->transmitterVisible;
  pub_.publish(msg);
}

void SimDev_DirectionalReceiver::IntersectorUpdateCallback::operator()(osg::Node *node, osg::NodeVisitor *nv)
{
  traverse(node, nv);
  osg::Matrixd receiverPose = osg::computeLocalToWorld(nv->getNodePath());

  osg::Vec3 position = receiverPose.getTrans();
  osg::Matrixd orientation(receiverPose.getRotate());

  osg::Vec3d end = position + orientation * osg::Vec3d(range, 0, 0);

  osg::Polytope sensor_cone;
  osg::Plane plane;
  osg::Vec3d normal;

  double half_angle = angle / 2;

  // side planes, starting with right (z is down), which is parallel to z axis
  const int numOfCorners = min(max(int(numLines), 3), 10000);
  for (int i = 0; i < numOfCorners; i++)
  {
    osg::Quat rotation = osg::Quat(i * osg::DegreesToRadians(360.0f / numOfCorners), osg::Vec3d(1, 0, 0));
    normal.set(sin(half_angle), cos(half_angle), 0);
    normal = rotation * normal;
    normal = orientation * normal;
    plane.set(normal, position);
    sensor_cone.add(plane);
  }

  //Far plane
  normal.set(-1, 0, 0);
  normal = orientation * normal;
  plane.set(normal, end);
  sensor_cone.add(plane);

  osg::ref_ptr<osgUtil::PolytopeIntersector> intersector = new osgUtil::PolytopeIntersector(sensor_cone);

  osg::ref_ptr<osgUtil::IntersectionVisitor> visitor = new osgUtil::IntersectionVisitor;
  visitor->setIntersector(intersector);

  // Accept node at root of scene.
  root->accept(*visitor);

  transmitterVisible = false;

  // Look for intersections with objects in the sensitiveTo list
  if (intersector->containsIntersections())
  {
    osgUtil::PolytopeIntersector::Intersections intersections = intersector->getIntersections();
    for (osgUtil::PolytopeIntersector::Intersections::iterator lII = intersections.begin();
        lII != intersections.end() && !transmitterVisible; lII++)
    {
      for (osg::NodePath::const_iterator n = lII->nodePath.begin(); n != lII->nodePath.end(); n++)
      {
        if (std::find(sensitiveTo.begin(), sensitiveTo.end(), (*n)->getName()) != sensitiveTo.end())
        {
          transmitterVisible = true;
          break;
        }
      }
    }
  }
}

SimDev_DirectionalReceiver::IntersectorUpdateCallback::IntersectorUpdateCallback(double range, double angle,
                                                                                 bool visible, osg::Node *root,
                                                                                 unsigned int lines)
{
  this->range = range;
  this->angle = angle;
  this->numLines = lines;
  this->transmitterVisible = false;
  this->root = root;
  intersector = new osgUtil::LineSegmentIntersector(osg::Vec3d(0, 0, 0), osg::Vec3d(0, 0, 0));
  intersectVisitor.setIntersector(intersector.get());

  if (visible)
  {
    geode = osg::ref_ptr<osg::Geode>(new osg::Geode());
    const int numOfCorners = min(max(int(numLines), 3), 10000);
    for (int i = 0; i < numOfCorners; i++)
    {
      osg::Quat rotation = osg::Quat(i * osg::DegreesToRadians(360.0f / numOfCorners), osg::Vec3d(1, 0, 0));
      beam = osg::ref_ptr<osg::Geometry>(new osg::Geometry);
      osg::ref_ptr<osg::Vec3Array> points = new osg::Vec3Array;
      osg::Vec3d start(0, 0, 0);
      osg::Vec3d end(range, range * sin(angle / 2),
                     range * sin(angle / 2) * tan(osg::DegreesToRadians(360.0f / (2 * numOfCorners))));
      points->push_back(start);
      points->push_back(rotation * end);
      osg::ref_ptr<osg::Vec4Array> color = new osg::Vec4Array;
      color->push_back(osg::Vec4(0.0, 1.0, 0.0, 0.6));
      beam->setVertexArray(points.get());
      beam->setColorArray(color.get());
      beam->setColorBinding(osg::Geometry::BIND_PER_PRIMITIVE_SET);
      beam->addPrimitiveSet(new osg::DrawArrays(GL_LINES, 0, 2));
      geode->addDrawable(beam.get());
    }
  }
}
