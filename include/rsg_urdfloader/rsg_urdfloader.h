#ifndef RSG_URDFLOADER_H
#define RSG_URDFLOADER_H

#include <string>
#include <urdf_model/model.h>
#include <urdf/model.h>
#include <tinyxml.h>
#include <ros/console.h>
#include <iostream>
using namespace std;
using namespace urdf;
namespace rsg_urdfloader{

class URDFtoRSG{

public:

URDFtoRSG(){
 
};

~URDFtoRSG(){};

bool rsgFromUrdfModel(const urdf::ModelInterface& robot_model)
{
  std::cout << "Found a robot with root :" << robot_model.getRoot()->name << std::endl;
  
  //  add all children
  for (size_t i=0; i<robot_model.getRoot()->child_links.size(); i++)
    if (!addChildrenToRSG(robot_model.getRoot()->child_links[i]))
      return false;

  return true;
};


bool rsgFromXml(TiXmlDocument *xml_doc)
{
  urdf::Model robot_model;
  if (!robot_model.initXml(xml_doc)){
    ROS_ERROR("Could not generate robot model");
    return false;
  }
  return rsgFromUrdfModel(robot_model);
};

bool rsgFromFile(const string& file)
{
  TiXmlDocument urdf_xml;
  urdf_xml.LoadFile(file);
  return rsgFromXml(&urdf_xml);
};


bool rsgFromParam(const string& param)
{
  urdf::Model robot_model;
  if (!robot_model.initParam(param)){
    ROS_ERROR("Could not generate robot model");
    return false;
  }
  return rsgFromUrdfModel(robot_model);
};

bool rsgFromString(const string& xml)
{
  TiXmlDocument urdf_xml;
  urdf_xml.Parse(xml.c_str());
  return rsgFromXml(&urdf_xml);
};

private:

// recursive function to walk through tree
bool addChildrenToRSG(boost::shared_ptr<const urdf::Link> root)
{
  std::vector<boost::shared_ptr<urdf::Link> > children = root->child_links;
  ROS_DEBUG("Link %s had %i children", root->name.c_str(), (int)children.size());

  // constructs the optional inertia
  /*RigidBodyInertia inert(0);
  if (root->inertial) 
    inert = toKdl(root->inertial);

  // constructs the kdl joint
  Joint jnt = toKdl(root->parent_joint);

  // construct the kdl segment
  Segment sgm(root->name, jnt, toKdl(root->parent_joint->parent_to_joint_origin_transform), inert);

  // add segment to tree
  tree.addSegment(sgm, root->parent_joint->parent_link_name);

  // recurslively add all children*/
  for (size_t i=0; i<children.size(); i++){
    if (!addChildrenToRSG(children[i]))
      return false;
  }
  return true;
}






};
}

#endif
