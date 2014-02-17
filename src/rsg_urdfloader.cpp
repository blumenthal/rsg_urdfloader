#include "rsg_urdfloader/rsg_urdfloader.h"
#include <urdf/model.h>
#include <iostream>

using namespace std;
using namespace urdf;
using namespace rsg_urdfloader;


bool URDFtoRSG::rsgFromUrdfModel(const urdf::ModelInterface& robot_model)
{
  std::cout << "Found a robot with root :" << robot_model.getRoot()->name << std::endl;
  //  add all children
  for (size_t i=0; i<robot_model.getRoot()->child_links.size(); i++)
    if (!addChildrenToRSG(robot_model.getRoot()->child_links[i]))
      return false;

  return true;
}


// recursive function to walk through tree
bool URDFtoRSG::addChildrenToRSG(boost::shared_ptr<const urdf::Link> root)
{
  std::vector<boost::shared_ptr<urdf::Link> > children = root->child_links;
  ROS_DEBUG("Link %s had %i children", root->name.c_str(), (int)children.size());
  vector<rsg::Attribute> tmpAttributes;
  tmpAttributes.clear();
  // constructs the optional inertia
  /*RigidBodyInertia inert(0);
  if (root->inertial) 
    inert = toKdl(root->inertial);*/
  tmpAttributes.clear();
  //Collect atributes of the joint
  tmpAttributes = addJoint(root->parent_joint);
  /*tmpAttributes.clear();
  // construct the kdl segment
  Segment sgm(root->name, jnt, toKdl(root->parent_joint->parent_to_joint_origin_transform), inert);

  // add segment to tree
  tree.addSegment(sgm, root->parent_joint->parent_link_name);*/

  // recurslively add all children*/
  for (size_t i=0; i<children.size(); i++){
    if (!addChildrenToRSG(children[i]))
      return false;
  }
  return true;
}

// Need to add jnt->axis
vector<rsg::Attribute> URDFtoRSG::addJoint(boost::shared_ptr<urdf::Joint> jnt)
{
  vector<rsg::Attribute> tmpAttributes;
  tmpAttributes.clear();
  switch (jnt->type){
  case urdf::Joint::FIXED:{
    tmpAttributes.push_back(Attribute("type","FIXED"));
    return tmpAttributes;
  }
  case urdf::Joint::REVOLUTE:{
    tmpAttributes.push_back(Attribute("type","REVOLUTE"));
    return tmpAttributes;
  }
  case urdf::Joint::CONTINUOUS:{
    tmpAttributes.push_back(Attribute("type","CONTINUOUS"));
    return tmpAttributes;
  }
  case urdf::Joint::PRISMATIC:{
    tmpAttributes.push_back(Attribute("type","PRISMATIC"));
    return tmpAttributes;
  }
  default:{
    ROS_WARN("Converting unknown joint type of joint '%s' into a fixed joint", jnt->name.c_str());
    tmpAttributes.push_back(Attribute("type","FIXED"));
    return tmpAttributes;
  }
  }
  //empty attributes
  return tmpAttributes;

}





int main(int argc, char** argv)
{
  URDFtoRSG *urdftorsg;
  urdftorsg = new URDFtoRSG();
  if (argc < 2){
    std::cerr << "Expect xml file to parse" << std::endl;
    return -1;
  }

  WorldModel* wm = new WorldModel();
  

  Model robot_model;
  if (!robot_model.initFile(argv[1]))
  {cerr << "Could not generate robot model" << endl; return false;}

  if (!urdftorsg->rsgFromUrdfModel(robot_model)) 
   {cerr << "Could not Load the Robot Scene Graph" << endl; return false;}

 
}
