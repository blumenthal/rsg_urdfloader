#ifndef RSG_URDFLOADER_H
#define RSG_URDFLOADER_H

#include <string>
#include <urdf_model/model.h>
#include <urdf/model.h>
#include <tinyxml.h>
#include <ros/console.h>

using namespace std;
//using namespace urdf;
namespace rsg_urdfloader{

class URDFtoRSG{

public:
bool rsgFromUrdfModel(const urdf::ModelInterface& robot_model)
{
  robot_model.getRoot()->name;
  //  add all children
  for (size_t i=0; i<robot_model.getRoot()->child_links.size(); i++)
   // if (!addChildrenToRSG(robot_model.getRoot()->child_links[i]))
      return false;

  return true;
};

private:
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


};

}

#endif
