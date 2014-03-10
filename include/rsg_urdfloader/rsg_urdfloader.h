#ifndef RSG_URDFLOADER_H
#define RSG_URDFLOADER_H

#include <string>
#include <urdf_model/model.h>
#include <urdf/model.h>
#include <tinyxml.h>
#include <ros/console.h>
#include <iostream>
#include <brics_3d/util/OSGPointCloudVisualizer.h>
#include <brics_3d/util/OSGTriangleMeshVisualizer.h>
#include <brics_3d/worldModel/sceneGraph/OSGVisualizer.h>
#include <brics_3d/core/PointCloud3D.h>
#include <brics_3d/core/Logger.h>
#include <brics_3d/core/HomogeneousMatrix44.h>
#include <brics_3d/core/TriangleMeshImplicit.h>
#include <brics_3d/core/TriangleMeshExplicit.h>
#include <brics_3d/algorithm/filtering/Octree.h>
#include <brics_3d/algorithm/filtering/BoxROIExtractor.h>
#include <brics_3d/algorithm/registration/IterativeClosestPointFactory.h>
#include <brics_3d/algorithm/meshGeneration/DelaunayTriangulationOSG.h>
#include <brics_3d/worldModel/WorldModel.h>
#include <brics_3d/worldModel/sceneGraph/Mesh.h>
#include <brics_3d/worldModel/sceneGraph/Box.h>
#include <brics_3d/worldModel/sceneGraph/Cylinder.h>
#include <brics_3d/worldModel/sceneGraph/DotGraphGenerator.h>
#include <assimp/Importer.hpp> 
#include <assimp/scene.h>
#include <assimp/postprocess.h> 

using namespace std;
using namespace urdf;
using namespace brics_3d;
using namespace brics_3d::rsg;
namespace rsg_urdfloader{

class URDFtoRSG{

public:
WorldModel *wm;

URDFtoRSG(){
	//With every loader, a new world model is created
	wm = new WorldModel;
};

~URDFtoRSG(){
	delete wm;

};

bool rsgFromUrdfModel(const urdf::ModelInterface& robot_model);


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

bool loadgeometry(const std::string& pFile){

	
	Assimp::Importer importer;
	const aiScene* scene = importer.ReadFile( pFile,
	aiProcess_CalcTangentSpace |
	aiProcess_Triangulate |
	aiProcess_JoinIdenticalVertices |
	aiProcess_SortByPType);
	if( !scene)
	{
		std::cerr << "Error Found :: " << importer.GetErrorString() << std::endl;
		return false;
	}
	
	//DoTheSceneProcessing( scene);
	return true;

}


bool visualize()
{
  return true;
}
;

private:

// recursive function to walk through tree
bool addChildrenToRSG(boost::shared_ptr<const urdf::Link> root, rsg::Id id);
vector<rsg::Attribute> addJoint(boost::shared_ptr<urdf::Joint> jnt);
vector<rsg::Attribute> addMassProperties(boost::shared_ptr<const urdf::Link> link);
bool addGeometry();




};
}

#endif
