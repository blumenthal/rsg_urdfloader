#include "rsg_urdfloader/rsg_urdfloader.h"
#include <urdf/model.h>
#include <iostream>

using namespace std;
using namespace urdf;
using namespace rsg_urdfloader;

int main(int argc, char** argv)
{
  URDFtoRSG *urdftorsg;
  urdftorsg = new URDFtoRSG();
  if (argc < 2){
    std::cerr << "Expect xml file to parse" << std::endl;
    return -1;
  }
  Model robot_model;
  if (!robot_model.initFile(argv[1]))
  {cerr << "Could not generate robot model" << endl; return false;}

  if (!urdftorsg->rsgFromUrdfModel(robot_model)) 
   {cerr << "Could not Load the Robot Scene Graph" << endl; return false;}

  /*// walk through tree
  cout << " ======================================" << endl;
  cout << " Tree has " << my_tree.getNrOfSegments() << " link(s) and a root link" << endl;
  cout << " ======================================" << endl;
  SegmentMap::const_iterator root = my_tree.getRootSegment();
  printLink(root, "");*/
}
