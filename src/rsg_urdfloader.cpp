#include "rsg_urdfloader/rsg_urdfloader.h"
#include <urdf/model.h>
#include <iostream>

using namespace std;
using namespace urdf;

int main(int argc, char** argv)
{
  if (argc < 2){
    std::cerr << "Expect xml file to parse" << std::endl;
    return -1;
  }
  Model robot_model;
  if (!robot_model.initFile(argv[1]))
  {cerr << "Could not generate robot model" << endl; return false;}

  //Tree my_tree;
  //if (!kdl_parser::treeFromUrdfModel(robot_model, my_tree)) 
  //{cerr << "Could not extract kdl tree" << endl; return false;}

  /*// walk through tree
  cout << " ======================================" << endl;
  cout << " Tree has " << my_tree.getNrOfSegments() << " link(s) and a root link" << endl;
  cout << " ======================================" << endl;
  SegmentMap::const_iterator root = my_tree.getRootSegment();
  printLink(root, "");*/
}




