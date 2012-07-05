//    Copyright (C) 2012  ASTRIL, ASU (http://robotics.asu.edu)
//
//    This program is free software: you can redistribute it and/or modify
//    it under the terms of the GNU Affero General Public License as
//    published by the Free Software Foundation, either version 3 of the
//    License, or (at your option) any later version.
//
//    This program is distributed in the hope that it will be useful,
//    but WITHOUT ANY WARRANTY; without even the implied warranty of
//    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//    GNU Affero General Public License for more details.
//
//    You should have received a copy of the GNU Affero General Public License
//    along with this program.  If not, see <http://www.gnu.org/licenses/>
//
//    Author - Aravindhan K Krishnan (aravindhan.krishnan@asu.edu)
//

#include <iostream>
#include <string>
#include <fstream>
using namespace std;

#include <boost/program_options.hpp>
#include <boost/timer.hpp>
#include <boost/lexical_cast.hpp>
using boost::lexical_cast;
using boost::bad_lexical_cast;

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/transforms.h>
#include <pcl/common/centroid.h>
#include <pcl/common/transforms.h>

string inputfile;
string outputfile;
string basepath;
int total_partitions;
float x, y;

int parse_arguments (int argc, char *argv[])
{
  namespace po = boost::program_options;
  po::options_description desc ("Allowed options");
  desc.add_options ()
    ("help,h", "./get_displacements -i inputfile -o outputfile -b x/window- -x 0 -y 0")
    ("input-file,i", po::value<string> (&inputfile), "input file (log file obtained by running registration-ed program)")
    ("output-file,o", po::value<string> (&outputfile), "output file")
    ("base-path,b", po::value<string> (&basepath), "base path for the partitioned cloud (e.g) x/window-")
    ("xoffset,x", po::value<float> (&x), "x offset")
    ("yoffset,y", po::value<float> (&y), "y offset");

  po::variables_map vm;
  po::store (po::parse_command_line (argc, argv, desc), vm);
  po::notify (vm);

  if (vm.count ("help")) {
    cout << desc << "\n";
    return -1; 
  }
  if (!vm.count ("input-file")) {
    cout << "Please enter an input flie..\n";
    return -1;
  }
  if (!vm.count ("output-file")) {
    cout << "Please enter an output file..\n";
    return -1;
  }
  if (!vm.count ("base-path")) {
    cout << "Please enter the base path for the paritioned cloud..\n";
    return -1;
  }
  if (!vm.count ("xoffset")) {
    cout << "Please enter the x offset.\n";
    return -1;
  }
  if (!vm.count ("yoffset")) {
    cout << "Please enter the y offset.\n";
    return -1;
  }
  return 0;
}
void find_yaw_pitch_roll (Eigen::Matrix4f& transform, float& yaw, float& pitch, float& roll)
{
  float N = 0.0;
  float D = 0.0;

  N = transform (1, 0);
  D = transform (0, 0);
  yaw = atan2 (N, D);

  N = -transform (2, 0);
  D = sqrt (transform (2, 1)*transform (2, 1) + transform (2, 2)*transform (2, 2));
  pitch = atan2 (N, D);

  N = transform (2, 1);
  D = transform (2, 2);
  roll = atan2 (N, D);
}
int main (int argc, char *argv[])
{
  if (parse_arguments (argc, argv) == -1) {
    return -1;
  }

  ifstream ifile (inputfile.c_str ());
  if (!ifile) {
    cout << "Couldn't open the input file " << inputfile << "..\n";
    return -1;
  }
  else {
    cout << "Opened file " << inputfile << endl;
  }

  ofstream ofile (outputfile.c_str ());
  if (!ofile) {
    cout << "Couldn't open the output file.. Please check the path..\n";
    return -1;
  }

  int i = 0;
  char buffer[4096];
  fill_n(buffer, 4096, '\0');
  int partition = 1;
  int k = 1;

  while (ifile) {
    ifile.getline (buffer, 4096);
    string line = buffer;
    size_t pos = 0;
    if( (pos = line.find ("Transformation for partition")) != string::npos) {
      cout << "String match..\n";
      pos += strlen ("Transformation for partition");
      size_t pos1 = line.find_first_of ("0123456789", pos);
      size_t pos2 = line.find_first_not_of ("0123456789", pos1);
      int N = boost::lexical_cast <int> (line.substr (pos1, pos2-pos1).c_str ());
      cout << buffer << "\t(" << N << ")\t\n";

      ostringstream ostr;
      ostr << basepath << N << ".pcd";

      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

      if (pcl::io::loadPCDFile<pcl::PointXYZ> (ostr.str (), *cloud) == -1) {
        cout << "Couldn't read file " << ostr.str () << endl;
        return -1;
      }
      cout << "Loaded file - " << ostr.str () << endl;


      Eigen::Vector4f centroid;
      compute3DCentroid (*cloud, centroid);
      centroid (3) = 1;

      cout << "Centroid = " << centroid (0) << " " << centroid (1) << " " << centroid (2) << " " << centroid (3) << endl;

      Eigen::Matrix4f transform;
      vector<float> T (16);
      int z = 0;

      try {
        for (int k=0; k<4; k++) {
          fill_n (buffer, 4096, '\0');
          ifile.getline (buffer, 4096);
          istringstream istr (buffer);
          float val = 0.0;
          for (int p=0; p<4; p++) {
            istr >> val;
            T[z++] = val;
          }
        }
      }
      catch (boost::bad_lexical_cast &e) {
        cout << "Bad lexical cast..\n";
        exit (0);
      } 

      transform <<  T[0], T[1], T[2], T[3],
                T[4], T[5], T[6], T[7],
                T[8], T[9], T[10], T[11],
                T[12], T[13], T[14], T[15];

      cout << transform << endl;
      float yaw = 0.0, pitch = 0.0, roll = 0.0;
      find_yaw_pitch_roll (transform, yaw, pitch, roll);

      Eigen::Vector4f newcentroid = transform*centroid;
      cout << "centroid \n" << centroid << endl;
      cout << "New centroid \n" << newcentroid << endl;

      Eigen::Vector4f translation = newcentroid - centroid;

      ofile << x+newcentroid (0) << "\t" << y+newcentroid (1) << "\t" << 
        yaw*180/M_PI << "\t" << pitch*180/M_PI << "\t" << roll*180/M_PI << "\t" << 
        translation (0) << "\t" << translation (1) << "\t" << translation (2) << endl;
    }
    i++;
  }

  ofile.close ();
  ifile.close ();
  return 0;
}
