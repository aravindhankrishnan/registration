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
#include <fstream>
#include <string>
#include <strstream>
using namespace std;

#include <boost/program_options.hpp>
#include <boost/tokenizer.hpp>

#include "pcl/common/common_headers.h"
#include "pcl/io/pcd_io.h"

string inputfile;
string outputfile;
string separator;
double xoffset, yoffset;

int parse_arguments (int argc, char *argv[])
{
	namespace po = boost::program_options;

	po::options_description desc ("Allowed options");
	desc.add_options ()
		("help,h", "./texttopcd -i inputfile -s \",\" -o outputfile -x xoffset_val -y yoffset_val\n"
		           "The input files should be separated by spaces..\n")
		("input-file,i", po::value<string> (&inputfile), "input ascii file (with x y z values only separated by commas)")
    ("separator,s", po::value <string> (&separator), "field separator in the input file. for e.g. comma, space etc")
		("output-file,o", po::value<string> (&outputfile), "output pcd file (with a .pcd extension)")
		("xoffset,x", po::value<double> (&xoffset), "x offset value, this is subtracted from the x coordinate of the input data") 
		("yoffset,y", po::value<double> (&yoffset), "y offset value, this is subtracted from the y cooordinate of the input data");

	po::variables_map vm;
	po::store (po::parse_command_line (argc, argv, desc), vm);
	po::notify (vm);

	if (vm.count ("help")) {
		cout << desc << "\n";
		return -1; 
	}
  if (!vm.count ("input-file")) {
    cout << "Please enter the input file..\n";
		return -1; 
  }
  else {
    cout << "Input file = " << inputfile << endl;
  }
  if (!vm.count ("output-file")) {
    cout << "Please enter the output file..\n";
		return -1; 
  }
  else {
    cout << "Output file = " << outputfile << endl;
  }
  if (!vm.count ("separator")) {
    cout << "Please enter the field separator..\n";
		return -1; 
  }
  else {
    cout << "Field separator (enclosed within angled braces) = <" << separator << ">..\n";
  }
	if (!vm.count ("xoffset")) {
		cout << "Please enter x offset parameter" << endl;
		return -1;
	}
  else {
    cout << "X offset = " << double (xoffset) << endl;
  }
	if (!vm.count ("yoffset")) {
		cout << "Please enter y offset parameter" << endl;
		return -1;
	}
  else {
    cout << "Y offset = " << double (yoffset) << endl;
  }
	return 0;
}

int main(int argc, char *argv[])
{
  cout.setf (ios::fixed);
	if (parse_arguments (argc, argv) < 0) {
		return -1;
	}

  cout << "Opening file " << inputfile << endl;
	ifstream ifile (inputfile.c_str ());
	if (!ifile) {
		cout << "File " << inputfile << " could not be opened..\n";
		return -1;
	}
  else {
    cout << "Opened file " << inputfile << endl;
  }

	ofstream ofile (outputfile.c_str ());
	if (!ofile) {
		cout << "Output file cannot be opened.. Check the path and directory permissions..\n";
		return -1;
	}
	else {
		remove (outputfile.c_str ());
	}

  double data[3] = {0.0};
	double x = 0.0, y = 0.0, z = 0.0;
	
	pcl::PointCloud<pcl::PointXYZ>::Ptr basic_cloud_ptr (new pcl::PointCloud<pcl::PointXYZ>);

  int count = 0;
	while (ifile) {
		string buffer = "";
		getline(ifile, buffer);
		if (buffer.length () == 0) {
      break;
    }
    boost::char_separator <char> sep (separator.c_str ());
    boost::tokenizer <boost::char_separator <char> > tokens (buffer, sep);

    short i = 0;
    for (boost::tokenizer <boost::char_separator <char> >::iterator itr = tokens.begin ();
        itr != tokens.end ();
        itr++) {
      string t = *itr;
      double val = boost::lexical_cast <double> (t.c_str ());
      data[i++] = val;
    }
    x = data[0];
    y = data[1];
    z = data[2];

		pcl::PointXYZ basic_point;
		basic_point.x = double(x) - xoffset;
		basic_point.y = double(y) - yoffset;
		basic_point.z = z;
		basic_cloud_ptr->points.push_back (basic_point);
	}
	
	basic_cloud_ptr->width = 1;
	basic_cloud_ptr->height = basic_cloud_ptr->points.size ();
  cout << "Saving file " << outputfile << endl;
	pcl::io::savePCDFileASCII (outputfile, *basic_cloud_ptr);
	return 0;
}
