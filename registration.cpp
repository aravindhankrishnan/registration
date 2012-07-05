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
//    along with this program.  If not, see <http://www.gnu.org/licenses/>.
//
//    Author - Aravindhan K Krishnan (aravindhan.krishnan@asu.edu)
//

#include <iostream>
#include <vector>
using namespace std;

#include <boost/program_options.hpp>
#include <boost/timer.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/transformation_estimation_point_to_plane_lls.h>

string f1, f2;
float ransac_par = 0.5;
float correspondence_threshold;
int nx, ny;
double dx, dy, wx, wy;
double transformation_epsilon;

//#define EIGEN_DONT_ALIGN_STATICALLY

int parse_arguments (int argc, char *argv[]) 
{
	namespace po = boost::program_options;

	vector <string> inputfiles;

	po::options_description desc ("Allowed options");
	desc.add_options ()
		("help,h", "./regisration -i predata -i postdata -t val -e val --wx val --wy val --dx val --dy val | tee log.txt ")
		("input-files,i", po::value< vector < string > > (&inputfiles), "input file")
		("correspondence-threshold,t", po::value<float> (&correspondence_threshold), "Correspondence threshold (2.7 works)")
		("transformation-epsilon,e", po::value<double> (&transformation_epsilon), "Termination threshold for ICP (0.001 works)")
    ("wx", po::value <double> (&wx), "window size x (for e.g. --wx 100)")
    ("wy", po::value <double> (&wy), "window size y (for e.g. --wy 100)")
    ("dx", po::value <double> (&dx), "search space x (for e.g. --dx 10)")
    ("dy", po::value <double> (&dy), "search space y (for e.g. --dy 10)");

	po::variables_map vm;
	po::store (po::parse_command_line (argc, argv, desc), vm);
	po::notify (vm);

	if (vm.count ("help")) {
		cout << desc << "\n";
		return -1; 
	}
	if (vm.count ("input-files")) {
		f1 = inputfiles[0];
		f2 = inputfiles[1];

		cout << f1 << "\t" << f2 << endl;
	}
	else {
		cout << "Enter input files as argument..\n";
		if (inputfiles.size () != 2){
			cout << "Enter 2 input files..\n";
		}
		return -1;
	}
	if (!vm.count ("correspondence-threshold")) {
		cout << "Enter a correspondence thresold parameter as an argument..\n";
		return -1;
	}
  else {
    cout << "Correspondence rejection threshold = " << correspondence_threshold << endl;
  }
	if (!vm.count ("transformation-epsilon")) {
		cout << "Enter a transformation epsilon for as an argument..\n";
		return -1;
	}
  else {
    cout << "Transformation epsilon = " << transformation_epsilon << endl;
  }

  if (vm.count ("wx")) {
    cout << "Window size in x direction = " << wx << endl;
  }
  else {
    cout << "Enter the window size in x direction\n";
    return -1;
  }
  if (vm.count ("wy")) {
    cout << "Window size in y direction = " << wy << endl;
  }
  else {
    cout << "Enter the window size in y direction\n";
    return -1;
  }
  if (vm.count ("dx")) {
    cout << "Search space in x direction = " << dx << endl;
  }
  else {
    cout << "Enter the search space in x direction\n";
    return -1;
  }
  if (vm.count ("dy")) {
    cout << "Search space in x direction = " << dy << endl;
  }
  else {
    cout << "Enter the search space in y direction\n";
    return -1;
  }
	return 0;
}
int find_bin_number (double val, double min_val, double max_val, int n_bins)
{
	double bin_size = (max_val - min_val) / n_bins;

	int k=0;
	for (float i=min_val; i<= max_val; i+=bin_size,k++) {
		if (val >= i and val <= (i+bin_size)) {
			return k;
    }
	}
}
void find_min_max (pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, float& minx, 
                   float& maxx, float& miny, float& maxy)
{
	minx = cloud->points[0].x;
	maxx = cloud->points[0].x;
	miny = cloud->points[0].y;
	maxy = cloud->points[0].y;

	size_t N = cloud->points.size ();

	for (size_t i=0; i<N; i++) {
		if (cloud->points[i].x > maxx) {
			maxx = cloud->points[i].x;
		}
		if (cloud->points[i].x < minx) {
			minx = cloud->points[i].x;
		}
		if (cloud->points[i].y > maxy) {
			maxy = cloud->points[i].y;
		}
		if (cloud->points[i].y < miny) {
			miny = cloud->points[i].y;
		}
	}
}
void partition_point_cloud (pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, 
		                        vector < vector <pcl::PointCloud<pcl::PointXYZ>::Ptr> >& cloud_partitions,
                            float dx, float dy, int& nx, int& ny) 
{
	float minx, maxx, miny, maxy;

	find_min_max (cloud, minx, maxx, miny, maxy);

	cout << "minx = " << minx << " maxx = " << maxx << " miny = " << miny << " maxy = " << maxy << endl;
	nx = ceil ((maxx-minx) / dx);
	ny = ceil ((maxy-miny) / dy);
	cout << "nx = " << nx << " ny = " << ny << endl;

	cloud_partitions.reserve (nx);
	for (int i=0; i<=nx; i++) {
		cloud_partitions.push_back (vector <pcl::PointCloud<pcl::PointXYZ>::Ptr> ());
	}

	for (int i=0; i<=nx; i++) {
		vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> &vec = cloud_partitions[i];
		for (int j=0; j<=ny; j++) {
			pcl::PointCloud<pcl::PointXYZ>::Ptr C(new pcl::PointCloud<pcl::PointXYZ>);
			vec.push_back (C);
		}
	}

	size_t N = cloud->points.size ();
	for (size_t i=0; i<N; i++) {
		size_t ix = 0, iy = 0;
		float x = cloud->points[i].x;
		float y = cloud->points[i].y;
		ix = int (floor((x-minx)/dx));
		iy = int (floor((y-miny)/dy));

		pcl::PointCloud<pcl::PointXYZ>::Ptr &C = cloud_partitions[ix][iy];
		C->points.push_back (cloud->points[i]);
	}

	for (int i=0; i<nx; i++) {
		for (int j=0; j<ny; j++) {
			cloud_partitions[i][j]->height = cloud_partitions[i][j]->points.size ();
			cloud_partitions[i][j]->width = 1;
		}
	}
}
void get_search_space (pcl::PointCloud<pcl::PointXYZ>::Ptr& partition1, 
                       pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud2,
                       pcl::PointCloud<pcl::PointXYZ>::Ptr& searchspace, 
                       float wx, float wy) 
{
  cout << "get_search_space(), # of points in partition1 = " << partition1->points.size () << endl;
  float minx, maxx, miny, maxy;
  find_min_max (partition1, minx, maxx, miny, maxy);
  cout << "minx = " << minx << " maxx = " << maxx << endl;
  cout << "miny = " << miny << " maxy = " << maxy << endl;
  
  cout << "Search space = " << minx-dx << " " << minx+dx << " " << miny-dy << " " << miny+dy << endl;

  for (size_t i = 0; i < cloud2->points.size (); i++) {
    if (cloud2->points[i].x >= minx-dx && cloud2->points[i].x < maxx+dx && 
       cloud2->points[i].y >= miny-dy && cloud2->points[i].y < maxy+dy) {
      searchspace->points.push_back (cloud2->points[i]);
    }
  }
  searchspace->width = 1;
  searchspace->height = searchspace->points.size ();
  cout << "get_search_space(), # of points in searchspace = " << searchspace->points.size () << endl;
}
void window_icp_pt_to_plane (pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud1, 
                             pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud2) 
{
  cout << "Window ICP Point to Plane..\n";
  cout << "Number of windows, nx = " << nx << " ny = " << ny << endl;
  ofstream ofile ("x/partition.pcd");
  if (!ofile) {
    cout << "Cannot create output files.. The output files are created in a sub directory ./x under the current directory.."
      "Please verify that the directory exists..\n";
    return;
  }
  else {
    remove ("x/partition.pcd");
  }

  vector < vector <pcl::PointCloud<pcl::PointXYZ>::Ptr> > cloud1_partitions;
  partition_point_cloud (cloud1, cloud1_partitions, wx, wy, nx, ny);

  int k=1;
  for (size_t i = 0; i < nx; i++) {
    for (size_t j = 0; j < ny; j++) {
      pcl::PointCloud<pcl::PointXYZ>::Ptr searchspace (new pcl::PointCloud<pcl::PointXYZ>);
      cout << "Getting search space for partition " << i << " , " << j << endl;
      cout << "#, of pts = " << cloud1_partitions[i][j]->points.size () << endl;
      if (cloud1_partitions[i][j]->points.size () == 0) {
        continue;
      }

      get_search_space (cloud1_partitions[i][j], cloud2, searchspace, wx, wy);
      //get_search_space(cloud1_partitions[i][j], cloud2, searchspace, wx, wy);
      cout << "Search space for the partition = " << searchspace->points.size () << endl;
      
      ostringstream ostr;
      ostr << "x/window-" << k << ".pcd";
      cout << "Saving file " << ostr.str () << endl;
      pcl::io::savePCDFileASCII (ostr.str (), *(cloud1_partitions[i][j]));

      if (searchspace->points.size ()) {
        ostr.str ("");
        ostr << "x/searchspace-" << k << ".pcd";
        cout << "Saving file " << ostr.str () << endl;
        pcl::io::savePCDFileASCII (ostr.str (), *searchspace);
      }
      k++;
    }
  }

  delete cloud2.get ();

  for (int i=1; i<k; i++) {
    ostringstream ostr;
    ostr << "x/window-" << i << ".pcd";
    pcl::PointCloud<pcl::PointXYZ>::Ptr window (new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ> (ostr.str (), *window) == -1) {
      cout << "Couldn't read file " << ostr.str () << endl;
      return;
    }
    else {
      cout << "Loaded file " << ostr.str () << endl;
    }

    ostr.str ("");
    ostr << "x/searchspace-" << i << ".pcd";
    pcl::PointCloud<pcl::PointXYZ>::Ptr searchspace (new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ> (ostr.str (), *searchspace) == -1) {
      //cout << "Couldn't read file " << ostr.str() << endl;
      cout << "Skipping parition " << i << " in the 'before' cloud as corresponding search space was not found in the 'after' cloud..\n";
      continue;
    }
    else {
      cout << "Loaded file " << ostr.str () << endl;
    }

    pcl::PointCloud<pcl::PointNormal>::Ptr src (new pcl::PointCloud<pcl::PointNormal>);
    pcl::copyPointCloud (*window, *src);
    pcl::PointCloud<pcl::PointNormal>::Ptr tgt (new pcl::PointCloud<pcl::PointNormal>);

    pcl::copyPointCloud (*searchspace, *tgt);

    pcl::NormalEstimation<pcl::PointNormal, pcl::PointNormal> norm_est;
    norm_est.setSearchMethod (pcl::search::KdTree<pcl::PointNormal>::Ptr (new pcl::search::KdTree<pcl::PointNormal>));
    norm_est.setKSearch (10);
    norm_est.setInputCloud (tgt);
    norm_est.compute (*tgt);

    pcl::IterativeClosestPoint<pcl::PointNormal, pcl::PointNormal> icp;
    icp.disableRANSAC ();
    typedef pcl::registration::TransformationEstimationPointToPlaneLLS<pcl::PointNormal, pcl::PointNormal> PointToPlane;
    boost::shared_ptr<PointToPlane> point_to_plane (new PointToPlane);
    icp.setTransformationEstimation (point_to_plane);
    icp.setInputCloud (src);
    icp.setInputTarget (tgt);
		icp.setRANSACOutlierRejectionThreshold (ransac_par);
    icp.setRANSACIterations (100);
    icp.setMaximumIterations (1000);
    icp.setTransformationEpsilon (transformation_epsilon);
    pcl::PointCloud<pcl::PointNormal> output;
    icp.align (output);
    pcl::PointCloud<pcl::PointXYZ> outputxyz;
    pcl::copyPointCloud (output, outputxyz);
    std::cout << icp.getFinalTransformation () << std::endl;	

    ostr.str ("");
    
    ostr << "x/final"<< i <<".pcd";
    cout << "Saving file " << ostr.str () << endl;
    pcl::io::savePCDFileASCII(ostr.str ().c_str (), outputxyz);
    cout << "Transformation for partition " << i << endl;
    std::cout << icp.getFinalTransformation () << std::endl;	
  }
}
int main (int argc, char *argv[]) 
{
	pcl::console::setVerbosityLevel (pcl::console::L_DEBUG);

	if (parse_arguments (argc, argv) == -1) {
    return -1;
  }

  ofstream ofile ("x/partition.pcd");
  if (!ofile) {
    cout << "Cannot create output files.. The output files are created in a sub directory ./x under the current directory.."
      "Please verify that the directory exists..\n";
    return -1;
  }
  else {
    remove ("x/partition.pcd");
  }

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1 (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2 (new pcl::PointCloud<pcl::PointXYZ>);

  cout << "Loading file " << f1 << endl;
	if (pcl::io::loadPCDFile<pcl::PointXYZ> (f1, *cloud1) == -1) {
		cout << "Couldn't read file " << f1 << endl;
		return -1;
	}
	cout << "Loaded " << cloud1->width * cloud1->height << " data points from " <<  f1 << endl;

  cout << "Loading file " << f2 << endl;
	if (pcl::io::loadPCDFile<pcl::PointXYZ> (f2, *cloud2) == -1) {
		cout << "Couldn't read file " << f2 << endl;
		return -1;
	}
	cout << "Loaded " << cloud2->width * cloud2->height	<< " data points from " <<  f2	<< endl;

	boost::posix_time::ptime start = boost::posix_time::second_clock::local_time ();
  window_icp_pt_to_plane (cloud1, cloud2);
	boost::posix_time::ptime end = boost::posix_time::second_clock::local_time ();
	cout << "Total duration of the algorithm = " << end - start << endl;

	return 0;
}
