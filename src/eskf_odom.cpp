#include "eskf_odom.h"

using namespace std;

// EskfOdom::EskfOdom(ros::NodeHandle &nh) :
EskfOdomAlgNode::EskfOdom(ros::NodeHandle &nh) :
  public algorithm_base::IriBaseAlgorithm<EskfOdomAlgorithm>
{
  this->public_node_handle_ = nh;

  this->a_ = read_from_file("/home/arcas/iri-lab/ros/catkin_ws/src/external/angel/eskf_odom/data/kinton_outdoor_paper/2/linear_acceleration.txt");
  this->w_ = read_from_file("/home/arcas/iri-lab/ros/catkin_ws/src/external/angel/eskf_odom/data/kinton_outdoor_paper/2/angular_velocity.txt");
  this->px4_ = read_from_file("/home/arcas/iri-lab/ros/catkin_ws/src/external/angel/eskf_odom/data/kinton_outdoor_paper/2/opt_flow.txt");
  this->llstatus_ = read_from_file("/home/arcas/iri-lab/ros/catkin_ws/src/external/angel/eskf_odom/data/kinton_outdoor_paper/2/ll_status.txt");
}

// EskfOdom::~EskfOdom(void)
EskfOdomAlgNode::~EskfOdom(void)
{
}

// vector <vector<double>> EskfOdom::read_from_file(const string& path)
vector <vector<double>> EskfOdomAlgNode::read_from_file(const string& path)
{

  vector <vector<double>> data;
  ifstream infile(path.c_str());

  string s;
  getline(infile,s);

  while (infile)
  {
    if (!getline(infile,s)) break;

    istringstream ss(s);
    vector<double> record;

    while (ss)
    {
      string s;
      if (!getline( ss, s, ',' )) break;
      record.push_back(atof(s.c_str()));
    }

    data.push_back(record);
  }

  if (!infile.eof())
  {
    cerr << "Error reading file " << path.c_str() << "\n";
  }

  return data;
}

// bool EskfOdom::run()
bool EskfOdomAlgNode::run()
{
  ros::Rate loop_rate(100);

  int count = 0;
  while (ros::ok())
  {
    ROS_DEBUG("time: %f",this->a_[count][0]);
    ROS_DEBUG("ax: %f, ay: %f, az: %f", this->a_[count][1],this->a_[count][2],this->a_[count][3]);
    ROS_DEBUG("wx: %f, wy: %f, wz: %f", this->w_[count][1],this->w_[count][2],this->w_[count][3]);
    ROS_DEBUG("gdist: %f, vx: %f, vy: %f", this->px4_[count][4],this->px4_[count][7],this->px4_[count][8]);
    ROS_DEBUG("flying: %f", this->llstatus_[count][10]);

    this->alg_.lock();  // protect algorithm

    this->alg_.unlock();  



    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }

  return true;
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "eskf_odom");

  ros::NodeHandle n;


  // EskfOdom eskf_node(n);
  EskfOdomAlgNode eskf_node(n);
  eskf_node.run();

  return EXIT_SUCCESS;

}