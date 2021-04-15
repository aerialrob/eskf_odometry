#ifndef _ESKF_ODOM_H
#define _ESKF_ODOM_H

#include "ros/ros.h"

#include <fstream>
#include <iostream>
#include <sstream>
#include <string>

#include <eigen3/Eigen/Dense>

// Subscribers
#include <sensor_msgs/Imu.h>
#include <px_comm/OpticalFlow.h>
#include <asctec_msgs/LLStatus.h>


// Algorithm (wrapper of the low-level library)
#include <iri_base_algorithm/iri_base_algorithm.h>
#include "eskf_odom_alg.h"

using namespace std;
using namespace Eigen;

// class EskfOdom : public algorithm_base::IriBaseAlgorithm<EskfOdomAlgorithm>

class EskfOdomAlgNode : public algorithm_base::IriBaseAlgorithm<EskfOdomAlgorithm>
{
    private:

        vector <vector<double>> a_;
        vector <vector<double>> w_;
        vector <vector<double>> px4_;
        vector <vector<double>> llstatus_;

        vector <vector<double>> read_from_file(const string& path);


        ros::NodeHandle public_node_handle_;


    public:
        // EskfOdom(ros::NodeHandle &nh);
        EskfOdomAlgNode(ros::NodeHandle &nh);
        // ~EskfOdom(void);
        ~EskfOdomAlgNode(void);
        bool run();
};

#endif
