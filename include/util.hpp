/*
 Point Cloud Registration Tool
 
 BSD 2-Clause License
 Copyright (c) 2017, Dario Rethage
 See LICENSE at package root for full license
 */

#ifndef util_hpp
#define util_hpp

// Standard
#include <stdio.h>
#include <string>
#include <iostream>
#include <locale>
#include <random>
#include <fstream>
#include <sys/stat.h>

// Boost
#include <boost/bind.hpp>

// Eigen
#include <Eigen/Dense>

// PCL
#include <pcl/common/transforms.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/obj_io.h>
#include <pcl/console/time.h>

// PRT
#include "types.hpp"
#include "hsvrgb.hpp"
#include "tinyply.h"

// GFLAGS
#include <gflags/gflags.h>
DECLARE_bool(verbose);

using namespace PRT;

namespace util
{
    
    void print4x4Matrix (const Eigen::Matrix4f & matrix);
    double uniformRandom();
    int loadPointCloud(std::string filepath, PointCloudT &cloud);
    int writePointCloudToPLY(std::string filepath, PointCloudT &cloud);
    int writePointCloudToPLY(std::string filepath, PointCloudRGBT &cloud);
    void transformPointCloud(PointCloudT &cloud, double alpha, double beta, double gamma, double tx, double ty, double tz);
    void randomTransformPointCloud(PointCloudT::Ptr &cloud);
    void hsv2rgb(double& r, double& g, double& b, double& h, double& s, double& v);
    void rgb2hsv(double& r, double& g, double& b, double& h, double& s, double& v);
    std::string stringTail(const std::string s, const size_t l);
    void vectorMin(DoubleVectorPtr &v, double min);
    void removeElementsAboveThreshold(DoubleVectorPtr &v, double threshold);
    void vectorRescale(DoubleVectorPtr &v, double max_element);
    int writeMatrixToFile(const Eigen::MatrixXf &matrix, const std::string filepath);
    FilepairVectorPtr readBatchProcessingFile(std::string filepath);
    int ensureUniqueFilepath(std::string &filepath);
    bool fileExists(const std::string& filepath);
    std::string extractFileExtension(const std::string &filepath);
    std::string removeFileExtension(const std::string &filepath);
    void writeStringToFile(const std::string& filepath, const std::string s);
}


#endif /* util_hpp */
