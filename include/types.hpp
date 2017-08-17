/*
 Point Cloud Registration Tool
 
 BSD 2-Clause License
 Copyright (c) 2017, Dario Rethage
 See LICENSE at package root for full license
 */

#ifndef types_hpp
#define types_hpp

// Standard
#include <vector>

// PCL
#include <pcl/point_types.h>
#include <pcl/common/projection_matrix.h>
#include <pcl/visualization/pcl_plotter.h>

// Boost
#include <boost/shared_ptr.hpp>

namespace PRT {
    
    struct filepair_t {
        std::string sourcefile;
        std::string targetfile;
    };
    
    struct RGB {
        uint8_t r;
        uint8_t g;
        uint8_t b;
        
        RGB() : r(0), g(0), b(0) {}
        
        RGB(const uint8_t r, const uint8_t g, const uint8_t b) : r(r), g(g), b(b) {}
    };
    
    typedef pcl::PointXYZ PointT;
    typedef pcl::PointCloud<PointT> PointCloudT;
    typedef pcl::Normal NormalT;
    typedef pcl::PointCloud<NormalT> NormalCloudT;
    typedef pcl::SHOT352 SHOTDescriptorT;
    typedef pcl::PointCloud<SHOTDescriptorT> SHOTDescriptorCloudT;
    typedef pcl::PointXYZRGB PointRGBT;
    typedef pcl::PointCloud<PointRGBT> PointCloudRGBT;
    
    typedef boost::shared_ptr< pcl::visualization::PCLPlotter > PCLPlotterPtr;
    
    typedef std::vector<float> FloatVector;
    typedef boost::shared_ptr< FloatVector > FloatVectorPtr;
    typedef std::vector<double> DoubleVector;
    typedef boost::shared_ptr< DoubleVector > DoubleVectorPtr;
    typedef std::vector<std::string> StringVector;
    typedef boost::shared_ptr< StringVector > StringVectorPtr;
    typedef std::vector<filepair_t> FilepairVector;
    typedef boost::shared_ptr< FilepairVector > FilepairVectorPtr;
    
    
}

#endif /* types_hpp */
