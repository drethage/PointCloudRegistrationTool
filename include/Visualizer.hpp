/*
 Point Cloud Registration Tool
 
 BSD 2-Clause License
 Copyright (c) 2017, Dario Rethage
 See LICENSE at package root for full license
 */

#ifndef Visualizer_hpp
#define Visualizer_hpp

// Standard
#include <stdio.h>
#include <math.h>
#include <string>
#include <vector>
#include <algorithm>

// Boost
#include <boost/shared_ptr.hpp>

// PCL
#include <pcl/common/io.h>
#include <pcl/console/time.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/recognition/color_gradient_dot_modality.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/pcl_plotter.h>

// VTK
#include <vtkVersion.h>
#include <vtkRenderWindow.h>
#include <vtkWindow.h>
#include <vtkSmartPointer.h>
#include <vtkPNGWriter.h>
#include <vtkWindowToImageFilter.h>

// PRT
#include "types.hpp"
#include "util.hpp"
#include "Registrator.hpp"

using namespace PRT;

void keyboardEventOccurred (const pcl::visualization::KeyboardEvent& event, void* none);

class Visualizer
{
public:
    //Types
    typedef boost::shared_ptr< Visualizer > Ptr;
    
    //Constructor
    Visualizer(const std::string &name);
    
    //Set methods
    void setRegistrator(const Registrator::Ptr &registrator);
    void setHistogramXRange(double min = 0, double max = -1);
    
    //Toggle switches
    void toggleShowKeypoints();
    void toggleShowSource();
    void toggleShowTarget();
    void toggleShowRegistered();
    
    //Interactive methods
    void updateText();
    void computeResidualHistogram();
    void incrementResidualThreshold();
    void decrementResidualThreshold();
    void saveHistogramImage(std::string &filepath);
    
    void visualize();
    void closeVisualizer();
    
private:
    
    Registrator::Ptr registrator_;
    pcl::visualization::PCLVisualizer::Ptr viewer_;
    PCLPlotterPtr plotter_;
    
    // Switches
    bool show_keypoints_ = false;
    bool show_source_ = false;
    bool show_target_ = false;
    bool show_registered_ = false;
    
    // Display states
    bool keypoints_visible_ = false;
    bool s_cloud_visible_ = false;
    bool t_cloud_visible_ = false;
    bool r_cloud_visible_ = false;
    bool histogram_computed_ = false;
    bool text_added_ = false;
    bool stop_ = false;

    // Colors
    RGB s_color_;
    RGB t_color_;
    RGB r_color_;
    RGB keypoints_color_;
    
    //Sizes
    double point_size_;
    double keypoint_size_;

    DoubleVectorPtr histogram_residuals_;
    
    int getNumHistogramBins();
    void resetRegisteredCloud();
    
};

#endif /* Visualizer_hpp */
