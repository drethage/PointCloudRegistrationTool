/*
 Point Cloud Registration Tool
 
 BSD 2-Clause License
 Copyright (c) 2017, Dario Rethage
 See LICENSE at package root for full license
 */

#include "Visualizer.hpp"

void keyboardEventOccurred (const pcl::visualization::KeyboardEvent& event, void* obj) {
    
    Visualizer* vis = (Visualizer*) obj;
    
    if (event.getKeySym () == "s" && event.keyDown ())
        vis->toggleShowSource();
    
    if (event.getKeySym () == "t" && event.keyDown ())
        vis->toggleShowTarget();
    
    if (event.getKeySym () == "r" && event.keyDown ())
        vis->toggleShowRegistered();
    
    if (event.getKeySym () == "k" && event.keyDown ())
        vis->toggleShowKeypoints();
    
    if (event.getKeySym () == "Up" && event.keyDown ())
        vis->incrementResidualThreshold();
    
    if (event.getKeySym () == "Down" && event.keyDown ())
        vis->decrementResidualThreshold();
    
}

Visualizer::Visualizer(const std::string &name) {
    
    viewer_ = pcl::visualization::PCLVisualizer::Ptr(new pcl::visualization::PCLVisualizer(name));
    viewer_->setBackgroundColor(0,0,0);
    viewer_->setCameraPosition (-7.5, 0, 7.5, 0, 0, 1, 0);
    viewer_->setSize (1024, 768);
    viewer_->registerKeyboardCallback (&keyboardEventOccurred, this);
    viewer_->setShowFPS(false);
    
    plotter_ = PCLPlotterPtr(new pcl::visualization::PCLPlotter);
    plotter_->setTitle("Residuals Histogram");
    plotter_->setXTitle("Distance (m)");
    plotter_->setYTitle("Frequency");
    plotter_->setShowLegend(false);
    
    registrator_ = Registrator::Ptr();
    
    s_color_ = RGB(0, 0, 255); //Blue
    t_color_ = RGB(0, 255, 0); // Green
    r_color_ = RGB(255, 0, 0); // Red
    keypoints_color_ = RGB(255, 255, 0); // Yellow
    
    show_source_ = false;
    show_target_ = false;
    show_registered_ = true;
    
    histogram_residuals_ = DoubleVectorPtr(new DoubleVector());
    
    point_size_ = 1.5;
    keypoint_size_ = 3;
}

void Visualizer::setRegistrator(const Registrator::Ptr &registrator) {
    registrator_ = registrator;
}

void Visualizer::incrementResidualThreshold() {
    registrator_->setResidualThreshold((registrator_->getResidualThreshold() < 1) ? registrator_->getResidualThreshold() + 0.01 : registrator_->getResidualThreshold());
    registrator_->computeResidualColormap();
    
    updateText();
    setHistogramXRange();
    computeResidualHistogram();
    resetRegisteredCloud();
}

void Visualizer::decrementResidualThreshold() {
    registrator_->setResidualThreshold((registrator_->getResidualThreshold() >= .02) ? registrator_->getResidualThreshold() - 0.01 : registrator_->getResidualThreshold());
    registrator_->computeResidualColormap();
    
    updateText();
    setHistogramXRange();
    computeResidualHistogram();
    resetRegisteredCloud();
}

int Visualizer::getNumHistogramBins() {
    return 2 * std::max((int)(registrator_->getResidualThreshold() * 100),1); //One bar represents 0.5cm
}

void Visualizer::setHistogramXRange(double min, double max) {
    
    if (max == -1)
        max = registrator_->getResidualThreshold();
    
    plotter_->setXRange(min, max);
}

void Visualizer::toggleShowKeypoints() {
    show_keypoints_ = !show_keypoints_;
}

void Visualizer::toggleShowSource() {
    show_source_ = !show_source_;
}

void Visualizer::toggleShowTarget() {
    show_target_ = !show_target_;
}

void Visualizer::toggleShowRegistered() {
    show_registered_ = !show_registered_;
}

void Visualizer::updateText() {
    
    if (text_added_) {
        
        std::stringstream ss;
        ss << "f-score @ threshold = " << registrator_->getFScoreAtThreshold(registrator_->getResidualThreshold());
        viewer_->updateText(ss.str(), 50, 175, "fscore");
        
        ss.str(std::string()); // Clear stringstream
        ss << "max residual threshold = " << registrator_->getResidualThreshold() << " (modify: up/down)";
        viewer_->updateText(ss.str(), 50, 200, "max_residual_threshold");
        
    }
}

void Visualizer::resetRegisteredCloud() {

    if (r_cloud_visible_) {
        viewer_->removePointCloud("r_cloud");
        r_cloud_visible_ = false;
    }
    
    if (show_registered_) {
        pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> color(registrator_->getRegisteredRGBCloud());
        viewer_->addPointCloud<PointRGBT> (registrator_->getRegisteredRGBCloud(), color, "r_cloud");
        r_cloud_visible_ = true;
        viewer_->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, point_size_, "r_cloud");
    }
    
}

void Visualizer::computeResidualHistogram() {
    
    if (histogram_residuals_->size() != registrator_->getRegisteredToTargetResiduals()->size())
        histogram_residuals_->resize(registrator_->getRegisteredToTargetResiduals()->size());
    
    std::copy(registrator_->getRegisteredToTargetResiduals()->begin(), registrator_->getRegisteredToTargetResiduals()->end(), histogram_residuals_->begin());
    
    util::removeElementsAboveThreshold(histogram_residuals_, registrator_->getResidualThreshold()); //Remove residual values above residual_threshold_
    
    if (!histogram_computed_) {
        plotter_->clearPlots();
        plotter_->addHistogramData(*histogram_residuals_, getNumHistogramBins());
    } else {
        #if VTK_MAJOR_VERSION > 6
        plotter_->clearPlots();
        plotter_->addHistogramData(*histogram_residuals_, getNumHistogramBins());
        #endif
    }
    
    histogram_computed_ = true;
}

void Visualizer::saveHistogramImage(std::string &filepath) {

    if (!histogram_computed_)
        computeResidualHistogram();
    
    plotter_->spinOnce();
    
    vtkWindowToImageFilter* wif = vtkWindowToImageFilter::New();
    wif->Modified();
    wif->SetInput(plotter_->getRenderWindow());
    wif->SetInputBufferTypeToRGB();
    wif->Update();
    
    vtkPNGWriter* screenshot_writer = vtkPNGWriter::New();
    
    #if VTK_MAJOR_VERSION<6
    screenshot_writer->SetInput(wif->GetOutput());
    #else
    screenshot_writer->SetInputData(wif->GetOutput());
    #endif
    screenshot_writer->SetFileName (filepath.c_str());
    screenshot_writer->Write();
    screenshot_writer->Delete();
    
    wif->Delete();
    
    #if VTK_MAJOR_VERSION < 6
    plotter_->clearPlots();
    plotter_->close();
    plotter_->getRenderWindow()->Delete();
    #endif
    
}

void Visualizer::visualize() {
    
    if (!histogram_computed_)
        computeResidualHistogram();
    
    while (!viewer_->wasStopped())
    {
        viewer_->spinOnce();
        #if VTK_MAJOR_VERSION > 6
        plotter_->spinOnce();
        #endif
        
        //Registered
        if (show_registered_ && !registrator_->getRegisteredRGBCloud()->empty() && !r_cloud_visible_) {
            
            pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> color(registrator_->getRegisteredRGBCloud());
            viewer_->addPointCloud<PointRGBT> (registrator_->getRegisteredRGBCloud(), color, "r_cloud");
            r_cloud_visible_ = true;
            viewer_->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, point_size_, "r_cloud");
        } else if (!show_registered_ && r_cloud_visible_) {
            viewer_->removePointCloud("r_cloud");
            r_cloud_visible_ = false;
        }
        
        //Source
        if (show_source_ && !registrator_->getSourceCloud()->empty() && !s_cloud_visible_) {
            pcl::visualization::PointCloudColorHandlerCustom<PointT> s_cloud_color_h (registrator_->getSourceCloud(), s_color_.r, s_color_.g, s_color_.b);
            viewer_->addPointCloud<PointT> (registrator_->getSourceCloud(), s_cloud_color_h, "s_cloud");
            s_cloud_visible_ = true;
            viewer_->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, point_size_, "s_cloud");
        } else if (!show_source_ && s_cloud_visible_) {
            viewer_->removePointCloud("s_cloud");
            s_cloud_visible_ = false;
        }
        
        //Target
        if (show_target_ && !registrator_->getTargetCloud()->empty() && !t_cloud_visible_) {
            pcl::visualization::PointCloudColorHandlerCustom<PointT> t_cloud_color_h (registrator_->getTargetCloud(), t_color_.r, t_color_.g, t_color_.b);
            viewer_->addPointCloud<PointT> (registrator_->getTargetCloud(), t_cloud_color_h, "t_cloud");
            t_cloud_visible_ = true;
            viewer_->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, point_size_, "t_cloud");
        } else if (!show_target_ && t_cloud_visible_) {
            viewer_->removePointCloud("t_cloud");
            t_cloud_visible_ = false;
        }
        
        //Source Keypoints
        if (show_keypoints_ && !keypoints_visible_) {
            
            if (!registrator_->getSourceKeypointsCloud()->empty()) {
                pcl::visualization::PointCloudColorHandlerCustom<PointT> s_cloud_keypoints_color_h (registrator_->getSourceKeypointsCloud(), keypoints_color_.r, keypoints_color_.g, keypoints_color_.b);
                viewer_->addPointCloud<PointT> (registrator_->getSourceKeypointsCloud(), s_cloud_keypoints_color_h, "s_cloud_keypoints");
                viewer_->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, keypoint_size_, "s_cloud_keypoints");
                keypoints_visible_ = true;
            }
            
            if (!registrator_->getSourceKeypointsCloud()->empty()) {
                pcl::visualization::PointCloudColorHandlerCustom<PointT> t_cloud_keypoints_color_h (registrator_->getTargetKeypointsCloud(), keypoints_color_.r, keypoints_color_.g, keypoints_color_.b);
                viewer_->addPointCloud<PointT> (registrator_->getTargetKeypointsCloud(), t_cloud_keypoints_color_h, "t_cloud_keypoints");
                viewer_->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, keypoint_size_, "t_cloud_keypoints");
                keypoints_visible_ = true;
            }
            
        } else if (!show_keypoints_ && keypoints_visible_) {
            viewer_->removePointCloud("s_cloud_keypoints");
            viewer_->removePointCloud("t_cloud_keypoints");
            keypoints_visible_ = false;
        }
        
        //Text
        if (!text_added_) {
            std::string command_panel = "S: toggle source cloud\nT: toggle target cloud\nR: toggle registered cloud\nK: toggle keypoints\nF: adjust center of revolution\nQ: quit";
            viewer_->addText(command_panel, 50, 75, 15, 1, 1, 1, "command_panel");
            
            std::stringstream ss;
            ss << "f-score @ threshold = " << registrator_->getFScoreAtThreshold(registrator_->getResidualThreshold());
            viewer_->addText(ss.str(), 50, 175, 15, 1, 1, 1, "fscore");
            
            ss.str(std::string()); // Clear stringstream
            ss << "max residual threshold = " << registrator_->getResidualThreshold() << " (modify: up/down)";
            viewer_->addText(ss.str(), 50, 200, 15, 1, 1, 1, "max_residual_threshold");
            
            text_added_ = true;
        }
    }
    
}
