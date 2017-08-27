/*
 Point Cloud Registration Tool
 
 BSD 2-Clause License
 Copyright (c) 2017, Dario Rethage
 See LICENSE at package root for full license
 */

#ifndef Registrator_hpp
#define Registrator_hpp

// Standard
#include <stdio.h>

// Boost
#include <boost/shared_ptr.hpp>

// PCL
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/correspondence.h>
#include <pcl/registration/icp.h>
#include <pcl/console/time.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/shot_omp.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/registration/transformation_estimation_svd.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>

// PRT
#include "types.hpp"
#include "util.hpp"

// GFLAGS
#include <gflags/gflags.h>
DECLARE_string(registration_technique);
DECLARE_double(residual_threshold);
DECLARE_uint64(num_ksearch_neighbors);
DECLARE_double(descriptor_radius);
DECLARE_double(subsampling_radius);
DECLARE_double(consensus_inlier_threshold);
DECLARE_uint64(consensus_max_iterations);
DECLARE_uint64(icp_max_iterations);
DECLARE_double(icp_max_correspondence_distance);

using namespace PRT;

class Registrator
{
public:
    // Constants
    static const std::string DEFAULT_registration_technique;
    static const double DEFAULT_residual_threshold;
    static const int DEFAULT_num_ksearch_neighbors;
    static const double DEFAULT_descriptor_radius;
    static const double DEFAULT_subsampling_radius;
    static const double DEFAULT_consensus_inlier_threshold;
    static const int DEFAULT_consensus_max_iterations;
    static const int DEFAULT_icp_max_iterations;
    static const double DEFAULT_icp_max_correspondence_distance;
    
    // Typedefs
    typedef boost::shared_ptr< Registrator > Ptr;
    
    // Constructor
    Registrator();
    
    /**
     Sets the source cloud
     The source cloud is that which we aim to align to the source cloud
     */
    void setSourceCloud(const PointCloudT::Ptr &cloud);
    
    /**
     Sets the target cloud
     The target cloud is that which we aim to align against
     */
    void setTargetCloud(const PointCloudT::Ptr &cloud);
    
    //Setup methods
    void setSubsamplingRadius(double r);
    void setDescriptorRadius(double r);
    void setNumKSearchNeighbors(int n);
    void setConsensusInlierThreshold(double t);
    void setConsensusMaxIterations(int i);
    void setICPMaxCorrespondenceDistance(double d);
    void setICPMaxIterations(int i);
    
    //Computation methods
    void performRegistration(const std::string);
    void computeResidualColormap();
    int saveResidualColormapPointCloud(std::string &filepath);
    
    void setResidualThreshold(double residual_threshold);
    
    //Get methods
    Eigen::Matrix4f getCorrespondenceBasedTransformation();
    Eigen::Matrix4f getICPBasedTransformation();
    Eigen::Matrix4f getCombinedTransformation();
    PointCloudT::Ptr getRegisteredCloud();
    PointCloudRGBT::Ptr getRegisteredRGBCloud();
    PointCloudT::Ptr getSourceCloud();
    PointCloudT::Ptr getTargetCloud();
    PointCloudT::Ptr getSourceKeypointsCloud();
    PointCloudT::Ptr getTargetKeypointsCloud();
    DoubleVectorPtr getTargetToRegisteredResiduals();
    DoubleVectorPtr getRegisteredToTargetResiduals();
    double getMaxTargetToRegisteredResidual();
    double getMaxRegisteredToTargetResidual();
    double getFScoreAtThreshold(double threshold = -1);
    int saveFinalTransform(const std::string filepath);
    void saveFScoreAtThreshold(const std::string filepath, double threshold);
    double getResidualThreshold();
    
private:
    //Computation methods
    void correspondenceBasedRegistration();
    void ICPBasedRegistration();
    void computeNormals();
    void extractKeypoints();
    void computeDescriptors();
    void computeResiduals();
    void findCorrespondences();
    void filterCorrespondences();
    void computeCorrespondenceBasedTransformation();
    
    void setRegisteredCloudToDefaultColor();
    
    //Data members
    PointCloudT::Ptr s_cloud_; //Source Cloud
    PointCloudT::Ptr t_cloud_; //Target Cloud
    PointCloudT::Ptr r_cloud_; //Source Cloud after Registration
    PointCloudRGBT::Ptr r_cloud_rgb_; //Source Cloud after Registration (with RGB attributes)
    
    NormalCloudT::Ptr s_cloud_normals_; //Source Cloud Normals
    NormalCloudT::Ptr t_cloud_normals_; //Target Cloud Normals
    
    PointCloudT::Ptr s_cloud_keypoints_; //Source Cloud Keypoints
    PointCloudT::Ptr t_cloud_keypoints_; //Target Cloud Keypoints
    
    SHOTDescriptorCloudT::Ptr s_cloud_descriptors_; //Source SHOT Descriptors
    SHOTDescriptorCloudT::Ptr t_cloud_descriptors_; //Target SHOT Descriptors
    
    PointCloudT::Ptr s_cloud_corr_; //Source Cloud Correspondence Points
    PointCloudT::Ptr t_cloud_corr_; //Target Cloud Correspondence Points
    
    pcl::CorrespondencesPtr correspondences_; //Keypoint Correspondences
    
    DoubleVectorPtr t_r_residuals_; //Residuals from target cloud to registered cloud
    DoubleVectorPtr r_t_residuals_; //Residuals from registered cloud to target cloud
    double t_r_max_residual_; //Max residual from target cloud to registered cloud
    double r_t_max_residual_; //Max residual from registered cloud to target cloud
    
    Eigen::Matrix4f correspondence_T_; //Transformation matrix estimated from point correspondences
    Eigen::Matrix4f icp_T_; //Transformation matrix estimated from ICP
    Eigen::Matrix4f combined_T_; //Transformation matrix from both correspondence matching & ICP
    
    double uniform_sampling_radius_;
    double descriptor_radius_;
    int num_k_search_neighbors_;
    double consensus_inlier_threshold_;
    int consensus_max_iterations_;
    int icp_max_iterations_;
    double icp_max_correspondence_distance_;
    
    bool is_correspondence_matching_complete_ = false;
    bool is_icp_complete_ = false;
    bool is_residual_colormap_computed_ = false;
    
    DoubleVectorPtr colormap_residuals_;
    RGB r_color_;
    
    double residual_threshold_;
    
};

#endif /* Registrator_hpp */
