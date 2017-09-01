/*
 Point Cloud Registration Tool
 
 BSD 2-Clause License
 Copyright (c) 2017, Dario Rethage
 See LICENSE at package root for full license
 */

#include "Registrator.hpp"

// Commandline Flags
const std::string Registrator::DEFAULT_registration_technique("both");
const double Registrator::DEFAULT_residual_threshold(0.1);
const int Registrator::DEFAULT_num_ksearch_neighbors(100);
const double Registrator::DEFAULT_descriptor_radius(1.0);
const double Registrator::DEFAULT_subsampling_radius(0.2);
const double Registrator::DEFAULT_consensus_inlier_threshold(0.2);
const int Registrator::DEFAULT_consensus_max_iterations(100);
const int Registrator::DEFAULT_icp_max_iterations(100);
const double Registrator::DEFAULT_icp_max_correspondence_distance(0.05);

DEFINE_string(registration_technique, Registrator::DEFAULT_registration_technique, "what kind of registration technique to use, 'correspondence', 'icp', 'both'");
DEFINE_double(residual_threshold, Registrator::DEFAULT_residual_threshold, "in meters, max residual used in calculation of f-score, also corresponds to blue in the residual colormap");
DEFINE_uint64(num_ksearch_neighbors, Registrator::DEFAULT_num_ksearch_neighbors, "number of neighbors to consider in normals computation");
DEFINE_double(descriptor_radius, Registrator::DEFAULT_descriptor_radius, "in meters, radius of surrounding points to consider during descriptor computation");
DEFINE_double(subsampling_radius, Registrator::DEFAULT_subsampling_radius, "in meters, determines the degree of subsampling used, all points within a sphere of the specified radius are replaced by their centroid");
DEFINE_double(consensus_inlier_threshold, Registrator::DEFAULT_consensus_inlier_threshold, "in meters, maximum distance a point can lie from a hypothesis during sample consensus to be considered an inlier");
DEFINE_uint64(consensus_max_iterations, Registrator::DEFAULT_consensus_max_iterations, "maximum number of iterations to perform sample consensus");
DEFINE_uint64(icp_max_iterations, Registrator::DEFAULT_icp_max_iterations, "maximum number of iterations to perform ICP");
DEFINE_double(icp_max_correspondence_distance, Registrator::DEFAULT_icp_max_correspondence_distance, "correspondences greater than this distance apart will be ignored in ICP");

Registrator::Registrator() {
    
    s_cloud_ = PointCloudT::Ptr();
    t_cloud_ = PointCloudT::Ptr();
    
    r_cloud_ = PointCloudT::Ptr(new PointCloudT);
    r_cloud_rgb_ = PointCloudRGBT::Ptr(new PointCloudRGBT);
    s_cloud_normals_ = NormalCloudT::Ptr(new NormalCloudT);
    t_cloud_normals_ = NormalCloudT::Ptr(new NormalCloudT);
    s_cloud_keypoints_ = PointCloudT::Ptr(new PointCloudT);
    t_cloud_keypoints_ = PointCloudT::Ptr(new PointCloudT);
    s_cloud_corr_ = PointCloudT::Ptr(new PointCloudT);
    t_cloud_corr_ = PointCloudT::Ptr(new PointCloudT);
    s_cloud_descriptors_ = SHOTDescriptorCloudT::Ptr(new SHOTDescriptorCloudT);
    t_cloud_descriptors_ = SHOTDescriptorCloudT::Ptr(new SHOTDescriptorCloudT);
    correspondences_ = pcl::CorrespondencesPtr(new pcl::Correspondences);
    t_r_residuals_ = DoubleVectorPtr (new DoubleVector);
    r_t_residuals_ = DoubleVectorPtr (new DoubleVector);
    t_r_max_residual_ = 0;
    r_t_max_residual_ = 0;
    
    correspondence_T_ = Eigen::Matrix4f::Identity();
    icp_T_ = Eigen::Matrix4f::Identity();
    combined_T_ = Eigen::Matrix4f::Identity();
    
    uniform_sampling_radius_ = 0.25;
    descriptor_radius_ = 1;
    num_k_search_neighbors_ = 100;
    consensus_inlier_threshold_ = 0.5;
    consensus_max_iterations_ = 100;
    icp_max_iterations_ = 100;
    icp_max_correspondence_distance_ = 0.05;
    
    residual_threshold_ = 0.25;
    
    colormap_residuals_ = DoubleVectorPtr(new DoubleVector());
    
    r_color_ = RGB(0, 255, 0); // Green
}

void Registrator::setSourceCloud(const PointCloudT::Ptr &cloud) {
    s_cloud_ = cloud;
}

void Registrator::setTargetCloud(const PointCloudT::Ptr &cloud) {
    t_cloud_ = cloud;
}

void Registrator::setDescriptorRadius(double r) {
    
    descriptor_radius_ = r;
}

void Registrator::setSubsamplingRadius(double r) {
    
    uniform_sampling_radius_ = r;
}

void Registrator::setNumKSearchNeighbors(int n) {
    
    num_k_search_neighbors_ = n;
}

void Registrator::setConsensusInlierThreshold(double t) {
    
    consensus_inlier_threshold_ = t;
}

void Registrator::setConsensusMaxIterations(int i) {
    
    consensus_max_iterations_ = i;
}

void Registrator::setICPMaxIterations(int i) {
    
    icp_max_iterations_ = i;
}

void Registrator::setICPMaxCorrespondenceDistance(double d) {
    
    icp_max_correspondence_distance_ = d;
}

Eigen::Matrix4f Registrator::getCorrespondenceBasedTransformation() {
    return correspondence_T_;
}

Eigen::Matrix4f Registrator::getICPBasedTransformation() {
    return icp_T_;
}

Eigen::Matrix4f Registrator::getCombinedTransformation() {
    return combined_T_;
}

PointCloudT::Ptr Registrator::getRegisteredCloud() {
    return r_cloud_;
}

PointCloudRGBT::Ptr Registrator::getRegisteredRGBCloud() {
    return r_cloud_rgb_;
}

PointCloudT::Ptr Registrator::getSourceCloud() {
    return s_cloud_;
}

PointCloudT::Ptr Registrator::getTargetCloud() {
    return t_cloud_;
}

PointCloudT::Ptr Registrator::getSourceKeypointsCloud() {
    return s_cloud_keypoints_;
}

PointCloudT::Ptr Registrator::getTargetKeypointsCloud() {
    return t_cloud_keypoints_;
}

DoubleVectorPtr Registrator::getTargetToRegisteredResiduals() {
    if (t_r_residuals_->empty())
        computeResiduals();
    
    return t_r_residuals_;
}

DoubleVectorPtr Registrator::getRegisteredToTargetResiduals() {
    if (r_t_residuals_->empty())
        computeResiduals();
    
    return r_t_residuals_;
}

double Registrator::getMaxTargetToRegisteredResidual() {
    return t_r_max_residual_;
}

double Registrator::getMaxRegisteredToTargetResidual() {
    return r_t_max_residual_;
}


/**
 Run through the entire correspondence-based registration routine
 */
void Registrator::correspondenceBasedRegistration() {
    computeNormals();
    extractKeypoints();
    computeDescriptors();
    findCorrespondences();
    filterCorrespondences();
    computeCorrespondenceBasedTransformation();
    is_correspondence_matching_complete_ = true;
}


/**
 Computes normals for each point in each point cloud
 */
void Registrator::computeNormals() {
    
    #pragma omp parallel sections
    {
        #pragma omp section
        {
            pcl::NormalEstimationOMP<PointT, NormalT> s_norm_estimation;
            s_norm_estimation.setKSearch (num_k_search_neighbors_);
            s_norm_estimation.setInputCloud (s_cloud_);
            s_norm_estimation.compute (*s_cloud_normals_);
            
            if(FLAGS_verbose) {
                std::cout << "Computed source cloud normals (" << s_cloud_normals_->size() << ")" << std::endl;
            }
            
        }
        
        #pragma omp section
        {
            pcl::NormalEstimationOMP<PointT, NormalT> t_norm_estimation;
            t_norm_estimation.setKSearch (num_k_search_neighbors_);
            
            t_norm_estimation.setInputCloud (t_cloud_);
            t_norm_estimation.compute (*t_cloud_normals_);
            
            if(FLAGS_verbose) {
                std::cout << "Computed target cloud normals (" << t_cloud_normals_->size() << ")" << std::endl;
            }
        }
    }
    
}


/**
 Extracts keypoints using uniform sampling
 */
void Registrator::extractKeypoints() {
    
    pcl::VoxelGrid<PointT> subsampling;
    subsampling.setLeafSize (uniform_sampling_radius_, uniform_sampling_radius_, uniform_sampling_radius_);
    
    subsampling.setInputCloud (s_cloud_);
    subsampling.filter (*s_cloud_keypoints_);
    
    if(FLAGS_verbose) {
        std::cout << "Subsampled source cloud from " << s_cloud_->size() << " -> " << s_cloud_keypoints_->size() << std::endl;
    }

    subsampling.setInputCloud(t_cloud_);
    subsampling.filter(*t_cloud_keypoints_);
    
    if(FLAGS_verbose) {
        std::cout << "Subsampled target cloud from " << t_cloud_->size() << " -> " << t_cloud_keypoints_->size() << std::endl;
    }
}


/**
 Computes SHOT descriptors on keypoint cloud
 */
void Registrator::computeDescriptors() {
    
    #pragma omp parallel sections
    {
        #pragma omp section
        {
            pcl::SHOTEstimationOMP<PointT, NormalT, SHOTDescriptorT> s_shot;
            s_shot.setRadiusSearch (descriptor_radius_);

            s_shot.setInputCloud (s_cloud_keypoints_);
            s_shot.setInputNormals (s_cloud_normals_);
            s_shot.setSearchSurface (s_cloud_);
            s_shot.compute (*s_cloud_descriptors_);
            
            if(FLAGS_verbose) {
                std::cout << "Computed " << s_cloud_descriptors_->size() << " descriptors on source cloud keypoints" << std::endl;
            }
            
        }

        #pragma omp section
        {
            pcl::SHOTEstimationOMP<PointT, NormalT, SHOTDescriptorT> t_shot;
            t_shot.setRadiusSearch (descriptor_radius_);
            
            t_shot.setInputCloud (t_cloud_keypoints_);
            t_shot.setInputNormals (t_cloud_normals_);
            t_shot.setSearchSurface (t_cloud_);
            t_shot.compute (*t_cloud_descriptors_);
            
            if(FLAGS_verbose) {
                std::cout << "Computed " << t_cloud_descriptors_->size() << " descriptors on target cloud keypoints" << std::endl;
            }
        }
    }

}


/**
 Find descriptor correspondences
 */
void Registrator::findCorrespondences() {
    
    pcl::KdTreeFLANN<SHOTDescriptorT> correspondence_search;
    correspondence_search.setInputCloud (s_cloud_descriptors_);
    
    for (size_t i = 0; i < t_cloud_descriptors_->size(); ++i)
    {
        std::vector<int> neighbor_indice (1);
        std::vector<float> neighbor_squared_distance (1);
        if (!pcl_isfinite (t_cloud_descriptors_->at(i).descriptor[0])) //skipping NaNs
        {
            continue;
        }
        int neighbors_found = correspondence_search.nearestKSearch(t_cloud_descriptors_->at(i), 1, neighbor_indice, neighbor_squared_distance);
        if(neighbors_found == 1 && neighbor_squared_distance[0] < 0.25f) //Only accept correspondence if squared distance < 0.25
        {
            pcl::Correspondence corr (neighbor_indice[0], static_cast<int> (i), neighbor_squared_distance[0]);
            correspondences_->push_back (corr);
        }
    }
    
    if(FLAGS_verbose) {
        std::cout << "Found " << correspondences_->size() << " correspondences" << std::endl;
    }
    
}

/**
 Filters correspondences using RANSAC
 */
void Registrator::filterCorrespondences() {
    
    pcl::registration::CorrespondenceRejectorSampleConsensus<PointT> ransac;
    ransac.setInputSource(s_cloud_keypoints_);
    ransac.setInputTarget(t_cloud_keypoints_);
    ransac.setInlierThreshold(consensus_inlier_threshold_);
    ransac.setMaximumIterations(consensus_max_iterations_);
    ransac.setInputCorrespondences(correspondences_);
    ransac.getCorrespondences(*correspondences_);

    if(FLAGS_verbose) {
        std::cout << "Retained " << correspondences_->size() << " correspondences after sample consensus filtering" << std::endl;
    }

    //Create PointClouds containing just valid correspondence points
    for (size_t i = 0; i < correspondences_->size (); ++i)
    {
        s_cloud_corr_->push_back(s_cloud_keypoints_->at(correspondences_->at(i).index_query));
        t_cloud_corr_->push_back(t_cloud_keypoints_->at(correspondences_->at(i).index_match));
    }
}


/**
 Estimates rigid-body transformation based on point correspondences
 */
void Registrator::computeCorrespondenceBasedTransformation() {
    
    pcl::registration::TransformationEstimationSVD<PointT, PointT> transformation_estimation;
    transformation_estimation.estimateRigidTransformation (*s_cloud_keypoints_,
                                           *t_cloud_keypoints_, *correspondences_, correspondence_T_);
    
    if(FLAGS_verbose) {
        std::cout << "Estimated correspondence-based transformation:" << std::endl;
        util::print4x4Matrix (correspondence_T_);
    }
    
}


/**
 Computes rigid-body transformation based on Iterative Closest Point algorithm
 */
void Registrator::ICPBasedRegistration() {
    
    pcl::IterativeClosestPoint<PointT, PointT> icp;
    icp.setMaxCorrespondenceDistance(icp_max_correspondence_distance_);
    icp.setMaximumIterations(icp_max_iterations_);
    icp.setInputSource(s_cloud_);
    icp.setInputTarget(t_cloud_);
    
    if (is_correspondence_matching_complete_) {
        icp.align(*r_cloud_, correspondence_T_);
    
        if(FLAGS_verbose)
            std::cout << "Estimated combined correspondence/ICP-based transformation:" << std::endl;
    }
    else {
        icp.align(*r_cloud_);
        
        if(FLAGS_verbose)
            std::cout << "Estimated ICP-based transformation:" << std::endl;
    }
    
    icp_T_ = icp.getFinalTransformation();
    
    if(FLAGS_verbose)
        util::print4x4Matrix (icp_T_);
    
    is_icp_complete_ = true;
}

/**
 Perform registration, using either correspondence-matching, ICP or both
 */
void Registrator::performRegistration(const std::string registration_technique) {
    
    if (registration_technique.compare("correspondence") == 0 || registration_technique.compare("both") == 0)
        correspondenceBasedRegistration();
    if (registration_technique.compare("icp") == 0 || registration_technique.compare("both") == 0)
        ICPBasedRegistration();
    
    if (is_correspondence_matching_complete_ && !is_icp_complete_)
        pcl::transformPointCloud(*s_cloud_, *r_cloud_, correspondence_T_);
    else if(!is_correspondence_matching_complete_ && is_icp_complete_)
        pcl::transformPointCloud(*s_cloud_, *r_cloud_, icp_T_);
    else if(is_correspondence_matching_complete_ && is_icp_complete_) {
        combined_T_ = icp_T_;
        pcl::transformPointCloud(*s_cloud_, *r_cloud_, combined_T_);
    }
    
    //Save copy of registered point cloud to colorize
    pcl::copyPointCloud(*r_cloud_, *r_cloud_rgb_);
    setRegisteredCloudToDefaultColor();
    
    colormap_residuals_->resize(getRegisteredToTargetResiduals()->size());
    
}

/**
 Computes residuals at each point to the closest point of the other point cloud
 */
void Registrator::computeResiduals() {
    
    t_r_residuals_->resize(t_cloud_->size());
    r_t_residuals_->resize(r_cloud_->size());

    #pragma omp parallel sections
    {
        #pragma omp section
        {
            std::vector<int> neighbor_indice_1 (1);
            std::vector<float> neighbor_squared_distance_1 (1);
            pcl::KdTreeFLANN<PointT> nn_search_1;
            nn_search_1.setInputCloud (r_cloud_);
            t_r_max_residual_ = 0;
            int i_1 = 0;
            double d_1 = 0;
            
            for(PointCloudT::iterator it = t_cloud_->begin(); it != t_cloud_->end(); it++){
                
                nn_search_1.nearestKSearch((PointT)*it, 1, neighbor_indice_1, neighbor_squared_distance_1);
                d_1 = std::sqrt(neighbor_squared_distance_1[0]);
                
                t_r_residuals_->at(i_1++) = d_1;
                
                if (d_1 > t_r_max_residual_)
                    t_r_max_residual_ = d_1;
                
            }
        }
    
        #pragma omp section
        {
            std::vector<int> neighbor_indice_2 (1);
            std::vector<float> neighbor_squared_distance_2 (1);
            pcl::KdTreeFLANN<PointT> nn_search_2;
            nn_search_2.setInputCloud (t_cloud_);
            r_t_max_residual_ = 0;
            int i_2 = 0;
            double d_2 = 0;
            
            for(PointCloudT::iterator it = r_cloud_->begin(); it != r_cloud_->end(); it++){
                
                nn_search_2.nearestKSearch((PointT)*it, 1, neighbor_indice_2, neighbor_squared_distance_2);
                d_2 = std::sqrt(neighbor_squared_distance_2[0]);
                
                r_t_residuals_->at(i_2++) = d_2;
                
                if (d_2 > r_t_max_residual_)
                    r_t_max_residual_ = d_2;
                
            }
        }
    }
    
    if(FLAGS_verbose)
        std::cout << "Computed source -> target cloud residuals" << std::endl;
}


/**
 Computes the F-score of the registration at a given 'threshold'
 */
double Registrator::getFScoreAtThreshold(double threshold) {
    
    if (threshold == -1)
        threshold = residual_threshold_;
    
    if (t_r_residuals_->empty() || r_t_residuals_->empty())
        computeResiduals();
    
    //Precision
    double precision = 0;
    for(std::vector<double>::iterator it = r_t_residuals_->begin(); it != r_t_residuals_->end(); it++)
        precision += (*it < threshold) ? *it : 0;
    precision *= (100.0/r_t_residuals_->size());
    
    //Recall
    double recall = 0;
    for(std::vector<double>::iterator it = t_r_residuals_->begin(); it != t_r_residuals_->end(); it++)
        recall += (*it < threshold) ? *it : 0;
    recall *= (100.0/t_r_residuals_->size());
    
    return (2 * precision * recall) / (precision + recall);

}

/**
 Saves the final transformation matrix to 'filepath'
 */
int Registrator::saveFinalTransform(const std::string filepath) {
    return util::writeMatrixToFile(getCombinedTransformation(), filepath);
}


/**
 Saves the f-score to 'filepath' at the given 'threshold'
 */
void Registrator::saveFScoreAtThreshold(const std::string filepath, double threshold) {

    std::stringstream ss;
    ss << "residual threshold: " << threshold << "\nf-score: " << getFScoreAtThreshold(threshold) << std::endl;
    util::writeStringToFile(filepath, ss.str());
}


/**
 Computes the colormap for the registered point cloud based on the distance 
 of each point in the cloud to the nearest point in the target cloud
 */
void Registrator::computeResidualColormap() {
    
    std::copy(getRegisteredToTargetResiduals()->begin(), getRegisteredToTargetResiduals()->end(), colormap_residuals_->begin());
    
    util::vectorMin(colormap_residuals_, residual_threshold_); //Clip residual values to ensure good color gradient
    util::vectorRescale(colormap_residuals_, residual_threshold_);
    
    {
        double h, s, v, r, g, b;
        s = 1;
        v = 1;
        PointCloudRGBT::iterator it;
        int i;
        for(it = r_cloud_rgb_->begin(), i = 0; it != r_cloud_rgb_->end(); it++, i++) {
            
            h = 240.0 * colormap_residuals_->at(i); //Limit to [0,240] range to end in blue color space
            util::hsv2rgb(r, g, b, h, s, v);
            it->r = 255 * r;
            it->g = 255 * g;
            it->b = 255 * b;
        }
    }
    
    is_residual_colormap_computed_ = true;
    
}


/**
 Saves registered point cloud with residual colormap to disk
 */
int Registrator::saveResidualColormapPointCloud(std::string &filepath) {
    
    if (!is_residual_colormap_computed_)
        computeResidualColormap();
    
    return util::writePointCloudToPLY(filepath, *r_cloud_rgb_);
    
}

void Registrator::setResidualThreshold(double residual_threshold) {
    residual_threshold_ = residual_threshold;
}

double Registrator::getResidualThreshold() {
    return residual_threshold_;
}

void Registrator::setRegisteredCloudToDefaultColor() {
    
    for(PointCloudRGBT::iterator it = r_cloud_rgb_->begin(); it != r_cloud_rgb_->end(); it++){
        it->r = r_color_.r;
        it->g = r_color_.g;
        it->b = r_color_.b;
    }
    
}

