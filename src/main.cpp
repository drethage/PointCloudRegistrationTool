/*
 Point Cloud Registration Tool
 
 BSD 2-Clause License
 Copyright (c) 2017, Dario Rethage
 See LICENSE at package root for full license
 */

#ifdef _OPENMP
#include <omp.h>
#endif

// Standard
#include <iostream>
#include <iterator>
#include <algorithm>
#include <string>
#include <vector>

// Boost
#include <boost/program_options.hpp>

// PRT
#include "util.hpp"
#include "types.hpp"
#include "Registrator.hpp"
#include "Visualizer.hpp"

using namespace PRT;

int main (int argc, char* argv[]) {
    
    boost::program_options::options_description desc("Supported Options");
    desc.add_options()
        ("help,h", "program description")
        ("gui,g", boost::program_options::bool_switch()->default_value(false), "run with GUI or headless mode")
        ("batch_file,b", boost::program_options::value<std::string>(), "path to batch processing file")
        ("verbose,v", boost::program_options::bool_switch()->default_value(false), "enable verbosity")
        ("input-files", boost::program_options::value< std::vector<std::string> >(), "two input file paths (in order): target_point_cloud source_point_cloud")
        ("registration_technique,r", boost::program_options::value< std::string >()->default_value("both"), "what kind of registration technique to use, 'correspondence', 'icp', 'both'")
        ("residual_threshold,t", boost::program_options::value< double >()->default_value(0.1), "(in meters) max residual used in calculation of f-score, corresponds to blue in the residual colormap")
        ("num_ksearch_neighbors,n", boost::program_options::value< int >()->default_value(100), "number of neighbors to consider in normals computation")
        ("descriptor_radius,d", boost::program_options::value< double >()->default_value(1.0), "(in meters) radius of surrounding points to consider during descriptor computation")
        ("subsampling_radius,s", boost::program_options::value< double >()->default_value(0.2), "(in meters) determines the degree of subsampling used, all points within a sphere of the specified radius are replaced by their centroid")
        ("consensus_inlier_threshold,i", boost::program_options::value< double >()->default_value(0.2), "(in meters) maximum distance a point can lie from a hypothesis during sample consensus to be considered an inlier")
        ("consensus_max_iterations", boost::program_options::value< int >()->default_value(100), "maximum number of iterations to perform sample consensus")
        ("icp_max_iterations", boost::program_options::value< int >()->default_value(100), "maximum number of iterations to perform ICP")
        ("icp_max_correspondence_distance", boost::program_options::value< double >()->default_value(0.05), "correspondences greater than this distance apart will be ignored in ICP")
        ("transformation_matrix_filepath", boost::program_options::value<std::string>(), "filepath to output transformation matrix CSV")
        ("registered_pointcloud_filepath", boost::program_options::value<std::string>(), "filepath to output registered point cloud PLY")
        ("residual_histogram_image_filepath", boost::program_options::value<std::string>(), "filepath to output histogram PNG")
        ("fscore_filepath", boost::program_options::value<std::string>(), "filepath to output f-score TXT")
    
    #ifdef _OPENMP
        ("no_parallel", boost::program_options::bool_switch()->default_value(false), "disable parallel execution")
    #endif
    ;
    
    boost::program_options::positional_options_description pos_desc;
    pos_desc.add("input-files", 2);
    
    boost::program_options::variables_map vm;
    boost::program_options::store(boost::program_options::command_line_parser(argc, argv).options(desc).positional(pos_desc).run(), vm);
    boost::program_options::notify(vm);
    
    if (vm.count("help")) {
        std::cout << desc << std::endl;
        return 0;
    }
    
    pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS);    
    util::verbose = vm["verbose"].as< bool >();
    
    bool gui = vm["gui"].as<bool>();
    
    FilepairVectorPtr filepairs;
    if (vm.count("batch_file")) {
        
        filepairs = util::readBatchProcessingFile(vm["batch_file"].as< std::string >());
        
        if(filepairs->empty()) {
            std::cout << "Invalid batch file path, exiting." << std::endl;
            return -1;
        }
        
        gui = false;
        
    } else if (vm.count("input-files") && vm["input-files"].as< std::vector< std::string > >().size() == 2) {
        
        StringVector input_files = vm["input-files"].as< std::vector< std::string > >();
        filepair_t pair;
        pair.targetfile = input_files[0];
        pair.sourcefile = input_files[1];
        filepairs = FilepairVectorPtr(new FilepairVector());
        filepairs->push_back(pair);
        
    } else {
        std::cout << "Usage:" << std::endl;
        std::cout << "1. [target_point_cloud] [source_point_cloud]" << std::endl;
        std::cout << "2. --batch_file [batch_filepath]" << std::endl;
        std::cout << std::endl;
        std::cout << "See --help for optional arguments" << std::endl;
        return 0;
    }
    
    #ifdef _OPENMP
    omp_set_nested(true);
    if (vm["no_parallel"].as< bool >())
        omp_set_num_threads(1);
    else
        if (util::verbose)
            std::cout << "Executing in parallel" << std::endl;
    #endif
    
    #pragma omp parallel for
    for(FilepairVector::size_type i = 0; i < filepairs->size(); i++) {
        
        PointCloudT::Ptr target_cloud (new PointCloudT);
        target_cloud->clear();
        std::string target_cloud_filepath = filepairs->at(i).targetfile;
        
        PointCloudT::Ptr source_cloud (new PointCloudT);
        source_cloud->clear();
        std::string source_cloud_filepath = filepairs->at(i).sourcefile;
        bool reads_successful = true;
        
        #pragma omp parallel sections
        {
            #pragma omp section
            {
                if (util::loadPointCloud(target_cloud_filepath, *target_cloud) != 0) {
                    std::cout << "Failed to load: " << target_cloud_filepath << std::endl;
                    std::cout << "Skipping.." << std::endl;
                    reads_successful = false;
                }
            }
            
            #pragma omp section
            {
                if (util::loadPointCloud(source_cloud_filepath, *source_cloud) != 0) {
                    std::cout << "Failed to load: " << source_cloud_filepath << std::endl;
                    std::cout << "Skipping.." << std::endl;
                    reads_successful = false;
                }
            }
        }
        
        if(!reads_successful)
            continue;
        
        //Extract prefix for output files
        std::string prefix = util::removeFileExtension(filepairs->at(i).sourcefile);
        
        //Set output file paths
        std::string transformation_matrix_filepath;
        std::string registered_pointcloud_filepath;
        std::string residual_histogram_image_filepath;
        std::string fscore_filepath;
        
        if (vm.count("transformation_matrix_filepath"))
            transformation_matrix_filepath = vm["transformation_matrix_filepath"].as<std::string>();
        else
            transformation_matrix_filepath = prefix + "_transform.csv";
    
        if (vm.count("registered_pointcloud_filepath"))
            registered_pointcloud_filepath = vm["registered_pointcloud_filepath"].as<std::string>();
        else
            registered_pointcloud_filepath = prefix + "_registered.ply";
        
        if (vm.count("residual_histogram_image_filepath"))
            residual_histogram_image_filepath = vm["residual_histogram_image_filepath"].as<std::string>();
        else
            residual_histogram_image_filepath = prefix + "_residual_histogram.png";
        
        if (vm.count("fscore_filepath"))
            fscore_filepath = vm["fscore_filepath"].as<std::string>();
        else
            fscore_filepath = prefix + "_fscore.txt";
    
        //Ensure unique output filepaths
        if (util::ensureUniqueFilepath(transformation_matrix_filepath) != 0) {
            std::cout << "Failed to create transformation matrix file." << std::endl;
            continue;
        }
        
        if (util::ensureUniqueFilepath(fscore_filepath) != 0) {
            std::cout << "Failed to create f-score file." << std::endl;
            continue;
        }
        
        if (util::ensureUniqueFilepath(registered_pointcloud_filepath) != 0) {
            std::cout << "Failed to create registered PLY file." << std::endl;
            continue;
        }
        
        if (util::ensureUniqueFilepath(residual_histogram_image_filepath) != 0) {
            std::cout << "Failed to create residual histogram file." << std::endl;
            continue;
        }
        
        //Registration
        //Setup
        Registrator::Ptr registrator (new Registrator());
        registrator->setNumKSearchNeighbors(vm["num_ksearch_neighbors"].as<int>());
        registrator->setDescriptorRadius(vm["descriptor_radius"].as<double>());
        registrator->setSubsamplingRadius(vm["subsampling_radius"].as<double>());
        registrator->setConsensusInlierThreshold(vm["consensus_inlier_threshold"].as<double>());
        registrator->setConsensusMaxIterations(vm["consensus_max_iterations"].as<int>());
        registrator->setICPMaxIterations(vm["icp_max_iterations"].as<int>());
        registrator->setICPMaxCorrespondenceDistance(vm["icp_max_correspondence_distance"].as<double>());
        registrator->setResidualThreshold(vm["residual_threshold"].as<double>());
        registrator->setTargetCloud(target_cloud);
        registrator->setSourceCloud(source_cloud);
        
        //Compute
        registrator->performRegistration(vm["registration_technique"].as<std::string>());
        
        //Save Results
        registrator->saveResidualColormapPointCloud(registered_pointcloud_filepath);
        registrator->saveFinalTransform(transformation_matrix_filepath);
        registrator->saveFScoreAtThreshold(fscore_filepath, vm["residual_threshold"].as<double>());
        
        std::cout << "Registration of " << source_cloud_filepath << " finished" << std::endl;
        std::cout << "F-score: " << registrator->getFScoreAtThreshold() << std::endl;
        std::cout << "Saved to: " << registered_pointcloud_filepath << std::endl;
        
        if (!gui)
            continue;
        
        //Visualization
        Visualizer::Ptr visualizer (new Visualizer("Point Cloud Registration Tool"));
        visualizer->setRegistrator(registrator);
        visualizer->saveHistogramImage(residual_histogram_image_filepath);
        visualizer->visualize();
        
    }

    return 0;
}
