cmake_minimum_required(VERSION 2.6 FATAL_ERROR)
project(3D_REGISTRATION)
#find pcl libraries
find_package(PCL 1.4 REQUIRED)
#include files
include_directories(${PCL_INCLUDE_DIRS})
#lib files
link_directories(${PCL_LIBRARY_DIRS})
#definitions
add_definitions(${PCL_DEFINITIONS})

#necessary libs
set(pcl_libs
    ${PCL_COMMON_LIBRARIES} 
    ${PCL_IO_LIBRARIES} 
    ${PCL_KD_TREE_LIBRARY} 
    ${PCL_REGISTRATION_LIBRARY}
    ${PCL_SEARCH_LIBRARY}
    ${PCL_FEATURES_LIBRARY}
    ${PCL_VISUALIZATION_LIBRARY}
 )


################################
#Alignment library
set(alignment_sources
    alignment.h                 alignment.cxx
    correspondence_procrustes.h correspondence_procrustes.hpp
    sac_non_rigid.h             sac_non_rigid.hpp
    util.h                      util.cxx
    )
add_library(pcl_alignment ${alignment_sources})
target_link_libraries(pcl_alignment ${pcl_libs})

################################
#Executables
add_executable(align align_exe.cxx)
target_link_libraries(align pcl_alignment ${pcl_libs})

add_executable(corrs save_correspondences.cxx)
target_link_libraries(corrs pcl_alignment ${pcl_libs})

add_executable(visualize visualize.cxx)
target_link_libraries(visualize pcl_alignment ${pcl_libs})

add_executable(compare compare_correspondences.cxx)
target_link_libraries(compare pcl_alignment ${pcl_libs})

add_executable(bbox bbox.cxx)
target_link_libraries(bbox pcl_alignment ${pcl_libs})

add_executable(crop cropper.cxx)
target_link_libraries(crop pcl_alignment ${pcl_libs})

