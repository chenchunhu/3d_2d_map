cmake_minimum_required(VERSION 3.10)
project(3d_2d_map)

set(CMAKE_CXX_STANDARD 11)

find_package(PCL 1.8 REQUIRED)
find_package(Boost REQUIRED filesystem program_options)

include_directories(
        include
        ${PCL_INCLUDE_DIRS}
        ${Boost_INCLUDE_DIRS}
)



#add_definitions(-DBOOST_ERROR_CODE_HEADER_ONLY)

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
set(PCL_INCLUDE_DIRS /usr/local/include/pcl-1.8)

add_executable(3d_2d_map
          srcc/main.cpp
        srcc/pcl2dmap.cpp
        srcc/message_map.pb.cc
        srcc/message_gnss.pb.cc
        srcc/message_header.pb.cc
        srcc/message_imu.pb.cc
        srcc/filter.cpp)


#aux_source_directory(.. DIR_SRCCS)
#add_executable(3d_2d_map ${DIR_SRCCS})
#add_library(3d_2d_map ${DIR_SRCCS})


target_link_libraries(3d_2d_map
        #/home/cch/3d_2d_map/thirdparty/proto_msg/build/libproto_msg.so
        ${PCL_LIBRARIES}
       #${Boost_LIBRARIES}
        #libproto_msg.so
        libprotobuf.so
        )
