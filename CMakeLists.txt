
cmake_minimum_required(VERSION 2.8.3)
project( ndt_localizer )

set( CMAKE_CXX_STANDARD 14 )
set( CMAKE_BUILD_TYPE "Release" )

set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O2 -g -Wall" )



find_package( catkin REQUIRED COMPONENTS
        roscpp
        tf2
        tf2_ros
        tf2_geometry_msgs
        std_msgs
        sensor_msgs
        diagnostic_msgs
        pcl_conversions
        pcl_ros
        message_generation
        )

generate_messages(
        DEPENDENCIES
        std_msgs
)

find_package(PCL 1.8 REQUIRED)

# find_package(PCL 1.8 REQUIRED QUIET)
# 如果指定了QUIET选项，那么表示：就算未找到该包的配置文件，也不会报错。
# 相反地，如果指定了REQUIRED选项：如果未找到文件，会报错。

catkin_package(
        INCLUDE_DIRS include
        CATKIN_DEPENDS
        roscpp
        tf2
        tf2_ros
        tf2_geometry_msgs
        geometry_msgs
        sensor_msgs
        nav_msgs
        diagnostic_msgs
        pcl_conversions
        pcl_ros
        message_generation
)







# diagnostics意为诊断，该模块旨在从系统中收集各种状态信息和硬件信息，
# 以进行分析，故障排除和记录。 工具链中包含用于收集，发布，分析和查看diagnostics数据的工具。

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(voxel_grid_filter nodes/voxel_grid_filter.cpp)       # 可执行文件
add_dependencies(voxel_grid_filter ${catkin_EXPORTED_TARGETS})      # 添加依赖
target_link_libraries(voxel_grid_filter ${catkin_LIBRARIES})        # 添加库文件


add_executable(map_loader nodes/mapLoader.cpp)
target_link_libraries(map_loader ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(ndt_localizer_node nodes/ndt.cpp)
target_link_libraries(ndt_localizer_node ${catkin_LIBRARIES} ${PCL_LIBRARIES})


