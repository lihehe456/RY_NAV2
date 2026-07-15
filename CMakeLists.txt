cmake_minimum_required(VERSION 3.5)
project(nav2_route_polyline_planner)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_gtest REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_common REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_navfn_planner REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(tf2_ros REQUIRED)

nav2_package()
add_compile_options(-O3)

include_directories(include)

set(library_name ${PROJECT_NAME})

set(dependencies
  builtin_interfaces
  geometry_msgs
  nav2_common
  nav2_core
  nav2_costmap_2d
  nav2_navfn_planner
  nav2_util
  nav_msgs
  pluginlib
  rclcpp
  rclcpp_lifecycle
  tf2_ros
)

add_library(${library_name} SHARED
  src/route_clearance_narrow_planner.cpp
  src/route_clearance_planner.cpp
  src/route_clearance_planner_core.cpp
  src/route_polyline_planner.cpp
  src/route_polyline_planner_core.cpp
  src/route_sparse_curve_planner.cpp
  src/route_sparse_curve_planner_core.cpp
)

ament_target_dependencies(${library_name} ${dependencies})

target_compile_definitions(${library_name} PUBLIC "PLUGINLIB_DISABLE_BOOST_FUNCTIONS")

pluginlib_export_plugin_description_file(nav2_core global_planner_plugin.xml)

install(TARGETS ${library_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(FILES global_planner_plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(gtest_disable_pthreads OFF)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(test_route_polyline_planner_core
    test/test_route_polyline_planner_core.cpp
  )
  ament_target_dependencies(test_route_polyline_planner_core ${dependencies})
  target_link_libraries(test_route_polyline_planner_core ${library_name})

  ament_add_gtest(test_route_sparse_curve_planner_core
    test/test_route_sparse_curve_planner_core.cpp
  )
  ament_target_dependencies(test_route_sparse_curve_planner_core ${dependencies})
  target_link_libraries(test_route_sparse_curve_planner_core ${library_name})

  ament_add_gtest(test_route_clearance_planner_core
    test/test_route_clearance_planner_core.cpp
  )
  ament_target_dependencies(test_route_clearance_planner_core ${dependencies})
  target_link_libraries(test_route_clearance_planner_core ${library_name})

  ament_add_gtest(test_planner_plugin_registration
    test/test_planner_plugin_registration.cpp
  )
  ament_target_dependencies(test_planner_plugin_registration ${dependencies})
  target_link_libraries(test_planner_plugin_registration ${library_name})
endif()

ament_export_include_directories(include)
ament_export_libraries(${library_name})
ament_export_dependencies(${dependencies})
ament_package()
