add_artery_feature(cluster ClusterLaneService.cc ClusteringService.cc)
generate_opp_message(Clustering.msg TARGET cluster DIRECTORY cluster_msgs)
add_opp_run(highway_cluster NED_FOLDERS ${CMAKE_CURRENT_SOURCE_DIR})