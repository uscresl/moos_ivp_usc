# add each bhv directory that needs to be built, 
# each dir should have proper CMakeLists.txt to build actual bhv
add_subdirectory(BHV_OpRegionBounce)
add_subdirectory(BHV_WaypointSpd)

# from Henrik Schmidt
add_subdirectory(BHV_YoYo)
