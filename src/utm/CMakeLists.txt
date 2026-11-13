add_library(utm STATIC utm.cpp)
target_link_libraries(utm PUBLIC localization_core_headers rmc time_and_location)
