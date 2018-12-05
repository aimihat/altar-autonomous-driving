cmake_minimum_required(VERSION 3.10.0)

project(altar_object_detection)

include_directories(${OpenCV_INCLUDE_DIRS} darknet/include darknet/arapaho darknet/src /usr/local/cuda/include/ /usr/local/cudNN/include/x86_64-linux-gnu/)


set (SOURCEFILES    	darknet/arapaho/stdafx.h
        darknet/arapaho/AltarObjectDetection.h
        darknet/arapaho/AltarObjectDetection.cpp
        darknet/arapaho/arapaho.cpp
        darknet/arapaho/arapaho.hpp)

adtf_add_filter(${PROJECT_NAME}
				${SOURCEFILES}
)

# Specify where it should be installed to
adtf_install_target(${PROJECT_NAME} bin)


adtf_create_plugindescription(
    TARGET
        ${PROJECT_NAME}
    PLUGIN_SUBDIR
        "bin"
 )

link_directories(/usr/local/cuda/lib64)
target_link_libraries(${PROJECT_NAME} LINK_PUBLIC ${OpenCV_LIBS})
target_link_libraries(${PROJECT_NAME} LINK_PUBLIC cuda cudart cublas curand cudnn)
target_link_libraries(${PROJECT_NAME} LINK_PUBLIC /home/aadc/audi/AADC/src/aadcUser/AltarObjectDetection/darknet/arapaho/libdarknet-cpp-shared.so)

#this is to add a folder within the ADTF solution
set_property(TARGET ${PROJECT_NAME} PROPERTY FOLDER user)
set_property(TARGET ${PROJECT_NAME}_pdgen PROPERTY FOLDER user)
