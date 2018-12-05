cmake_minimum_required(VERSION 3.10.0)

project(altar_driver_module)

find_package(ADTF COMPONENTS filtersdk)

INCLUDE_DIRECTORIES( ${Boost_INCLUDE_DIR} )



set (HEADERS    stdafx.h
					    DriverModuleFilter.h)
set (SOURCES
					  DriverModuleFilter.cpp)

#uncomment if CMAKE_AUTOMOC/UIC is disabled
#ui/MainWindow.ui 
#set (UI_SOURCE car_controller.ui)

adtf_add_filter(${PROJECT_NAME}
				${HEADERS}
#				${UI_SOURCE}
				${SOURCES}
				)

set_property(TARGET ${PROJECT_NAME} PROPERTY AUTOGEN_TARGET_DEPENDS ${HEADERS})

target_link_libraries(${PROJECT_NAME} PUBLIC ${Boost_LIBRARIES} )
target_include_directories(${PROJECT_NAME} PUBLIC .)

#install the program you need    
adtf_install_target(${PROJECT_NAME} bin)

adtf_create_plugindescription(
    TARGET
        ${PROJECT_NAME}
    PLUGIN_SUBDIR
        "bin"
    DEPENDENT_PLUGINS
        "adtf_xsystem")

#this is to add a folder within the ADTF solution
set_property(TARGET ${PROJECT_NAME} PROPERTY FOLDER jury)
set_property(TARGET ${PROJECT_NAME}_pdgen PROPERTY FOLDER jury)
