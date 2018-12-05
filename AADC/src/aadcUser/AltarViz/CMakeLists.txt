
# Mixed template CMAKE and CMAKE File found in ADTF demo folder

cmake_minimum_required(VERSION 3.10.0)
include_directories(${OpenCV_INCLUDE_DIRS})

project(AltarViz)

if (NOT TARGET adtf::ui)
	find_package(ADTF COMPONENTS filtersdk ui)
endif()

adtf_use_qt(Widgets)

set (SOURCEFILES    	stdafx.h
		Viz.h
		Viz.cpp)

adtf_add_filter(${PROJECT_NAME}
		${SOURCEFILES}
		)

target_link_libraries(${PROJECT_NAME} PUBLIC adtf::ui Qt5::Widgets)

adtf_install_target(${PROJECT_NAME} bin)

adtf_disable_qt_warnings(${PROJECT_NAME})

adtf_install_target(${PROJECT_NAME} bin)

adtf_create_plugindescription(
		TARGET
		${PROJECT_NAME}
		PLUGIN_SUBDIR
		"bin"
		DEPENDENT_PLUGINS
		"adtf_xsystem"
)

#this is to add a folder within the ADTF solution
set_property(TARGET ${PROJECT_NAME} PROPERTY FOLDER user)
set_property(TARGET ${PROJECT_NAME}_pdgen PROPERTY FOLDER user)



