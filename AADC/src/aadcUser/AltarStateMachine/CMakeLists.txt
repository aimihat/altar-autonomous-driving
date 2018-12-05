cmake_minimum_required(VERSION 3.10.0)

project(altar_state_machine)

include_directories(${OpenCV_INCLUDE_DIRS} ${Boost_INCLUDE_DIR} sml-1.0.1/include/ ../AltarUtils/)

set (Boost_USE_STATIC_LIBS OFF CACHE BOOL "use static libraries from Boost")
set (Boost_USE_MULTITHREADED ON)
find_package(Boost 1.66 REQUIRED COMPONENTS
		system filesystem thread chrono date_time atomic timer)

set (SOURCEFILES    	stdafx.h
        AltarStateMachine.h
        AltarStateMachine.cpp
		sml-1.0.1/include/boost/sml.hpp
                DecisionMaker.h
		)

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

#Use link LINK_PUBLIC for linking custom libraries
target_link_libraries(${PROJECT_NAME} LINK_PUBLIC ${Boost_LIBRARIES} )

#this is to add a folder within the ADTF solution
set_property(TARGET ${PROJECT_NAME} PROPERTY FOLDER user)
set_property(TARGET ${PROJECT_NAME}_pdgen PROPERTY FOLDER user)

target_compile_features(${PROJECT_NAME} PRIVATE cxx_std_14)
