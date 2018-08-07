# Copyright: 2016 iCub Facility, Istituto Italiano di Tecnologia
# Author: Vadim Tikhanoff
# CopyPolicy: Released under the terms of the GNU GPL v2.0.
#
CMAKE_MINIMUM_REQUIRED(VERSION 3.5)
project(yarpOpenFace)

find_package(YARP 3.1 REQUIRED COMPONENTS OS sig dev math rosmsg)
find_package(ICUB REQUIRED)
find_package(ICUBcontrib)
#find_package(dlib)
find_package(CER REQUIRED)

list(APPEND CMAKE_MODULE_PATH ${YARP_MODULE_PATH})
list(APPEND CMAKE_MODULE_PATH ${ICUB_MODULE_PATH})
list(APPEND CMAKE_MODULE_PATH ${ICUBCONTRIB_MODULE_PATH})

#find_package(OpenCV)

##########################################################

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin/)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/yarpOpenFace/cmake/modules/")

set(CMAKE_CONFIG_DIR etc/OpenFace)
set(CONFIG_DIR "${CMAKE_INSTALL_PREFIX}/${CMAKE_CONFIG_DIR}")
add_definitions(-DCONFIG_DIR="${CONFIG_DIR}")


find_package( BLAS REQUIRED )
include_directories( ${BLAS_INCLUDE_DIRS} )
LINK_DIRECTORIES(${BLAS_LIBRARY_DIRS})

find_package( OpenCV 3.3 REQUIRED )

MESSAGE("OpenCV information:") 
MESSAGE("  OpenCV_INCLUDE_DIRS: ${OpenCV_INCLUDE_DIRS}") 
MESSAGE("  OpenCV_LIBRARIES: ${OpenCV_LIBRARIES}") 
MESSAGE("  OpenCV_LIBRARY_DIRS: ${OpenCV_LIBRARY_DIRS}") 
include_directories( ${OpenCV_INCLUDE_DIRS} )

find_package( Boost 1.5.9 REQUIRED COMPONENTS filesystem system)
MESSAGE("Boost information:") 
MESSAGE("  Boost_INCLUDE_DIRS: ${Boost_INCLUDE_DIRS}") 
MESSAGE("  Boost_LIBRARIES: ${Boost_LIBRARIES}") 
MESSAGE("  Boost_LIBRARY_DIRS: ${Boost_LIBRARY_DIRS}") 

INCLUDE_DIRECTORIES(${Boost_INCLUDE_DIRS})
INCLUDE_DIRECTORIES(${Boost_INCLUDE_DIRS}/boost)
LINK_DIRECTORIES(${Boost_LIBRARY_DIRS})	

# Try finding TBB in default location
find_package( TBB REQUIRED )

# If not found, use FindTBB.cmake
if ("${TBB_LIBRARIES}" STREQUAL "")
	MESSAGE("TBB not found in CONFIG, searching with FindTBB.cmake.") 
    find_package( TBB REQUIRED )
	if ("${TBB_LIBRARIES}" STREQUAL "")
		MESSAGE(FATAL_ERROR "TBB not found")
	endif()
else()
	MESSAGE("TBB found in CONFIG: ${TBB_LIBRARIES}") 
endif()

MESSAGE("TBB information:")
MESSAGE("  TBB_VERSION: ${TBB_VERSION}")
MESSAGE("  TBB_INCLUDE_DIRS: ${TBB_INCLUDE_DIRS}")
MESSAGE("  TBB_LIBRARIES: ${TBB_LIBRARIES}")


# Move LandmarkDetector model
file(GLOB files "lib/local/LandmarkDetector/model/*.txt")
foreach(file ${files})
	if (MSVC)
		file(COPY ${file} DESTINATION ${CMAKE_BINARY_DIR}/bin/Debug/model)
		file(COPY ${file} DESTINATION ${CMAKE_BINARY_DIR}/bin/Release/model)
	else(MSVC)
		file(COPY ${file} DESTINATION ${CMAKE_BINARY_DIR}/bin/model)
	endif(MSVC)

	install(FILES ${file} DESTINATION ${CMAKE_CONFIG_DIR}/model)
endforeach()

# Move the hierarchical LandmarkDetector models
file(GLOB files "lib/local/LandmarkDetector/model/model*")
foreach(file ${files})
	if (MSVC)
		file(COPY ${file} DESTINATION ${CMAKE_BINARY_DIR}/bin/Debug/model)
		file(COPY ${file} DESTINATION ${CMAKE_BINARY_DIR}/bin/Release/model)
	else(MSVC)
		file(COPY ${file} DESTINATION ${CMAKE_BINARY_DIR}/bin/model)
	endif(MSVC)

	install(DIRECTORY ${file} DESTINATION ${CMAKE_CONFIG_DIR}/model)
endforeach()

file(GLOB files "lib/local/LandmarkDetector/model/detection_validation/*.txt")
foreach(file ${files})
	if (MSVC)
		file(COPY ${file} DESTINATION ${CMAKE_BINARY_DIR}/bin/Debug/model/detection_validation)
		file(COPY ${file} DESTINATION ${CMAKE_BINARY_DIR}/bin/Release/model/detection_validation)
	else(MSVC)
		file(COPY ${file} DESTINATION ${CMAKE_BINARY_DIR}/bin/model/detection_validation)
	endif(MSVC)

	install(FILES ${file} DESTINATION ${CMAKE_CONFIG_DIR}/model/detection_validation)
endforeach()

file(GLOB files "lib/local/LandmarkDetector/model/patch_experts/*.txt")
foreach(file ${files})
	if (MSVC)
		file(COPY ${file} DESTINATION ${CMAKE_BINARY_DIR}/bin/Debug/model/patch_experts)
		file(COPY ${file} DESTINATION ${CMAKE_BINARY_DIR}/bin/Release/model/patch_experts)
	else(MSVC)
		file(COPY ${file} DESTINATION ${CMAKE_BINARY_DIR}/bin/model/patch_experts)
	endif(MSVC)

	install(FILES ${file} DESTINATION ${CMAKE_CONFIG_DIR}/model/patch_experts)
endforeach()

file(GLOB files "lib/local/LandmarkDetector/model/patch_experts/*.dat")
foreach(file ${files})
	if (MSVC)
		file(COPY ${file} DESTINATION ${CMAKE_BINARY_DIR}/bin/Debug/model/patch_experts)
		file(COPY ${file} DESTINATION ${CMAKE_BINARY_DIR}/bin/Release/model/patch_experts)
	else(MSVC)
		file(COPY ${file} DESTINATION ${CMAKE_BINARY_DIR}/bin/model/patch_experts)
	endif(MSVC)

	install(FILES ${file} DESTINATION ${CMAKE_CONFIG_DIR}/model/patch_experts)
endforeach()

file(GLOB files "lib/local/LandmarkDetector/model/mtcnn_detector/*.txt")
foreach(file ${files})
	if (MSVC)
		file(COPY ${file} DESTINATION ${CMAKE_BINARY_DIR}/bin/Debug/model/mtcnn_detector)
		file(COPY ${file} DESTINATION ${CMAKE_BINARY_DIR}/bin/Release/model/mtcnn_detector)
	else(MSVC)
		file(COPY ${file} DESTINATION ${CMAKE_BINARY_DIR}/bin/model/mtcnn_detector)
	endif(MSVC)

	install(FILES ${file} DESTINATION ${CMAKE_CONFIG_DIR}/model/mtcnn_detector)
endforeach()

file(GLOB files "lib/local/LandmarkDetector/model/mtcnn_detector/*.dat")
foreach(file ${files})
	if (MSVC)
		file(COPY ${file} DESTINATION ${CMAKE_BINARY_DIR}/bin/Debug/model/mtcnn_detector)
		file(COPY ${file} DESTINATION ${CMAKE_BINARY_DIR}/bin/Release/model/mtcnn_detector)
	else(MSVC)
		file(COPY ${file} DESTINATION ${CMAKE_BINARY_DIR}/bin/model/mtcnn_detector)
	endif(MSVC)

	install(FILES ${file} DESTINATION ${CMAKE_CONFIG_DIR}/model/mtcnn_detector)
endforeach()

file(GLOB files "lib/local/LandmarkDetector/model/pdms/*.txt")
foreach(file ${files})
	if (MSVC)
		file(COPY ${file} DESTINATION ${CMAKE_BINARY_DIR}/bin/Debug/model/pdms)
		file(COPY ${file} DESTINATION ${CMAKE_BINARY_DIR}/bin/Release/model/pdms)
	else(MSVC)
		file(COPY ${file} DESTINATION ${CMAKE_BINARY_DIR}/bin/model/pdms)
	endif(MSVC)

	install(FILES ${file} DESTINATION ${CMAKE_CONFIG_DIR}/model/pdms)
endforeach()

# Move OpenCV classifiers
file(GLOB files "lib/3rdParty/OpenCV3.1/classifiers/*.xml")
foreach(file ${files})
	if (MSVC)
		file(COPY ${file} DESTINATION ${CMAKE_BINARY_DIR}/bin/Debug/classifiers)
		file(COPY ${file} DESTINATION ${CMAKE_BINARY_DIR}/bin/Release/classifiers)
	else(MSVC)
		file(COPY ${file} DESTINATION ${CMAKE_BINARY_DIR}/bin/classifiers)
	endif(MSVC)

	install(FILES ${file} DESTINATION ${CMAKE_CONFIG_DIR}/classifiers)
endforeach()

# Move AU prediction modules
file(GLOB files "lib/local/FaceAnalyser/AU_predictors/*.txt")
foreach(file ${files})
	if (MSVC)
		file(COPY ${file} DESTINATION ${CMAKE_BINARY_DIR}/bin/Debug/AU_predictors)
		file(COPY ${file} DESTINATION ${CMAKE_BINARY_DIR}/bin/Release/AU_predictors)
	else(MSVC)
		file(COPY ${file} DESTINATION ${CMAKE_BINARY_DIR}/bin/AU_predictors)
	endif(MSVC)

	install(FILES ${file} DESTINATION ${CMAKE_CONFIG_DIR}/AU_predictors)
endforeach()

# Move AU prediction modules
file(GLOB files "lib/local/FaceAnalyser/AU_predictors/svr*")
foreach(file ${files})
	if (MSVC)
		file(COPY ${file} DESTINATION ${CMAKE_BINARY_DIR}/bin/Debug/AU_predictors)
		file(COPY ${file} DESTINATION ${CMAKE_BINARY_DIR}/bin/Release/AU_predictors)
	else(MSVC)
		file(COPY ${file} DESTINATION ${CMAKE_BINARY_DIR}/bin/AU_predictors)
	endif(MSVC)

	install(DIRECTORY ${file} DESTINATION ${CMAKE_CONFIG_DIR}/AU_predictors)
endforeach()

# Move AU prediction modules
file(GLOB files "lib/local/FaceAnalyser/AU_predictors/svm*")
foreach(file ${files})
	if (MSVC)
		file(COPY ${file} DESTINATION ${CMAKE_BINARY_DIR}/bin/Debug/AU_predictors)
		file(COPY ${file} DESTINATION ${CMAKE_BINARY_DIR}/bin/Release/AU_predictors)
	else(MSVC)
		file(COPY ${file} DESTINATION ${CMAKE_BINARY_DIR}/bin/AU_predictors)
	endif(MSVC)

	install(DIRECTORY ${file} DESTINATION ${CMAKE_CONFIG_DIR}/AU_predictors)
endforeach()

if (${CMAKE_CXX_COMPILER_ID} STREQUAL "GNU")
    execute_process(COMMAND ${CMAKE_CXX_COMPILER} -dumpversion OUTPUT_VARIABLE GCC_VERSION)
    if (GCC_VERSION VERSION_LESS 4.7)
        set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -msse -msse2 -msse3")
    else ()
        set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -msse -msse2 -msse3")
    endif ()
else ()
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -msse -msse2 -msse3")
endif ()

# Boost
if(WIN32)
	include_directories(lib/3rdParty/boost)
	include_directories(lib/3rdParty/boost/boost)
	link_directories( ${PROJECT_SOURCE_DIR}/lib/3rdParty/boost/lib )
else()
	INCLUDE_DIRECTORIES(${BOOST_INCLUDE_DIR})
endif()

# dlib
include_directories(lib/3rdParty/dlib/include)

# dlib library

add_subdirectory(lib/3rdParty/dlib)

# LandmarkDetector library
add_subdirectory(lib/local/LandmarkDetector)
# Facial Expression analysis library
add_subdirectory(lib/local/FaceAnalyser)
# Gaze estimation library
add_subdirectory(lib/local/GazeAnalyser)
# Utilities library
add_subdirectory(lib/local/Utilities)


##########################################################

#adding c++11 flags
set(CMAKE_CXX_FLAGS "-std=c++11")

include(YarpInstallationHelpers)
include(ICUBcontribHelpers)
include(ICUBcontribOptions)
icubcontrib_set_default_prefix()

include(YarpIDL)
yarp_add_idl(IDL_GEN_FILES ${PROJECT_NAME}.thrift)

file(GLOB source src/*.cpp)
file(GLOB header include/*.h)
set(idl ${PROJECT_NAME}.thrift)
set(doc ${PROJECT_NAME}.xml)

source_group("Source Files" FILES ${source})
source_group("Header Files" FILES ${header})
#source_group("Source Files" FILES ${DLIB_SRC})
source_group("IDL Files" FILES ${idl})
source_group("DOC Files" FILES ${doc})

include_directories(${PROJECT_SOURCE_DIR}/include
                    #${OpenCV_INCLUDE_DIRS}
                    ${YARP_INCLUDE_DIRS}
                    ${ICUB_INCLUDE_DIRS}
                    #${dlib_INCLUDE_DIRS}
                    ${CER_INCLUDE_DIRS}
                    )

#################################################
#TBB library
include_directories(${TBB_ROOT_DIR}/include)

#OpenBlas library
include_directories(${BLAS_ROOT_DIR})
include_directories(lib/3rdParty/OpenBLAS/include)

# Local libraries
include_directories(${LandmarkDetector_SOURCE_DIR}/include)

include_directories(lib/local/LandmarkDetector/include)
include_directories(lib/local/FaceAnalyser/include)
include_directories(lib/local/GazeAnalyser/include)
include_directories(lib/local/Utilities/include)

#add_executable(FaceLandmarkImg FaceLandmarkImg.cpp)
add_executable(${PROJECT_NAME} ${source} ${header} ${doc} ${idl} ${IDL_GEN_FILES})

target_link_libraries(${PROJECT_NAME} LandmarkDetector)
target_link_libraries(${PROJECT_NAME} FaceAnalyser)
target_link_libraries(${PROJECT_NAME} GazeAnalyser)
target_link_libraries(${PROJECT_NAME} Utilities)
target_link_libraries(${PROJECT_NAME} dlib)
target_link_libraries(${PROJECT_NAME} ${YARP_LIBRARIES} iKin CER::cer_kinematics)

target_link_libraries(${PROJECT_NAME} ${YARP_LIBRARIES} ${OpenCV_LIBRARIES} ${Boost_LIBRARIES} ${TBB_LIBRARIES} ${BLAS_LIBRARIES} ${CER_LIBRARIES})

install (TARGETS ${PROJECT_NAME} DESTINATION bin)

#################################################

yarp_install(FILES ${doc} DESTINATION ${ICUBCONTRIB_MODULES_INSTALL_DIR})

add_subdirectory(app)

option(DOWNLOAD_FACE_LANDMARKS_DAT "download face landmarks dataset" OFF)
set(BZIP2FILE shape_predictor_68_face_landmarks.dat)

if (DOWNLOAD_FACE_LANDMARKS_DAT)
  if (NOT EXISTS ${CMAKE_SOURCE_DIR}/${BZIP2FILE}.bz2 AND NOT EXISTS ${CMAKE_BINARY_DIR}/${BZIP2FILE})
    file(DOWNLOAD http://dlib.net/files/${BZIP2FILE}.bz2 ${CMAKE_BINARY_DIR}/${BZIP2FILE}.bz2 STATUS status LOG log SHOW_PROGRESS)
    find_program(BZIP2_EXECUTABLE bzip2)
    if (NOT BZIP2_EXECUTABLE)
      message(WARNING "BZIP2 Not found on you system, please extract ${CMAKE_BINARY_DIR}/${BZIP2FILE}.bz2 manually")
      message(WARNING "Extract it in $ICUBcontrib_DIR/share/ICUBcontrib/contexts/yarpOpenFace")
    else()
      execute_process(COMMAND ${BZIP2_EXECUTABLE} -d ${CMAKE_BINARY_DIR}/${BZIP2FILE}.bz2)
    endif()
  endif()
endif()

if (EXISTS ${CMAKE_BINARY_DIR}/${BZIP2FILE})
  yarp_install( FILES ${CMAKE_BINARY_DIR}/${BZIP2FILE} DESTINATION ${ICUBCONTRIB_CONTEXTS_INSTAL_DIR}/${PROJECT_NAME} )
else()
  message(WARNING "no ${BZIP2FILE} file has been found please turn ON the DOWNLOAD_FACE_LANDMARKS_DAT flag or download the file manually before running cmake")
endif()
