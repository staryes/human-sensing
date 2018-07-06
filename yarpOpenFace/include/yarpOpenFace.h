/*
 * Copyright (C) 2011 Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
 * Author: Vadim Tikhanoff
 * email:  vadim.tikhanoff@iit.it
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * http://www.robotcub.org/icub/license/gpl.txt
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
 */

#ifndef __YARP_OPENFACE_H__
#define __YARP_OPENFACE_H__

#include <yarp/os/BufferedPort.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/Network.h>
#include <yarp/os/Thread.h>
#include <yarp/os/RateThread.h>
#include <yarp/os/Time.h>
#include <yarp/os/Semaphore.h>
#include <yarp/os/Stamp.h>
#include <yarp/os/Os.h>
#include <yarp/os/Log.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/RpcClient.h>

#include <yarp/sig/Vector.h>
#include <yarp/sig/Image.h>

// OpenFace essential
// dlib
#include <dlib/image_processing/frontal_face_detector.h>

// OpenBLAS
#include <cblas.h>

#include "LandmarkCoreIncludes.h"

#include <tbb/tbb.h>

#include <FaceAnalyser.h>
#include <GazeEstimation.h>

//#include <ImageCapture.h>
#include <Visualizer.h>
#include <VisualizationUtils.h>
#include <RecorderOpenFace.h>
#include <RecorderOpenFaceParameters.h>

#ifndef CONFIG_DIR
#define CONFIG_DIR "~"
#endif

using namespace std;


/* #include <dlib/image_processing/frontal_face_detector.h> */
/* #include <dlib/matrix/lapack/gesvd.h> */
/* #include <dlib/image_processing.h> */
/* #include <dlib/image_io.h> */
/* #include <dlib/opencv.h> */

#include <cv.h>
#include <highgui.h>
#include <opencv2/imgproc/imgproc.hpp>

/* #include <time.h> */
/* #include <map> */
/* #include <dirent.h> */
/* #include <iostream> */
/* #include <string> */
/* #include <iomanip> */

#include "yarpOpenFace_IDLServer.h"

//#define DISPLAY_LANDMARKS           VOCAB4('d','i','s','p')

typedef struct __circle_t {
    float x;
    float y;
    float r;
}circle_t;

class FACEManager : public yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> >
{
private:

    std::string moduleName;             //string containing module name
    std::string predictorFile;          //stringc containing the path of the predictor file
    std::string cntxHomePath;           //contect home path
    std::string inImgPortName;          //string containing image input port name
    std::string outImgPortName;         //string containing image output port name
    std::string outTargetPortName;      //string containing the target port name
    std::string outLandmarksPortName;   //string containing the target port name

    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> >    imageInPort;            //input image ports
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> >    imageOutPort;           //output port Image
    yarp::os::BufferedPort<yarp::os::Bottle>                            targetOutPort;          //target port
    yarp::os::BufferedPort<yarp::os::Bottle>                            landmarksOutPort;

    cv::Mat                             imgMat;

    dlib::frontal_face_detector         faceDetector;
    dlib::shape_predictor               sp;
    cv::Scalar                          color;

    cv::Point                           leftEye, rightEye;

    //void    drawLandmarks(cv::Mat &mat, const dlib::full_object_detection &d);

    //========================
    LandmarkDetector::FaceModelParameters det_parameters;
    LandmarkDetector::CLNF              face_model;
	FaceAnalysis::FaceAnalyserParameters face_analysis_params;
    FaceAnalysis::FaceAnalyser          face_analyser;
	cv::CascadeClassifier               classifier;
	dlib::frontal_face_detector         face_detector_hog;
	LandmarkDetector::FaceDetectorMTCNN face_detector_mtcnn;
//===================================
public:
    /**
     * constructor
     * @param moduleName is passed to the thread in order to initialise all the ports correctly (default yuvProc)
     * @param imgType is passed to the thread in order to work on YUV or on HSV images (default yuv)
     */
    FACEManager( const std::string &moduleName, const std::string &predictorFile, const std::string &cntxHomePath );
    ~FACEManager();

    yarp::os::Semaphore         mutex;
    bool                        displayLandmarks;
    bool                        displayPoints;
    bool                        displayLabels;
    bool                        displayDarkMode;

    bool    open();
    void    close();
    void    onRead( yarp::sig::ImageOf<yarp::sig::PixelRgb> &img );
    void    interrupt();
    bool    execReq(const yarp::os::Bottle &command, yarp::os::Bottle &reply);
};

class FACEModule:public yarp::os::RFModule, public yarpOpenFace_IDLServer
{
    /* module parameters */
    std::string             moduleName;
    std::string             predictorFile;
    std::string             handlerPortName;
    yarp::os::RpcServer     rpcPort;                //rpc port

    /* pointer to a new thread */
    FACEManager            *faceManager;
    bool                    closing;
    std::string             cntxHomePath;

public:

    bool configure(yarp::os::ResourceFinder &rf); // configure all the module parameters and return true if successful
    bool interruptModule();                       // interrupt, e.g., the ports
    bool close();                                 // close and shut down the module

    double getPeriod();
    bool updateModule();

    //IDL interfaces
    /**
     * function that attaches the rpcServer port for IDL
     */
    bool attach(yarp::os::RpcServer &source);
    /**
     * function that handles an IDL message - display on/off
     */
    bool display(const std::string& element, const std::string& value);
    /**
     * function that handles an IDL message - quit
     */
    bool quit();
};

#endif
//empty line to make gcc happy
