/*
 * Copyright (C) 2011 Department of Robotics Brain and Cognitive Sciences -
 * Istituto Italiano di Tecnologia
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

#include "yarpOpenFace.h"
#include <yarp/sig/all.h>

/**********************************************************/
bool FACEModule::configure(yarp::os::ResourceFinder &rf) {
    rf.setVerbose();
    moduleName = rf.check("name", yarp::os::Value("yarpOpenFace"),
                          "module name (string)")
                     .asString();
    predictorFile =
        rf.check("yarpOpenFaceFile",
                 yarp::os::Value("shape_predictor_68_face_landmarks.dat"),
                 "path name (string)")
            .asString();

    std::string firstStr = rf.findFile(predictorFile.c_str());

    setName(moduleName.c_str());

    handlerPortName = "/";
    handlerPortName += getName();
    handlerPortName += "/rpc:i";

    cntxHomePath = rf.getHomeContextPath().c_str();

    if (!rpcPort.open(handlerPortName.c_str())) {
        yError() << getName().c_str() << " Unable to open port "
                 << handlerPortName.c_str();
        return false;
    }

    attach(rpcPort);
    closing = false;

    /* create the thread and pass pointers to the module parameters */
    faceManager = new FACEManager(moduleName, firstStr, cntxHomePath);

    /* now start the thread to do the work */
    faceManager->open();

    return true;
}

/**********************************************************/
bool FACEModule::interruptModule() {
    rpcPort.interrupt();
    return true;
}

/**********************************************************/
bool FACEModule::close() {
    // rpcPort.close();
    yDebug() << "starting the shutdown procedure";
    faceManager->interrupt();
    faceManager->close();
    yDebug() << "deleting thread";
    delete faceManager;
    closing = true;
    yDebug() << "done deleting thread";
    return true;
}

/**********************************************************/
bool FACEModule::updateModule() { return !closing; }

/**********************************************************/
double FACEModule::getPeriod() { return 0.01; }

/************************************************************************/
bool FACEModule::attach(yarp::os::RpcServer &source) {
    return this->yarp().attachAsServer(source);
}

/**********************************************************/
// bool FACEModule::display(const std::string& element, const std::string&
// value)
// {
//     bool returnVal = false;

//     if (element == "landmarks" || element == "points" || element == "labels"
//     || element == "dark-mode")
//     {
//         if (element == "landmarks")
//         {
//             if (value=="on")
//             {
//                 faceManager->displayLandmarks=true;
//                 returnVal = true;
//             }
//             else if (value=="off")
//             {
//                 faceManager->displayLandmarks = false;
//                 returnVal = true;
//             }
//             else
//                 yInfo() << "error setting value for landmarks";
//         }
//         if (element == "points")
//         {
//             if (value=="on")
//             {
//                 faceManager->displayPoints=true;
//                 returnVal = true;
//             }
//             else if (value=="off")
//             {
//                 faceManager->displayPoints = false;
//                 returnVal = true;
//             }
//             else
//                 yInfo() << "error setting value for points";
//         }
//         if (element == "labels")
//         {
//             if (value=="on")
//             {
//                 faceManager->displayLabels=true;
//                 returnVal = true;
//             }
//             else if (value=="off")
//             {
//                 faceManager->displayLabels = false;
//                 returnVal = true;
//             }
//             else
//                 yInfo() << "error setting value for labels";
//         }
//         if (element == "dark-mode")
//         {
//             if (value=="on")
//             {
//                 faceManager->displayDarkMode=true;
//                 returnVal = true;
//             }
//             else if (value=="off")
//             {
//                 faceManager->displayDarkMode = false;
//                 returnVal = true;
//             }
//             else
//                 yInfo() << "error setting value for darkMode";
//         }
//         //yInfo() << "should now display \"landmarks\" " <<
//         faceManager->displayLandmarks << "\"points\"" <<
//         faceManager->displayPoints << "\"labels\"" <<
//         faceManager->displayLabels  << "\"dark-mode\"" <<
//         faceManager->displayDarkMode;
//     }
//     else
//     {
//         returnVal = false;
//         yInfo() << "Error in display request";
//     }
//     return returnVal;
// }

/**********************************************************/
bool FACEModule::quit() {
    closing = true;
    return true;
}

/**********************************************************/
FACEManager::~FACEManager() {}

/**********************************************************/
FACEManager::FACEManager(const std::string &moduleName,
                         const std::string &predictorFile,
                         const std::string &cntxHomePath) {
    yDebug() << "initialising Variables";
    this->moduleName = moduleName;
    this->predictorFile = predictorFile;
    this->cntxHomePath = cntxHomePath;
    yInfo() << "contextPATH = " << cntxHomePath.c_str();
}

/**********************************************************/
bool FACEManager::open() {
    this->useCallback();

    // create all ports
    inImgPortName = "/" + moduleName + "/image:i";
    BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb>>::open(
        inImgPortName.c_str());

    outImgPortName = "/" + moduleName + "/image:o";
    imageOutPort.open(outImgPortName.c_str());

    outTargetPortName = "/" + moduleName + "/target:o";
    targetOutPort.open(outTargetPortName.c_str());

    outLandmarksPortName = "/" + moduleName + "/landmarks:o";
    landmarksOutPort.open(outLandmarksPortName.c_str());

    outImgLefteyePortName = "/" + moduleName + "/image:lefteye";
    imageOutLefteyePort.open(outImgLefteyePortName.c_str());

    outImgRighteyePortName = "/" + moduleName + "/image:righteye";
    imageOutRighteyePort.open(outImgRighteyePortName.c_str());

    // yDebug() << "path is: " << predictorFile.c_str();

    // faceDetector = dlib::get_frontal_face_detector();
    // dlib::deserialize(predictorFile.c_str()) >> sp;

    //   FACEModels fms;
    // fms.isModelsLoaded = fms.loadModels();
    LandmarkDetector::FaceModelParameters det_parameters; // test
    p_det_parameters = det_parameters;

    // The modules that are being used for tracking
    cout << "Loading the model" << endl;
    LandmarkDetector::CLNF face_model(p_det_parameters.model_location);
    p_face_model = face_model;

    cout << "Model loaded" << endl;

    // Load facial feature extractor and AU analyser (make sure it is static)
    // FaceAnalysis::FaceAnalyserParameters face_analysis_params("-yarp");
    // face_analysis_params.OptimizeForImages();

    //	FaceAnalysis::FaceAnalyser face_analyser(face_analysis_params);
    //    p_face_analyser = &face_analyser;

    // If bounding boxes not provided, use a face detector
    //	cv::CascadeClassifier
    // classifier(det_parameters.haar_face_detector_location);
    //	dlib::frontal_face_detector face_detector_hog =
    // dlib::get_frontal_face_detector();
    LandmarkDetector::FaceDetectorMTCNN face_detector_mtcnn(
        det_parameters.mtcnn_face_detector_location);
    p_face_detector_mtcnn = face_detector_mtcnn;

    // color = cv::Scalar( 0, 255, 0 );

    displayLandmarks = true;
    displayPoints = false;
    displayLabels = false;
    displayDarkMode = false;

    return true;
}

/**********************************************************/
void FACEManager::close() {
    mutex.wait();
    yDebug() << "now closing ports...";
    imageOutPort.writeStrict();
    imageOutPort.close();
    imageInPort.close();
    targetOutPort.close();
    landmarksOutPort.close();
    BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb>>::close();
    mutex.post();
    yDebug() << "finished closing the read port...";
}

/**********************************************************/
void FACEManager::interrupt() {
    yDebug() << "cleaning up...";
    yDebug() << "attempting to interrupt ports";
    BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb>>::interrupt();
    yDebug() << "finished interrupt ports";
}

/**********************************************************/
void FACEManager::onRead(yarp::sig::ImageOf<yarp::sig::PixelRgb> &img) {
    mutex.wait();
    yarp::sig::ImageOf<yarp::sig::PixelRgb> &outImg = imageOutPort.prepare();
    yarp::os::Bottle &target = targetOutPort.prepare();
    yarp::os::Bottle &landmarks = landmarksOutPort.prepare();
    target.clear();
    yarp::sig::ImageOf<yarp::sig::PixelRgb> &outRighteyeImg =
        imageOutRighteyePort.prepare();
    yarp::sig::ImageOf<yarp::sig::PixelRgb> &outLefteyeImg =
        imageOutLefteyePort.prepare();

    // dlib::cv_image<dlib::bgr_pixel> dlibimg(imgMat);
    //-------------------------
    // If can't find MTCNN face detector, default to HOG one
    if (p_det_parameters.curr_face_detector ==
            LandmarkDetector::FaceModelParameters::FaceDetector::
                MTCNN_DETECTOR &&
        p_face_detector_mtcnn.empty()) {
        cout << "INFO: defaulting to HOG-SVM face detector" << endl;
        p_det_parameters.curr_face_detector = LandmarkDetector::
            FaceModelParameters::FaceDetector::HOG_SVM_DETECTOR;
    }

    // A utility for visualizing the results
    Utilities::Visualizer visualizer(false, false, false, false);

    cv::Mat rgb_image;

//    cv::Mat leftEye;
//    cv::Mat rightEye;

    rgb_image = imgMat;

    float fx = 500.0;
    float fy = 500.0;
    float cx = imgMat.rows / 2;
    float cy = imgMat.cols / 2;

    if (!p_face_model.eye_model) {
        cout << "WARNING: no eye model found" << endl;
    }

    // cout << "Starting tracking" << endl;

    if (!rgb_image.empty()) {
        // std::vector<std::string> argument = "-yarp";

        // Utilities::RecorderOpenFaceParameters recording_params(false, false,
        // false, false, false,
        // false,false, false, false, false, false, false, fx, fy, cx, cy, 30);

        // if (!p_face_model.eye_model)
        {
            //	recording_params.setOutputGaze(false);
        }
        // Utilities::RecorderOpenFace open_face_rec(name, recording_params,
        // arguments); //XX
        visualizer.SetImage(rgb_image, fx, fy, cx, cy);

        // Making sure the image is in uchar grayscale (some face detectors use
        // RGB, landmark detector uses
        // grayscale)
        cv::Mat_<uchar> grayscale_image;
        cv::cvtColor(rgb_image, grayscale_image, CV_BGR2GRAY);

        // Detect faces in an image
        vector<cv::Rect_<float>> face_detections;

        vector<float> confidences;
        LandmarkDetector::DetectFacesMTCNN(face_detections, rgb_image,
                                           p_face_detector_mtcnn, confidences);

        // Detect landmarks around detected faces
        int face_det = 0;
        // perform landmark detection for every face detected
        for (size_t face = 0; face < face_detections.size(); ++face) {

            // if there are multiple detections go through them
            bool success = LandmarkDetector::DetectLandmarksInImage(
                rgb_image, face_detections[face], p_face_model,
                p_det_parameters, grayscale_image);

            // for (int i=0; i<p_face_model.detected_landmarks.size(); i++)
            //    std::cout << p_face_model.detected_landmarks.row[i] << endl;

            // 2D points
            cv::Mat_<float> landmarks_2D = p_face_model.detected_landmarks;

            landmarks_2D = landmarks_2D.reshape(1, 2).t();

            cv::Size s = landmarks_2D.size();
            // std::cout << "size: " << s.height << "," << s.width << endl;
            // for( int i=0; i < landmarks_2D)

            // int righteye_region_width =
            //     landmarks_2D[39][0] - landmarks_2D[36][0];
            // int righteye_region_height = 0.5 * righteye_region_width;
            //     //landmarks_2D[41][1] - landmarks_2D[37][1];
            // int righteye_region_top =
            //     landmarks_2D[37][1] - 0.5 * righteye_region_height;
            // int righteye_region_left =
            //     landmarks_2D[36][0] - 0.5 * righteye_region_width;

            // righteye_region_width = righteye_region_width * 2;
            // righteye_region_height = righteye_region_height * 2;

            // cv::Rect roi;
            // roi.x = righteye_region_top;
            // roi.y = righteye_region_left;
            // roi.width = righteye_region_width;
            // roi.height = righteye_region_height;

            // rightEye = rgb_image(roi);

            // int lefteye_region_width =
            //     landmarks_2D[45][0] - landmarks_2D[42][0];
            // int lefteye_region_height =
            //     landmarks_2D[47][1] - landmarks_2D[43][1];
            // int lefteye_region_top =
            //     landmarks_2D[43][1] - 0.5 * lefteye_region_height;
            // int lefteye_region_left =
            //     landmarks_2D[42][0] - 0.5 * lefteye_region_width;

            // std::cout << lefteye_region_width << endl;

            // lefteye_region_width = lefteye_region_width * 2;
            // lefteye_region_height = lefteye_region_height * 2;

//            roi.x = lefteye_region_top;
            //          roi.y = lefteye_region_left;
            // roi.width = lefteye_region_width;
            // roi.height = lefteye_region_height;

            // leftEye = rgb_image(roi);
            //   std::cout << "eyeWidth = " << leftEye.empty() << endl;
            // if(leftEye.empty() == 0)
            {
                //  cv::imshow("crop", leftEye);
            }

            // Estimate head pose and eye gaze
            cv::Vec6d pose_estimate =
                LandmarkDetector::GetPose(p_face_model, fx, fy, cx, cy);

            // Gaze tracking, absolute gaze direction
            cv::Point3f gaze_direction0(0, 0, -1);
            cv::Point3f gaze_direction1(0, 0, -1);
            cv::Vec2f gaze_angle(0, 0);

            if (p_face_model.eye_model) {
                GazeAnalysis::EstimateGaze(p_face_model, gaze_direction0, fx,
                                           fy, cx, cy, true);
                GazeAnalysis::EstimateGaze(p_face_model, gaze_direction1, fx,
                                           fy, cx, cy, false);
                gaze_angle = GazeAnalysis::GetGazeAngle(gaze_direction0,
                                                        gaze_direction1);
            }

            cv::Mat sim_warped_img;
            cv::Mat_<double> hog_descriptor;
            int num_hog_rows = 0, num_hog_cols = 0;

            // Displaying the tracking visualizations
            visualizer.SetObservationFaceAlign(sim_warped_img);
            visualizer.SetObservationHOG(hog_descriptor, num_hog_rows,
                                         num_hog_cols);
            visualizer.SetObservationLandmarks(
                p_face_model.detected_landmarks, 1.0,
                p_face_model.GetVisibilities()); // Set confidence to high to
                                                 // make sure we always
                                                 // visualize
            visualizer.SetObservationPose(pose_estimate, 1.0);
            visualizer.SetObservationGaze(
                gaze_direction0, gaze_direction1,
                LandmarkDetector::CalculateAllEyeLandmarks(p_face_model),
                LandmarkDetector::Calculate3DEyeLandmarks(p_face_model, fx, fy,
                                                          cx, cy),
                p_face_model.detection_certainty);
            //            	visualizer.SetObservationActionUnits(face_analyser.GetCurrentAUsReg(),
            //            face_analyser.GetCurrentAUsClass());
        }
        if (face_detections.size() > 0) {
            //			visualizer.ShowObservation(); // XXX turn off the
            // imshows
        }
    }

    //-------------------------
    IplImage yarpImg = visualizer.GetVisImage();

    outImg.resize(yarpImg.width, yarpImg.height);
    cvCopy(&yarpImg, (IplImage *)outImg.getIplImage());

    imageOutPort.write();

    //  IplImage yarpRighteyeImg;
    //  if (rightEye.empty() == 0){
        //       yarpRighteyeImg = rightEye;
        // outRighteyeImg.resize(yarpRighteyeImg.width, yarpRighteyeImg.height);
        // cvCopy(&yarpRighteyeImg, (IplImage *)outRighteyeImg.getIplImage());
        // imageOutRighteyePort.write();
//    }
    mutex.post();
}
