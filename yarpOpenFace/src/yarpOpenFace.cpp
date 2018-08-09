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
#include "constants.h"
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

/****************************************************************/
// bool getCameraOptions()
// {
//     if (camPort.getOutputCount()>0)
//     {
//         yarp::os::Bottle cmd,rep;
//         cmd.addVocab(yarp::os::Vocab::encode("visr"));
//         cmd.addVocab(yarp::os::Vocab::encode("get"));
//         cmd.addVocab(yarp::os::Vocab::encode("fov"));
//         if (camPort.write(cmd,rep))
//         {
//             if (rep.size()>=5)
//             {
//                 fx = rep.get(3).asDouble();
//                 fy = rep.get(4).asDouble();
//                 yInfo()<<"camera fov_h (from sensor) ="<<fx;
//                 yInfo()<<"camera fov_v (from sensor) ="<<fy;
//                 return true;
//             }
//         }
//     }

//     return true;
// }

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

    camPort.open("/" + moduleName + "/cam:rpc");
    yarp::os::Network::connect(camPort.getName().c_str(), "/depthCamera/rpc:i", "tcp");
    //getCameraOptions();

    // yDebug() << "path is: " << predictorFile.c_str();

    // faceDetector = dlib::get_frontal_face_detector();
    // dlib::deserialize(predictorFile.c_str()) >> sp;

    //   FACEModels fms;
    // fms.isModelsLoaded = fms.loadModels();
    det_parameters = new LandmarkDetector::FaceModelParameters();

    // The modules that are being used for tracking
    cout << "Loading the model" << endl;
    face_model = new LandmarkDetector::CLNF(det_parameters->model_location);

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
    face_detector_mtcnn = new LandmarkDetector::FaceDetectorMTCNN(
        det_parameters->mtcnn_face_detector_location);

    // color = cv::Scalar( 0, 255, 0 );

    yDebug() << " open the drivers";
    //std::vector<yarp::dev::PolyDriver> drivers;

    drivers = new std::vector<yarp::dev::PolyDriver>(3);
    openDrivers(*drivers);

    // Kalman Filter config
    // 1.kalman_left filter setup
    const int stateNum = 4;
    const int measureNum = 2;
    kalman_left = new cv::KalmanFilter(stateNum, measureNum ,0); //state_lefg(x,y,deltaX,deltaY)
    //state_left = new cv::Mat(stateNum, 1, CV_32F);
    //processNoise = new cv::Mat(stateNum, 1, CV_32F);
    measurement_left = new cv::Mat(cv::Mat::zeros(measureNum, 1, CV_32F)); //measure(x,y)

    //cv::randn( *state_left, cv::Scalar::all(0), cv::Scalar::all(0.1) );

    kalman_left->transitionMatrix = (cv::Mat_<float>(stateNum, stateNum) <<
                                     // transition matrix
                                     1, 0, 1, 0,
                                     0, 1, 0, 1,
                                     0, 0, 1, 0,
                                     0, 0, 0, 1
        );

    cv::setIdentity(kalman_left->measurementMatrix, cv::Scalar::all(1));
    cv::setIdentity(kalman_left->processNoiseCov, cv::Scalar::all(1e-5));
    cv::setIdentity(kalman_left->measurementNoiseCov, cv::Scalar::all(1e-1));
    cv::setIdentity(kalman_left->errorCovPost, cv::Scalar::all(1));

    //initialize post state_lefg of kalman_left filter at random
    cv::randn(kalman_left->statePost, cv::Scalar::all(0), cv::Scalar::all(0.1));

    kalman_right = new cv::KalmanFilter(stateNum, measureNum ,0); //state_lefg(x,y,deltaX,deltaY)
    //state_right = new cv::Mat(stateNum, 1, CV_32F);
    //processNoise = new cv::Mat(stateNum, 1, CV_32F);
    measurement_right = new cv::Mat(cv::Mat::zeros(measureNum, 1, CV_32F)); //measure(x,y)

    //cv::randn( *state_right, cv::Scalar::all(0), cv::Scalar::all(0.1) );

    kalman_right->transitionMatrix = (cv::Mat_<float>(stateNum, stateNum) <<
                                      // transition matrix
                                      1, 0, 1, 0,
                                      0, 1, 0, 1,
                                      0, 0, 1, 0,
                                      0, 0, 0, 1
        );

    cv::setIdentity(kalman_right->measurementMatrix, cv::Scalar::all(1));
    cv::setIdentity(kalman_right->processNoiseCov, cv::Scalar::all(1e-5));
    cv::setIdentity(kalman_right->measurementNoiseCov, cv::Scalar::all(1e-1));
    cv::setIdentity(kalman_right->errorCovPost, cv::Scalar::all(1));

    //initialize post state_lefg of kalman_left filter at random
    cv::randn(kalman_right->statePost, cv::Scalar::all(0), cv::Scalar::all(0.1));

    displayLandmarks = true;
    displayPoints = false;
    displayLabels = false;
    displayDarkMode = false;

    return true;
}

/**********************************************************/
void FACEManager::close() {
    mutex.wait();
    yDebug() << "now close drivers...";
    closeDrivers(drivers);
    //delete drivers;
    yDebug() << "now delete detectors...";
    delete face_detector_mtcnn;
    delete face_model;
    delete det_parameters;
    yDebug() << "now delete KF parameters...";
    delete kalman_left;
    //delete state_left;
    //delete processNoise;
    delete measurement_left;
    delete kalman_right;
    //delete state_right;
    //delete processNoise;
    delete measurement_right;
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

    // Get the image from the yarp port
    imgMat = cv::cvarrToMat((IplImage *)img.getIplImage());

    // dlib::cv_image<dlib::bgr_pixel> dlibimg(imgMat);
    //-------------------------
    // If can't find MTCNN face detector, default to HOG one
    if (det_parameters->curr_face_detector ==
        LandmarkDetector::FaceModelParameters::FaceDetector::
        MTCNN_DETECTOR &&
        face_detector_mtcnn->empty()) {
        cout << "INFO: defaulting to HOG-SVM face detector" << endl;
        det_parameters->curr_face_detector = LandmarkDetector::
            FaceModelParameters::FaceDetector::HOG_SVM_DETECTOR;
    }

    // A utility for visualizing the results
    Utilities::Visualizer visualizer(false, false, false, false);

    cv::Mat rgb_image;

    cv::Mat rightEye;
    cv::Mat leftEye;

    rgb_image = imgMat;


    fx = 450.0;
    fy = 460.0;
    cx = imgMat.cols / 2;
    cy = imgMat.rows / 2;

    if (!face_model->eye_model) {
        cout << "WARNING: no eye model found" << endl;
    }

//eye crop resize
    const int resize_width = 40;
    const int resize_height = 20;
    double resize_ratio = 1.0;

    // cout << "Starting tracking" << endl;

    if (!rgb_image.empty()) {

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
                                           *face_detector_mtcnn, confidences);

        // Detect landmarks around detected faces
        int face_det = 0;
        // perform landmark detection for every face detected
        for (size_t face = 0; face < face_detections.size(); ++face) {

            // if there are multiple detections go through them
            bool success = LandmarkDetector::DetectLandmarksInImage(
                rgb_image, face_detections[face], *face_model,
                *det_parameters, grayscale_image);

            // for (int i=0; i<p_face_model.detected_landmarks.size(); i++)
            //    std::cout << p_face_model.detected_landmarks.row[i] << endl;

            // 2D points
            cv::Mat_<float> landmarks_2D = face_model->detected_landmarks;

            landmarks_2D = landmarks_2D.reshape(1, 2).t();

            cv::Size s = landmarks_2D.size();
            // std::cout << "size: " << s.height << "," << s.width << endl;
            // for( int i=0; i < landmarks_2D)

            int lefteye_region_width =
                landmarks_2D[39][0] - landmarks_2D[36][0];
            int lefteye_region_height = 0.5 * lefteye_region_width;

            double mean_y = 0;
            for (int i = 0; i < 6; i++)
                mean_y += (double)landmarks_2D[36 + i][1];
            mean_y = mean_y / 6;

            double lefteye_region_center_y = mean_y;

            double mean_x = 0;
            for (int i = 0; i < 6; i++)
                mean_x += (double)landmarks_2D[36 + i][0];
            mean_x = mean_x / 6;

            double lefteye_region_center_x = mean_x;

            lefteye_region_width = lefteye_region_width * 2;
            lefteye_region_height = lefteye_region_height * 2;

            cv::Rect roi;
            roi.x = lefteye_region_center_x - 0.5 * (double)lefteye_region_width;
            roi.y = lefteye_region_center_y - 0.5 * (double)lefteye_region_height;
            roi.width = lefteye_region_width;
            roi.height = lefteye_region_height;

            cv::Mat tempLeftEye;
            tempLeftEye = rgb_image(roi);

            cv::resize(tempLeftEye, leftEye, cv::Size(resize_width, resize_height), 0, 0, CV_INTER_LINEAR);

            cv::Point leftPupil = findEyeCenter(leftEye, roi, "Left Eye");

            cv::circle(leftEye, leftPupil, 3, 1234);

            //Kalman filter
            //2.kalman_left filter prediction
            cv::Mat prediction = kalman_left->predict();
            cv::Point2f predictPt = cv::Point2f(prediction.at<float>(0), prediction.at<float>(1));

            //3.update measure
            measurement_left->at<float>(0) = (float)leftPupil.x;
            measurement_left->at<float>(1) = (float)leftPupil.y;

            //4.update
            kalman_left->correct(*measurement_left);

            cv::Point stateLeftPt = cv::Point( (int)kalman_left->statePost.at<float>(0), (int)kalman_left->statePost.at<float>(1));

            leftPupil = stateLeftPt;

            //std::cout << "leftPupil: " << leftPupil.x << "," << leftPupil.y << endl;

            cv::circle(leftEye, leftPupil, 3, cv::Scalar(0,255,0));

            resize_ratio = (double)lefteye_region_width / resize_width;
            std::cout << "ratio " << resize_ratio << endl;

            leftPupil.x = (leftPupil.x * resize_ratio) + roi.x;
            leftPupil.y = (leftPupil.y * resize_ratio) + roi.y;

            // right pupil
            int righteye_region_width =
                landmarks_2D[45][0] - landmarks_2D[42][0];
            int righteye_region_height = 0.5 * righteye_region_width;

            mean_y = 0;
            mean_x = 0;
            for (int i = 0; i < 6; i++) {
                mean_y += landmarks_2D[42 + i][1];
                mean_x += landmarks_2D[42 + i][0];
            }
            mean_x = mean_x / 6;
            mean_y = mean_y / 6;

            int righteye_region_center_y = mean_y;
            int righteye_region_center_x = mean_x;

            righteye_region_width = righteye_region_width * 2;
            righteye_region_height = righteye_region_height * 2;

            // cv::Rect roi;
            roi.x =
                righteye_region_center_x - 0.5 * righteye_region_width;
            roi.y =
                righteye_region_center_y - 0.5 * righteye_region_height;
            roi.width = righteye_region_width;
            roi.height = righteye_region_height;

            cv::Mat tempRightEye;
            tempRightEye = rgb_image(roi);
            //rightEye = rgb_image(roi);

            cv::resize(tempRightEye, rightEye, cv::Size(resize_width, resize_height), 0, 0, CV_INTER_LINEAR);

            cv::Point rightPupil = findEyeCenter(rightEye, roi, "Right Eye");

            cv::circle(rightEye, rightPupil, 3, 1234);

            //Kalman filter
            //2.kalman filter prediction
            prediction = kalman_right->predict();
            //predictPt = cv::Point2f(prediction.at<float>(0), prediction.at<float>(1));

            //3.update measure
            measurement_right->at<float>(0) = (float)rightPupil.x;
            measurement_right->at<float>(1) = (float)rightPupil.y;

            //4.update
            kalman_right->correct(*measurement_right);

            std::cout << "rightPupil: " << rightPupil.x << "," << rightPupil.y << endl;

            cv::Point stateRightPt = cv::Point( (int)kalman_right->statePost.at<float>(0), (int)kalman_right->statePost.at<float>(1));

            rightPupil = stateRightPt;

            std::cout << "rightPupil: " << rightPupil.x << "," << rightPupil.y << endl;

            cv::circle(rightEye, rightPupil, 3, cv::Scalar(0,255,0));

//            rightPupil.x = rightPupil.x + roi.x;
//            rightPupil.y = rightPupil.y + roi.y;

            resize_ratio = (double)righteye_region_width / resize_width;
            std::cout << "ratio " << resize_ratio << endl;
            rightPupil.x = (rightPupil.x * resize_ratio) + roi.x;
            rightPupil.y = (rightPupil.y * resize_ratio) + roi.y;


            // Estimate head pose and eye gaze
            cv::Vec6d pose_estimate =
                LandmarkDetector::GetPose(*face_model, fx, fy, cx, cy);

            // Gaze tracking, absolute gaze direction
            cv::Point3f gaze_direction0(0, 0, -1);
            cv::Point3f gaze_direction1(0, 0, -1);
            cv::Vec2f gaze_angle(0, 0);

            cv::Point3f leftEyeballCentre(0, 0, 0);
            cv::Point3f rightEyeballCentre(0, 0, 0);

            if (face_model->eye_model) {
                GazeAnalysis::EstimateGazeR1(*face_model, rightPupil,
                                             gaze_direction1, rightEyeballCentre, fx, fy, cx, cy,
                                             false);

                GazeAnalysis::EstimateGazeR1(*face_model, leftPupil,
                                             gaze_direction0, leftEyeballCentre, fx, fy, cx, cy,
                                             true);

                gaze_angle = GazeAnalysis::GetGazeAngle(gaze_direction0,
                                                        gaze_direction1);

                //std::cout << "gaze angle: " << gaze_angle[0] << " " << gaze_angle[1] << endl;
            }

            double baseline = sqrt(pow((leftEyeballCentre.x - rightEyeballCentre.x),2) + pow((leftEyeballCentre.y - rightEyeballCentre.y),2) + pow((leftEyeballCentre.z - rightEyeballCentre.z),2));
            //std::cout << "baseline: " << baseline << endl;
            //std::cout << "left eye ball " << leftEyeballCentre.x << " " << leftEyeballCentre.y << " " << leftEyeballCentre.z << endl;
            cv::Point3f gaze_direction_abs;
            //std::cout << "left gaze direction " << gaze_direction0.x << " " << gaze_direction0.y << " " << gaze_direction0.z << endl;
            gaze_direction_abs.z = baseline / ((gaze_direction0.x/gaze_direction0.z)+(gaze_direction1.x/gaze_direction1.z));

            //std::cout << gaze_direction_abs.z << endl;

            cv::Point3f gaze_center = (leftEyeballCentre + rightEyeballCentre)/2;
            cv::Point3f gaze_directionAvg = gaze_direction0 + gaze_direction1;
            gaze_directionAvg = gaze_directionAvg / norm(gaze_directionAvg);
            cv::Point3f gaze_point3d = gaze_center + gaze_directionAvg*gaze_center.z*0.5;

            std::cout << "gaze center " << gaze_center.x << " " <<gaze_center.y << " " << gaze_center.z << endl;
            std::cout << "gaze point  " << gaze_point3d.x << " " << gaze_point3d.y << " " << gaze_point3d.z << endl;
            std::cout << '0' << std::endl;

            q=getEncoders(*drivers);
            
            //    setHeadPoseEncoder(q);

            for (int i = 0; i < 6; i++)
                cout << q[i] << " " ;
            cout << endl;

            yarp::sig::Vector pose_act(3,0.0);
            yarp::sig::Vector ori_act(3, 0.0);
     
            pose_act.setSubvector(0,q.subVector(0,2));
            ori_act.setSubvector(0,q.subVector(3,5));

            // transforming the pose w.r.t the root of the robot
            // igaze->getLeftEyePose(pose_act,ori_act);

            yarp::sig::Matrix H;                       // transformation matrx
            //H = yarp::math::axis2dcm(ori_act);
            H = yarp::math::euler2dcm(ori_act);
            H(0,3) = pose_act[0];
            H(1,3) = pose_act[1];
            H(2,3) = pose_act[2];
            pose_clm.resize(4);
            pose_clm[0] = gaze_point3d.x / 1000; //convert to [m]
            pose_clm[1] = gaze_point3d.y / 1000;
            pose_clm[2] = gaze_point3d.z / 1000;
            pose_clm[3] = 1;
            pose_robot = H*pose_clm;

            std::cout << pose_clm[0] << " " << pose_clm[1] << " " << pose_clm[2] << std::endl;
            std::cout << pose_robot[0] << " " << pose_robot[1] << " " << pose_robot[2] << std::endl;

            yarp::os::Bottle &pos = target.addList();
            pos.addDouble(pose_robot[0]);
            pos.addDouble(pose_robot[1]);
            pos.addDouble(pose_robot[2]);
            // pos.addDouble(gaze_point3d.x);
            // pos.addDouble(gaze_point3d.y);
            // pos.addDouble(gaze_point3d.z);

            targetOutPort.write();
    
            cv::Mat sim_warped_img;
            cv::Mat_<double> hog_descriptor;
            int num_hog_rows = 0, num_hog_cols = 0;

            // Displaying the tracking visualizations
            visualizer.SetObservationFaceAlign(sim_warped_img);
            visualizer.SetObservationHOG(hog_descriptor, num_hog_rows,
                                         num_hog_cols);
            visualizer.SetObservationLandmarks(
                face_model->detected_landmarks, 1.0,
                face_model->GetVisibilities()); // Set confidence to high to
            // make sure we always
            // visualize
            visualizer.SetObservationPose(pose_estimate, 1.0);
            visualizer.SetObservationGaze(
                gaze_direction0, gaze_direction1,
                LandmarkDetector::CalculateAllEyeLandmarks(*face_model),
                LandmarkDetector::Calculate3DEyeLandmarks(*face_model, fx, fy,
                                                          cx, cy),
                face_model->detection_certainty);
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

    IplImage yarpRighteyeImg;
    if (leftEye.empty() == 0) {
        yarpRighteyeImg = rightEye;
        outRighteyeImg.resize(yarpRighteyeImg.width, yarpRighteyeImg.height);
        cvCopy(&yarpRighteyeImg, (IplImage *)outRighteyeImg.getIplImage());
        imageOutRighteyePort.write();
    }

    IplImage yarpLefteyeImg;
    if (rightEye.empty() == 0) {
        yarpLefteyeImg = leftEye;
        outLefteyeImg.resize(yarpLefteyeImg.width, yarpLefteyeImg.height);
        cvCopy(&yarpLefteyeImg, (IplImage *)outLefteyeImg.getIplImage());
        imageOutLefteyePort.write();
    }

    mutex.post();
}

/**********************************************************/
// void FACEManager::setHeadPoseEncoder(yarp::sig::Vector q)
// {
//   encoders = q;

//     for (int i = 0; i < 6; i++)
//         cout << q[i] << " " ;
//     cout << endl;

  
// }

cv::Mat computeMatXGradient(const cv::Mat &mat) {
    cv::Mat out(mat.rows, mat.cols, CV_64F);

    for (int y = 0; y < mat.rows; ++y) {
        const uchar *Mr = mat.ptr<uchar>(y);
        double *Or = out.ptr<double>(y);

        Or[0] = Mr[1] - Mr[0];
        for (int x = 1; x < mat.cols - 1; ++x) {
            Or[x] = (Mr[x + 1] - Mr[x - 1]) / 2.0;
        }
        Or[mat.cols - 1] = Mr[mat.cols - 1] - Mr[mat.cols - 2];
    }

    return out;
}

cv::Mat matrixMagnitude(const cv::Mat &matX, const cv::Mat &matY) {
    cv::Mat mags(matX.rows, matX.cols, CV_64F);
    for (int y = 0; y < matX.rows; ++y) {
        const double *Xr = matX.ptr<double>(y), *Yr = matY.ptr<double>(y);
        double *Mr = mags.ptr<double>(y);
        for (int x = 0; x < matX.cols; ++x) {
            double gX = Xr[x], gY = Yr[x];
            double magnitude = sqrt((gX * gX) + (gY * gY));
            Mr[x] = magnitude;
        }
    }
    return mags;
}

void testPossibleCentersFormula(int x, int y, const cv::Mat &weight, double gx,
                                double gy, cv::Mat &out) {
    // for all possible centers
    for (int cy = 0; cy < out.rows; ++cy) {
        double *Or = out.ptr<double>(cy);
        const unsigned char *Wr = weight.ptr<unsigned char>(cy);
        for (int cx = 0; cx < out.cols; ++cx) {
            if (x == cx && y == cy) {
                continue;
            }
            // create a vector from the possible center to the gradient origin
            double dx = x - cx;
            double dy = y - cy;
            // normalize d
            double magnitude = sqrt((dx * dx) + (dy * dy));
            dx = dx / magnitude;
            dy = dy / magnitude;
            double dotProduct = dx * gx + dy * gy;
            dotProduct = std::max(0.0, dotProduct);
            // square and multiply by the weight
            if (kEnableWeight) {
                Or[cx] += dotProduct * dotProduct * (Wr[cx] / kWeightDivisor);
            } else {
                Or[cx] += dotProduct * dotProduct;
            }
        }
    }
}

//cv::Mat

double computeDynamicThreshold(const cv::Mat &mat, double stdDevFactor) {
    cv::Scalar stdMagnGrad, meanMagnGrad;
    cv::meanStdDev(mat, meanMagnGrad, stdMagnGrad);
    double stdDev = stdMagnGrad[0] / sqrt(mat.rows * mat.cols);
    return stdDevFactor * stdDev + meanMagnGrad[0];
}

cv::Point unscalePoint(cv::Point p, cv::Rect origSize) {
    float ratio = (((float)kFastEyeWidth) / origSize.width);
    int x = round(p.x / ratio);
    int y = round(p.y / ratio);
    return cv::Point(x, y);
}

bool inMat(cv::Point p, int rows, int cols) {
    return p.x >= 0 && p.x < cols && p.y >= 0 && p.y < rows;
}

bool floodShouldPushPoint(const cv::Point &np, const cv::Mat &mat) {
    return inMat(np, mat.rows, mat.cols);
}

// returns a mask
cv::Mat floodKillEdges(cv::Mat &mat) {
    rectangle(mat, cv::Rect(0, 0, mat.cols, mat.rows), 255);

    cv::Mat mask(mat.rows, mat.cols, CV_8U, 255);
    std::queue<cv::Point> toDo;
    toDo.push(cv::Point(0, 0));
    while (!toDo.empty()) {
        cv::Point p = toDo.front();
        toDo.pop();
        if (mat.at<float>(p) == 0.0f) {
            continue;
        }
        // add in every direction
        cv::Point np(p.x + 1, p.y); // right
        if (floodShouldPushPoint(np, mat))
            toDo.push(np);
        np.x = p.x - 1;
        np.y = p.y; // left
        if (floodShouldPushPoint(np, mat))
            toDo.push(np);
        np.x = p.x;
        np.y = p.y + 1; // down
        if (floodShouldPushPoint(np, mat))
            toDo.push(np);
        np.x = p.x;
        np.y = p.y - 1; // up
        if (floodShouldPushPoint(np, mat))
            toDo.push(np);
        // kill it
        mat.at<float>(p) = 0.0f;
        mask.at<uchar>(p) = 0;
    }
    return mask;
}

cv::Point FACEManager::findEyeCenter(cv::Mat rgbeye, cv::Rect eye,
                                     std::string debugWindow) {
    cv::Mat eyeROI;
    cv::cvtColor(rgbeye, eyeROI, CV_BGR2GRAY);

    // cv::Mat eyeROIUnscaled = face(eye);
    // cv::Mat eyeROI;
    // scaleToFastSize(eyeROIUnscaled, eyeROI);
    // draw eye region
    // rectangle(face,eye,1234);
    //-- Find the gradient
    cv::Mat gradientX = computeMatXGradient(eyeROI);
    cv::Mat gradientY = computeMatXGradient(eyeROI.t()).t();
    //-- Normalize and threshold the gradient
    // compute all the magnitudes
    cv::Mat mags = matrixMagnitude(gradientX, gradientY);
    // compute the threshold
    double gradientThresh = computeDynamicThreshold(mags, kGradientThreshold);
    // double gradientThresh = kGradientThreshold;
    // double gradientThresh = 0;
    // normalize
    for (int y = 0; y < eyeROI.rows; ++y) {
        double *Xr = gradientX.ptr<double>(y), *Yr = gradientY.ptr<double>(y);
        const double *Mr = mags.ptr<double>(y);
        for (int x = 0; x < eyeROI.cols; ++x) {
            double gX = Xr[x], gY = Yr[x];
            double magnitude = Mr[x];
            if (magnitude > gradientThresh) {
                Xr[x] = gX / magnitude;
                Yr[x] = gY / magnitude;
            } else {
                Xr[x] = 0.0;
                Yr[x] = 0.0;
            }
        }
    }
    // imshow(debugWindow,gradientX);
    //-- Create a blurred and inverted image for weighting
    cv::Mat weight;
    GaussianBlur(eyeROI, weight, cv::Size(kWeightBlurSize, kWeightBlurSize), 0,
                 0);
    for (int y = 0; y < weight.rows; ++y) {
        unsigned char *row = weight.ptr<unsigned char>(y);
        for (int x = 0; x < weight.cols; ++x) {
            row[x] = (255 - row[x]);
        }
    }
    // imshow(debugWindow,weight);
    //-- Run the algorithm!
    cv::Mat outSum = cv::Mat::zeros(eyeROI.rows, eyeROI.cols, CV_64F);
    // for each possible gradient location
    // Note: these loops are reversed from the way the paper does them
    // it evaluates every possible center for each gradient location instead of
    // every possible gradient location for every center.
    //printf("Eye Size: %ix%i\n", outSum.cols, outSum.rows);
    for (int y = 0; y < weight.rows; ++y) {
        const double *Xr = gradientX.ptr<double>(y),
            *Yr = gradientY.ptr<double>(y);
        for (int x = 0; x < weight.cols; ++x) {
            double gX = Xr[x], gY = Yr[x];
            if (gX == 0.0 && gY == 0.0) {
                continue;
            }
            testPossibleCentersFormula(x, y, weight, gX, gY, outSum);
        }
    }
    // scale all the values down, basically averaging them
    double numGradients = (weight.rows * weight.cols);
    cv::Mat out;
    outSum.convertTo(out, CV_32F, 1.0 / numGradients);
    // imshow(debugWindow,out);
    //-- Find the maximum point
    cv::Point maxP;
    double maxVal;
    cv::minMaxLoc(out, NULL, &maxVal, NULL, &maxP);
    //-- Flood fill the edges
    if (kEnablePostProcess) {
        cv::Mat floodClone;
        // double floodThresh = computeDynamicThreshold(out, 1.5);
        double floodThresh = maxVal * kPostProcessThreshold;
        cv::threshold(out, floodClone, floodThresh, 0.0f, cv::THRESH_TOZERO);
        // if (kPlotVectorField) {
        // plotVecField(gradientX, gradientY, floodClone);
        // imwrite("eyeFrame.png", eyeROIUnscaled);
        //}
        cv::Mat mask = floodKillEdges(floodClone);
        // imshow(debugWindow + " Mask",mask);
        // imshow(debugWindow,out);
        // redo max
        cv::minMaxLoc(out, NULL, &maxVal, NULL, &maxP, mask);
    }
    // return unscalePoint(maxP, eye);
    return maxP;
}
