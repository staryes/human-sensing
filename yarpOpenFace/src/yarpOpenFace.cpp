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

#include <yarp/sig/all.h>
#include "yarpOpenFace.h"

/**********************************************************/
bool FACEModule::configure(yarp::os::ResourceFinder &rf)
{
    rf.setVerbose();
    moduleName = rf.check("name", yarp::os::Value("yarpOpenFace"), "module name (string)").asString();
    predictorFile = rf.check("yarpOpenFaceFile", yarp::os::Value("shape_predictor_68_face_landmarks.dat"), "path name (string)").asString();

    std::string firstStr = rf.findFile(predictorFile.c_str());

    setName(moduleName.c_str());

    handlerPortName =  "/";
    handlerPortName += getName();
    handlerPortName +=  "/rpc:i";

    cntxHomePath = rf.getHomeContextPath().c_str();

    if (!rpcPort.open(handlerPortName.c_str()))
    {
        yError() << getName().c_str() << " Unable to open port " << handlerPortName.c_str();
        return false;
    }

    attach(rpcPort);
    closing = false;

    /* create the thread and pass pointers to the module parameters */
    faceManager = new FACEManager( moduleName, firstStr, cntxHomePath );

    /* now start the thread to do the work */
    faceManager->open();

    return true ;
}

/**********************************************************/
bool FACEModule::interruptModule()
{
    rpcPort.interrupt();
    return true;
}

/**********************************************************/
bool FACEModule::close()
{
    //rpcPort.close();
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
bool FACEModule::updateModule()
{
    return !closing;
}

/**********************************************************/
double FACEModule::getPeriod()
{
    return 0.01;
}

/************************************************************************/
bool FACEModule::attach(yarp::os::RpcServer &source)
{
    return this->yarp().attachAsServer(source);
}

/**********************************************************/
bool FACEModule::display(const std::string& element, const std::string& value)
{
    bool returnVal = false;

    if (element == "landmarks" || element == "points" || element == "labels" || element == "dark-mode")
    {
        if (element == "landmarks")
        {
            if (value=="on")
            {
                faceManager->displayLandmarks=true;
                returnVal = true;
            }
            else if (value=="off")
            {
                faceManager->displayLandmarks = false;
                returnVal = true;
            }
            else
                yInfo() << "error setting value for landmarks";
        }
        if (element == "points")
        {
            if (value=="on")
            {
                faceManager->displayPoints=true;
                returnVal = true;
            }
            else if (value=="off")
            {
                faceManager->displayPoints = false;
                returnVal = true;
            }
            else
                yInfo() << "error setting value for points";
        }
        if (element == "labels")
        {
            if (value=="on")
            {
                faceManager->displayLabels=true;
                returnVal = true;
            }
            else if (value=="off")
            {
                faceManager->displayLabels = false;
                returnVal = true;
            }
            else
                yInfo() << "error setting value for labels";
        }
        if (element == "dark-mode")
        {
            if (value=="on")
            {
                faceManager->displayDarkMode=true;
                returnVal = true;
            }
            else if (value=="off")
            {
                faceManager->displayDarkMode = false;
                returnVal = true;
            }
            else
                yInfo() << "error setting value for darkMode";
        }
        //yInfo() << "should now display \"landmarks\" " << faceManager->displayLandmarks << "\"points\"" << faceManager->displayPoints << "\"labels\"" << faceManager->displayLabels  << "\"dark-mode\"" << faceManager->displayDarkMode;
    }
    else
    {
        returnVal = false;
        yInfo() << "Error in display request";
    }
    return returnVal;
}

/**********************************************************/
bool FACEModule::quit()
{
    closing = true;
    return true;
}

/**********************************************************/
FACEManager::~FACEManager()
{
}

/**********************************************************/
FACEManager::FACEManager( const std::string &moduleName,  const std::string &predictorFile, const std::string &cntxHomePath)
{
    yDebug() << "initialising Variables";
    this->moduleName = moduleName;
    this->predictorFile = predictorFile;
    this->cntxHomePath = cntxHomePath;
    yInfo() << "contextPATH = " << cntxHomePath.c_str();
}

/**********************************************************/
bool FACEManager::open()
{
    this->useCallback();

    //create all ports
    inImgPortName = "/" + moduleName + "/image:i";
    BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> >::open( inImgPortName.c_str() );

    outImgPortName = "/" + moduleName + "/image:o";
    imageOutPort.open( outImgPortName.c_str() );

    outTargetPortName = "/" + moduleName + "/target:o";
    targetOutPort.open( outTargetPortName.c_str() );

    outLandmarksPortName = "/" + moduleName + "/landmarks:o";
    landmarksOutPort.open( outLandmarksPortName.c_str() );

    //yDebug() << "path is: " << predictorFile.c_str();

    //faceDetector = dlib::get_frontal_face_detector();
    //dlib::deserialize(predictorFile.c_str()) >> sp;

    FACEModels fms;
    //fms.isModelsLoaded = fms.loadModels();

    color = cv::Scalar( 0, 255, 0 );

    displayLandmarks = true;
    displayPoints = false;
    displayLabels = false;
    displayDarkMode = false;

    return true;
}

/**********************************************************/
void FACEManager::close()
{
    mutex.wait();
    yDebug() <<"now closing ports...";
    imageOutPort.writeStrict();
    imageOutPort.close();
    imageInPort.close();
    targetOutPort.close();
    landmarksOutPort.close();
    BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> >::close();
    mutex.post();
    yDebug() <<"finished closing the read port...";
}

/**********************************************************/
void FACEManager::interrupt()
{
    yDebug() << "cleaning up...";
    yDebug() << "attempting to interrupt ports";
    BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> >::interrupt();
    yDebug() << "finished interrupt ports";
}

/**********************************************************/
void FACEManager::onRead(yarp::sig::ImageOf<yarp::sig::PixelRgb> &img)
{
    mutex.wait();
    yarp::sig::ImageOf<yarp::sig::PixelRgb> &outImg  = imageOutPort.prepare();
    yarp::os::Bottle &target=targetOutPort.prepare();
    yarp::os::Bottle &landmarks=landmarksOutPort.prepare();
    target.clear();

    // Get the image from the yarp port
    imgMat = cv::cvarrToMat((IplImage*)img.getIplImage());

    // Change to dlib's image format. No memory is copied.
    //dlib::cv_image<dlib::bgr_pixel> dlibimg(imgMat);
//-------------------------
// Load the models

    LandmarkDetector::FaceModelParameters det_parameters; //test

    // The modules that are being used for tracking
    cout << "Loading the model" << endl;
    LandmarkDetector::CLNF face_model(det_parameters.model_location);

    cout << "Model loaded" << endl;

	// Load facial feature extractor and AU analyser (make sure it is static)
	FaceAnalysis::FaceAnalyserParameters face_analysis_params("-yarp");
    face_analysis_params.OptimizeForImages();
	FaceAnalysis::FaceAnalyser face_analyser(face_analysis_params);

	// If bounding boxes not provided, use a face detector
	cv::CascadeClassifier classifier(det_parameters.haar_face_detector_location);
	dlib::frontal_face_detector face_detector_hog = dlib::get_frontal_face_detector();
	LandmarkDetector::FaceDetectorMTCNN face_detector_mtcnn(det_parameters.mtcnn_face_detector_location);

// If can't find MTCNN face detector, default to HOG one
    if (det_parameters.curr_face_detector == LandmarkDetector::FaceModelParameters::MTCNN_DETECTOR && face_detector_mtcnn.empty())
    {
        cout << "INFO: defaulting to HOG-SVM face detector" << endl;
        det_parameters.curr_face_detector = LandmarkDetector::FaceModelParameters::HOG_SVM_DETECTOR;
    }

    // A utility for visualizing the results
    Utilities::Visualizer visualizer(true, true, true, true);


    cv::Mat rgb_image;

    rgb_image = imgMat;

    float fx = 500.0;
    float fy = 500.0;
    float cx = imgMat.rows/2;
    float cy = imgMat.cols/2;

    if (!face_model.eye_model)
	{
		cout << "WARNING: no eye model found" << endl;
	}

	if (face_analyser.GetAUClassNames().size() == 0 && face_analyser.GetAUClassNames().size() == 0)
	{
		cout << "WARNING: no Action Unit models found" << endl;
	}

	cout << "Starting tracking" << endl;

	if (!rgb_image.empty())
	{
        //std::vector<std::string> argument = "-yarp";

		Utilities::RecorderOpenFaceParameters recording_params(false, false, false, false, false, false,false, false, false, false, false, false, fx, fy, cx, cy, 30);

		if (!face_model.eye_model)
		{
			recording_params.setOutputGaze(false);
		}
		//Utilities::RecorderOpenFace open_face_rec(name, recording_params, arguments); //XX

		visualizer.SetImage(rgb_image, fx, fy, cx, cy);

		// Making sure the image is in uchar grayscale (some face detectors use RGB, landmark detector uses grayscale)
		cv::Mat_<uchar> grayscale_image;
        cv::cvtColor(rgb_image, grayscale_image, CV_BGR2GRAY);


		// Detect faces in an image
		vector<cv::Rect_<float> > face_detections;

		//if (image_reader.has_bounding_boxes) //XXX
		{
            //	face_detections = image_reader.GetBoundingBoxes();
		}
		//else
		{
			if (det_parameters.curr_face_detector == LandmarkDetector::FaceModelParameters::HOG_SVM_DETECTOR)
			{
				vector<float> confidences;
				LandmarkDetector::DetectFacesHOG(face_detections, grayscale_image, face_detector_hog, confidences);
			}
			else if (det_parameters.curr_face_detector == LandmarkDetector::FaceModelParameters::HAAR_DETECTOR)
			{
				LandmarkDetector::DetectFaces(face_detections, grayscale_image, classifier);
			}
			else
			{
				vector<float> confidences;
				LandmarkDetector::DetectFacesMTCNN(face_detections, rgb_image, face_detector_mtcnn, confidences);
			}
		}

		// Detect landmarks around detected faces
		int face_det = 0;
		// perform landmark detection for every face detected
		for (size_t face = 0; face < face_detections.size(); ++face)
		{

			// if there are multiple detections go through them
			bool success = LandmarkDetector::DetectLandmarksInImage(rgb_image, face_detections[face], face_model, det_parameters, grayscale_image);

			// Estimate head pose and eye gaze
			cv::Vec6d pose_estimate = LandmarkDetector::GetPose(face_model, fx, fy, cx, cy);

			// Gaze tracking, absolute gaze direction
			cv::Point3f gaze_direction0(0, 0, -1);
			cv::Point3f gaze_direction1(0, 0, -1);
			cv::Vec2f gaze_angle(0, 0);

			if (face_model.eye_model)
			{
				GazeAnalysis::EstimateGaze(face_model, gaze_direction0, fx, fy, cx, cy, true);
				GazeAnalysis::EstimateGaze(face_model, gaze_direction1, fx, fy, cx, cy, false);
				gaze_angle = GazeAnalysis::GetGazeAngle(gaze_direction0, gaze_direction1);
			}

			cv::Mat sim_warped_img;
			cv::Mat_<double> hog_descriptor; int num_hog_rows = 0, num_hog_cols = 0;

			// Perform AU detection and HOG feature extraction, as this can be expensive only compute it if needed by output or visualization
			if (recording_params.outputAlignedFaces() || recording_params.outputHOG() || recording_params.outputAUs() || visualizer.vis_align || visualizer.vis_hog)
			{
				face_analyser.PredictStaticAUsAndComputeFeatures(rgb_image, face_model.detected_landmarks);
				face_analyser.GetLatestAlignedFace(sim_warped_img);
				face_analyser.GetLatestHOG(hog_descriptor, num_hog_rows, num_hog_cols);
			}

			// Displaying the tracking visualizations
			visualizer.SetObservationFaceAlign(sim_warped_img);
			visualizer.SetObservationHOG(hog_descriptor, num_hog_rows, num_hog_cols);
			visualizer.SetObservationLandmarks(face_model.detected_landmarks, 1.0, face_model.GetVisibilities()); // Set confidence to high to make sure we always visualize
			visualizer.SetObservationPose(pose_estimate, 1.0);
			visualizer.SetObservationGaze(gaze_direction0, gaze_direction1, LandmarkDetector::CalculateAllEyeLandmarks(face_model), LandmarkDetector::Calculate3DEyeLandmarks(face_model, fx, fy, cx, cy), face_model.detection_certainty);
			visualizer.SetObservationActionUnits(face_analyser.GetCurrentAUsReg(), face_analyser.GetCurrentAUsClass());

			// Setting up the recorder output
			// open_face_rec.SetObservationHOG(face_model.detection_success, hog_descriptor, num_hog_rows, num_hog_cols, 31); // The number of channels in HOG is fixed at the moment, as using FHOG
			// open_face_rec.SetObservationActionUnits(face_analyser.GetCurrentAUsReg(), face_analyser.GetCurrentAUsClass());
			// open_face_rec.SetObservationLandmarks(face_model.detected_landmarks, face_model.GetShape(fx, fy, cx, cy),
			// 	face_model.params_global, face_model.params_local, face_model.detection_certainty, face_model.detection_success);
			// open_face_rec.SetObservationPose(pose_estimate);
			// open_face_rec.SetObservationGaze(gaze_direction0, gaze_direction1, gaze_angle, LandmarkDetector::CalculateAllEyeLandmarks(face_model), LandmarkDetector::Calculate3DEyeLandmarks(face_model, fx, fy, cx, cy));
			// open_face_rec.SetObservationFaceAlign(sim_warped_img);
			// open_face_rec.SetObservationFaceID(face);
			// open_face_rec.WriteObservation();

		}
		if (face_detections.size() > 0)
		{
			//visualizer.ShowObservation(); // XXX turn off the imshows
		}

		// open_face_rec.SetObservationVisualization(visualizer.GetVisImage());
		// open_face_rec.WriteObservationTracked();

		// open_face_rec.Close();

		// Grabbing the next frame in the sequence
		//rgb_image = image_reader.GetNextImage();

	}


//-------------------------
    // Convert the opencv image to dlib format
    // dlib::array2d<dlib::rgb_pixel> dlibimg;
    // assign_image(dlibimg, dlib::cv_image<dlib::bgr_pixel>(imgMat));
    // //dlib::cv_image<dlib::bgr_pixel> dlibimg(imgMat);

    // // Make the image larger so we can detect small faces. 2x
    // pyramid_up(dlibimg);

    // // Now tell the face detector to give us a list of bounding boxes
    // // around all the faces in the image.
    // int count = 0;
    // std::vector<dlib::rectangle> dets = faceDetector(dlibimg);

    // std::vector<dlib::full_object_detection> shapes;

    // for (unsigned long j = 0; j < dets.size(); ++j)
    // {
    //     dlib::full_object_detection shape = sp(dlibimg, dets[j]);
    //     shapes.push_back(shape);
    // }

    // std::vector<std::pair<int, double >> idTargets;

    // for (unsigned long i = 0; i < dets.size(); ++i)
    // {
    //     DLIB_CASSERT(shapes[i].num_parts() == 68,
    //                  "\t std::vector<image_window::overlay_line> render_face_detections()"
    //                  << "\n\t Invalid inputs were given to this function. "
    //                  << "\n\t dets["<<i<<"].num_parts():  " << shapes[i].num_parts()
    //                  );

    //     const dlib::full_object_detection& d = shapes[i];

    //     if (displayDarkMode)
    //         imgMat.setTo(cv::Scalar(0, 0, 0));

    //     if (displayLandmarks)
    //         drawLandmarks(imgMat, d);

    //     //if (landmarksOutPort.getOutputCount()>0)
    //     //{
    //     landmarks.clear();
    //     yarp::os::Bottle &landM = landmarks.addList();
    //     for (int f=1; f<shapes[i].num_parts(); f++)
    //     {

    //         if (f != 17 || f != 22 || f != 27 || f != 42 || f != 48)
    //         {
    //             yarp::os::Bottle &temp = landM.addList();
    //             temp.addInt(d.part(f).x()/2);
    //             temp.addInt(d.part(f).y()/2);
    //         }
    //     }
    // //}

    //     if (displayPoints || displayLabels)
    //     {
    //         int pointSize = landmarks.get(0).asList()->size();

    //         if (displayPoints)
    //         {
    //             for (size_t i = 0; i < pointSize; i++)
    //             {
    //                 int pointx = landmarks.get(0).asList()->get(i).asList()->get(0).asInt();
    //                 int pointy = landmarks.get(0).asList()->get(i).asList()->get(1).asInt();
    //                 cv::Point center(pointx, pointy);
    //                 circle(imgMat, center, 3, cv::Scalar(255, 0 , 0), -1, 8);
    //             }
    //         }
    //         if (displayLabels)
    //         {
    //             for (size_t i = 0; i < pointSize; i++)
    //             {
    //                 int pointx = landmarks.get(0).asList()->get(i).asList()->get(0).asInt();
    //                 int pointy = landmarks.get(0).asList()->get(i).asList()->get(1).asInt();
    //                 cv::Point center(pointx, pointy);
    //                 std::string s = std::to_string(i);
    //                 putText(imgMat, s, cvPoint(pointx, pointy), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.6, cvScalar(200,200,250), 1, CV_AA);
    //             }
    //         }
    //     }

    //     cv::Point pt1, pt2;
    //     pt1.x = dets[i].tl_corner().x()/2;
    //     pt1.y = dets[i].tl_corner().y()/2;
    //     pt2.x = dets[i].br_corner().x()/2;
    //     pt2.y = dets[i].br_corner().y()/2;

    //     rightEye.x = d.part(42).x()/2 + ((d.part(45).x()/2) - (d.part(42).x()/2))/2;
    //     rightEye.y = d.part(43).y()/2 + ((d.part(46).y()/2) - (d.part(43).y()/2))/2;
    //     leftEye.x  = d.part(36).x()/2 + ((d.part(39).x()/2) - (d.part(36).x()/2))/2;
    //     leftEye.y  = d.part(38).y()/2 + ((d.part(41).y()/2) - (d.part(38).y()/2))/2;

    //     //yDebug("rightEye %d %d leftEye %d %d ", rightEye.x, rightEye.y, leftEye.x, leftEye.y);

    //     //draw center of each eye
    //     circle(imgMat, leftEye , 2, cv::Scalar( 0, 0, 255 ), -1);
    //     circle(imgMat, rightEye , 2, cv::Scalar( 0, 0, 255 ), -1);

    //     double areaCalculation =0.0;
    //     areaCalculation = (std::fabs(pt2.x-pt1.x)*std::fabs(pt2.y-pt1.y));

    //     idTargets.push_back(std::make_pair(i, areaCalculation));
    // }

    // if (idTargets.size()>0)
    // {
    //     std::sort(idTargets.begin(), idTargets.end(), [](const std::pair<int, double> &left, const std::pair<int, double> &right) {
    //         return left.second > right.second;
    //     });
    // }

    // if (dets.size() > 0 )
    // {
    //     for (int i=0; i< idTargets.size(); i++)
    //     {
    //         cv::Point pt1, pt2;
    //         pt1.x = dets[idTargets[i].first].tl_corner().x()/2;
    //         pt1.y = dets[idTargets[i].first].tl_corner().y()/2;
    //         pt2.x = dets[idTargets[i].first].br_corner().x()/2;
    //         pt2.y = dets[idTargets[i].first].br_corner().y()/2;

    //         if (pt1.x < 2)
    //             pt1.x = 1;
    //         if (pt1.x > 318)
    //             pt1.x = 319;
    //         if (pt1.y < 2)
    //             pt1.y = 1;
    //         if (pt1.y > 238)
    //             pt1.y = 239;

    //         if (pt2.x < 2)
    //             pt2.x = 1;
    //         if (pt2.x > 318)
    //             pt2.x = 319;
    //         if (pt2.y < 2)
    //             pt2.y = 1;
    //         if (pt2.y > 238)
    //             pt2.y = 239;


    //         yarp::os::Bottle &pos = target.addList();
    //         pos.addDouble(pt1.x);
    //         pos.addDouble(pt1.y);
    //         pos.addDouble(pt2.x);
    //         pos.addDouble(pt2.y);

    //         cv::Point biggestpt1, biggestpt2;
    //         biggestpt1.x = dets[idTargets[0].first].tl_corner().x()/2;
    //         biggestpt1.y = dets[idTargets[0].first].tl_corner().y()/2;
    //         biggestpt2.x = dets[idTargets[0].first].br_corner().x()/2;
    //         biggestpt2.y = dets[idTargets[0].first].br_corner().y()/2;

    //         rectangle(imgMat, biggestpt1, biggestpt2, cv::Scalar( 0, 255, 0 ), 1, 8, 0);

    //         targetOutPort.write();
    //         if (landmarksOutPort.getOutputCount()>0)
    //             landmarksOutPort.write();
    //     }
    // }
    IplImage yarpImg = visualizer.GetVisImage();
    //IplImage yarpImg = imgMat;
    outImg.resize(yarpImg.width, yarpImg.height);
    cvCopy( &yarpImg, (IplImage *) outImg.getIplImage());

    imageOutPort.write();

    mutex.post();
}

// void FACEManager::drawLandmarks(cv::Mat &mat, const dlib::full_object_detection &d)
// {
//     //draw face contour, jaw
//     for (unsigned long i = 1; i <= 16; ++i)
//         line(mat, cv::Point(d.part(i).x()/2, d.part(i).y()/2), cv::Point(d.part(i-1).x()/2, d.part(i-1).y()/2),  color);

//     //draw right eyebrow
//     for (unsigned long i = 18; i <= 21; ++i)
//         line(mat, cv::Point(d.part(i).x()/2, d.part(i).y()/2), cv::Point(d.part(i-1).x()/2, d.part(i-1).y()/2),  color);

//     //draw left eyebrow
//     for (unsigned long i = 23; i <= 26; ++i)
//         line(mat, cv::Point(d.part(i).x()/2, d.part(i).y()/2), cv::Point(d.part(i-1).x()/2, d.part(i-1).y()/2),  color);

//     //draw nose
//     for (unsigned long i = 28; i <= 30; ++i)
//         line(mat, cv::Point(d.part(i).x()/2, d.part(i).y()/2), cv::Point(d.part(i-1).x()/2, d.part(i-1).y()/2),  color);

//     //draw nostrils
//     line(mat, cv::Point(d.part(30).x()/2, d.part(30).y()/2), cv::Point(d.part(35).x()/2, d.part(35).y()/2),  color);
//     for (unsigned long i = 31; i <= 35; ++i)
//         line(mat, cv::Point(d.part(i).x()/2, d.part(i).y()/2), cv::Point(d.part(i-1).x()/2, d.part(i-1).y()/2),  color);

//     //draw right eye
//     for (unsigned long i = 37; i <= 41; ++i)
//         line(mat, cv::Point(d.part(i).x()/2, d.part(i).y()/2), cv::Point(d.part(i-1).x()/2, d.part(i-1).y()/2),  color);
//     line(mat, cv::Point(d.part(36).x()/2, d.part(36).y()/2), cv::Point(d.part(41).x()/2, d.part(41).y()/2),  color);

//     //draw left eye
//     for (unsigned long i = 43; i <= 47; ++i)
//         line(mat, cv::Point(d.part(i).x()/2, d.part(i).y()/2), cv::Point(d.part(i-1).x()/2, d.part(i-1).y()/2),  color);
//     line(mat, cv::Point(d.part(42).x()/2, d.part(42).y()/2), cv::Point(d.part(47).x()/2, d.part(47).y()/2),  color);

//     //draw outer mouth
//     for (unsigned long i = 49; i <= 59; ++i)
//         line(mat, cv::Point(d.part(i).x()/2, d.part(i).y()/2), cv::Point(d.part(i-1).x()/2, d.part(i-1).y()/2),  color);
//     line(mat, cv::Point(d.part(48).x()/2, d.part(48).y()/2), cv::Point(d.part(59).x()/2, d.part(59).y()/2),  color);

//     //draw inner mouth
//     line(mat, cv::Point(d.part(60).x()/2, d.part(60).y()/2), cv::Point(d.part(67).x()/2, d.part(67).y()/2),  color);
//     for (unsigned long i = 61; i <= 67; ++i)
//         line(mat, cv::Point(d.part(i).x()/2, d.part(i).y()/2), cv::Point(d.part(i-1).x()/2, d.part(i-1).y()/2),  color);

// }
//empty line to make gcc happy

// bool FACEModels::loadModels(void)
// {
//     FACEModels fms;
//     fms.face_model(fms.det_parameters);

//     return 1;
// }

FACEModels::FACEModels()
{

}

FACEModels::~FACEModels()
{
}
