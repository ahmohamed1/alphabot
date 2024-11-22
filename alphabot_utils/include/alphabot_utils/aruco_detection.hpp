#include <opencv2/opencv.hpp>
#include <opencv2/objdetect/aruco_detector.hpp>

using std::placeholders::_1;

class ArucoDetection {
public:
    ArucoDetection() {
        // Load the XML file
        cv::FileStorage fs("camera_calibration_params.xml", cv::FileStorage::READ);

        if (!fs.isOpened()) {
            std::cerr << "Failed to open file!" << std::endl;
        }

        // Read the camera matrix and distortion coefficients
        fs["camera_matrix"] >> camMatrix;
        fs["dist_coeffs"] >> distCoeffs;

        // Release the file storage
        fs.release();

        // Print loaded parameters
        std::cout << "Camera matrix:" << std::endl << camMatrix << std::endl;
        std::cout << "Distortion coefficients:" << std::endl << distCoeffs << std::endl;

        // set coordinate system
        markerLength = 0.05f; // Example length, you should set this appropriately
        objPoints.ptr<cv::Vec3f>(0)[0] = cv::Vec3f(-markerLength / 2.f, markerLength / 2.f, 0);
        objPoints.ptr<cv::Vec3f>(0)[1] = cv::Vec3f(markerLength / 2.f, markerLength / 2.f, 0);
        objPoints.ptr<cv::Vec3f>(0)[2] = cv::Vec3f(markerLength / 2.f, -markerLength / 2.f, 0);
        objPoints.ptr<cv::Vec3f>(0)[3] = cv::Vec3f(-markerLength / 2.f, -markerLength / 2.f, 0);
    }

    cv::Point2f track_aruco(cv::Mat image) {
        image.copyTo(imageCopy);
        // detect markers and estimate pose
        detector.detectMarkers(image, corners, ids, rejected);

        size_t nMarkers = corners.size();
        std::vector<cv::Vec3d> rvecs(nMarkers), tvecs(nMarkers);

        if (!ids.empty()) {
            // Calculate pose for each marker
            for (size_t i = 0; i < nMarkers; i++) {
                cv::solvePnP(objPoints, corners.at(i), camMatrix, distCoeffs, rvecs.at(i), tvecs.at(i));
            }
            // draw results
            image.copyTo(imageCopy);
            cv::aruco::drawDetectedMarkers(imageCopy, corners, ids);
            for (unsigned int i = 0; i < ids.size(); i++) {
                if (ids[i] == 23)
                {
                    cv::drawFrameAxes(imageCopy, camMatrix, distCoeffs, rvecs[i], tvecs[i], markerLength * 1.5f, 2);
                    std::cout << ids[i] << ": " << tvecs[i] << std::endl;
                }
            }
        }
        cv::imshow("out", imageCopy);
        char key = cv::waitKey(0);
        if (key == 'q') {
            // break; // This should be inside a loop
            return cv::Point2f(); // Return an appropriate value
        }
        return cv::Point2f(); // Return an appropriate value if not quitting
    }

private:
    // opencv variables 
    float markerLength;
    cv::Mat imageCopy;
    cv::Mat camMatrix, distCoeffs;
    cv::Mat objPoints = cv::Mat(4, 1, CV_32FC3);
    cv::aruco::DetectorParameters detectorParams = cv::aruco::DetectorParameters();
    cv::aruco::Dictionary dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
    cv::aruco::ArucoDetector detector = cv::aruco::ArucoDetector(dictionary, detectorParams);
    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f> > corners, rejected;
};