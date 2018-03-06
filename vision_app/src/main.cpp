#include "llvm/StringRef.h"
#include "ntcore.h"
#include <iostream>

#include <llvm/raw_ostream.h>
#include <cscore.h>
#include <opencv2/core.hpp>
#include "getContours.h"
#include "tape.h"

int main() {
  auto inst = NT_GetDefaultInstance();
  //auto distanceEntry = 0;
  (void)inst;
  //NT_GetDefaultInstance().StartClientTeam(4534);
  nt::StartClientTeam(inst, 4534, 1735); // 1735 is default port.
  cs::UsbCamera camera{"usbcam", 0};
  int width = 320;
  int height = 240;
  double fieldOfViewX = (16 * (68.5 / (sqrt(337))));
  double fieldOfViewY = (9 * (68.5 / (sqrt(337))));
  // converts standard aspect ratio of the camera into degrees
  double degreesToRadians = ((atan(1) * 4) / 180);
  double inchesOffCenterCamera = 6;
  int fps = 10;
  //int debugCountInny = 0;
  //int debugCountOutty = 0;
  grip::getContours GetContours = grip::getContours();
  grip::tape Tape = grip::tape();
  camera.SetVideoMode(cs::VideoMode::kMJPEG, width, height, fps);
  cs::MjpegServer mjpegServer{"httpserver", 8081};
  mjpegServer.SetSource(camera);
  cs::CvSink cvsink{"cvsink"};
  cvsink.SetSource(camera);
  cs::CvSource filterimage{"filterimage", cs::VideoMode::kMJPEG, width, height, fps};
  cs::MjpegServer fiMjpegServer{"fihttpserver", 8082};
  fiMjpegServer.SetSource(filterimage);
  cs::CvSource finale{"finale", cs::VideoMode::kMJPEG, width, height, fps};
  cs::MjpegServer cvMjpegServer{"finalhttpserver", 8083};
  cvMjpegServer.SetSource(finale);

  nt::SetEntryValue ("/CameraPublisher/usbcam/streams", nt::Value::MakeStringArray(llvm::ArrayRef<std::string> ({"http://10.45.34.55:8081/?action=stream"})));
  nt::SetEntryValue ("/CameraPublisher/filtered/streams", nt::Value::MakeStringArray(llvm::ArrayRef<std::string> ({"http://10.45.34.55:8082/?action=stream"})));
  nt::SetEntryValue ("/CameraPublisher/final/streams", nt::Value::MakeStringArray(llvm::ArrayRef<std::string> ({"http://10.45.34.55:8083/?action=stream"})));


  cv::Mat test;
  cv::Mat flip;
  for (;;){
    uint64_t time = cvsink.GrabFrame(test);
    if (time == 0) {
      std::cout << "error: " << cvsink.GetError() << std::endl;
      continue;
    }

    //std::cout << "Outer = " << ++debugCountOutty << std::endl;

    GetContours.Process(test);
    Tape.Process(test);
    std::cout << "got frame at time " << time << " size " << test.size()
              << std::endl;
    //cv::flip(test, flip, 0);


    int largestArea = 0;
    int secondLargestArea = 0;
    int largestAreaIndex = -1;
    int secondLargestAreaIndex = -1;
    //char* target = nt::GetEntryValue("vision/target")->value().data.v_string.str;
    //std::string findTarget(target);
    std::string findTarget = "cube";
    if (findTarget == "cube") {

    std::vector<std::vector<cv::Point> > contours = *GetContours.GetFindContoursOutput();
    // std::vector<std::vector<cv::Point> >::const_iterator i;

    // for (i=contours.begin(); i!=contours.end(); ++i) {
        // *i =
        // double area = contourArea(*i);
    for (unsigned int i = 0; i < contours.size(); i++) {
        //convexHull(contours[i], contours[i]);
        double area = contourArea(contours[i]);
     //std::cout << "Inner = " << ++debugCountInny << std::endl;
       if (area > largestArea) {
            secondLargestArea = largestArea;
            secondLargestAreaIndex = largestAreaIndex;
            largestArea = area;
            largestAreaIndex = i;
        }

        //std::cout << "largestAreaIndex: " << largestAreaIndex << std::endl;
        }
    if (largestAreaIndex > -1) {
        //std::cout << "largestArea: " << largestArea << std::endl;

        std::vector<cv::Point> nearContour = contours[largestAreaIndex];

        cv::Scalar color( rand()&255, rand()&255, rand()&255 );
        drawContours(test, contours, largestAreaIndex, color);
        //delete(&contours);
        cv::Rect nearBox = boundingRect(nearContour);

        rectangle(test, nearBox.tl(), nearBox.br(), color);

        std::cout << "Top left: (" << nearBox.tl().x << ", " << nearBox.tl().y << ")" << std::endl;
        std::cout << "Top right: (" << nearBox.br().x << ", " << nearBox.tl().y << ")" << std::endl;
        std::cout << "Bottom left: (" << nearBox.tl().x << ", " << nearBox.br().y << ")" << std::endl;
        std::cout << "Bottom right: (" << nearBox.br().x << ", " << nearBox.br().y << ")" << std::endl;
        //prints where the corners of the box are located in the field of view with pixel coordinates

        cv::Point center = cv::Point((nearBox.br().x - ((nearBox.br().x - nearBox.tl().x) / 2)), (nearBox.br().y - ((nearBox.br().y - nearBox.tl().y) / 2)));
        std::cout << "Center: (" << center.x << ", " << center.y << ")" << std::endl;

        int pixelsOffCenterX = ((width / 2) - center.x);
        int pixelsOffCenterY = ((height / 2) - center.y);

        double magicRatio = (11.0 / (nearBox.br().y - nearBox.tl().y));
        //conversion from pixels to inches using the height of the box
        double inchesOffCenterX = (pixelsOffCenterX * magicRatio);

        std::cout << "Inches off center: " << inchesOffCenterX << std::endl;
        //std::shared_ptr<nt::Value> NTOffCenter = (new nt::Value())
        //NTOffCenter.reset(inchesOffCenterX);
        nt::SetEntryValue ("vision/InchesOffCenter", nt::Value::MakeDouble(inchesOffCenterX));
        double distanceToBox = ((magicRatio * height) / (2.0 * tan((fieldOfViewY * degreesToRadians) / 2)));
        std::cout << "Distance to box: " << distanceToBox << std::endl;
        nt::SetEntryValue ("vision/cubeDistance", nt::Value::MakeDouble(distanceToBox));
        //how far away the box is in inches
		double straightDistanceToBox = (sqrt((distanceToBox * distanceToBox) - (inchesOffCenterX * inchesOffCenterX)));
		double inchesOffCenterBox = (inchesOffCenterX - inchesOffCenterCamera);
		
        //double angleOfBox = atan2(inchesOffCenterX, distanceToBox);
		double cubeAngle = atan2(inchesOffCenterBox, straightDistanceToBox);
		nt::SetEntryValue ("vision/cubeAngle", nt::Value::MakeDouble(cubeAngle));
        double pythagDistanceToBox = (sqrt((inchesOffCenterX * inchesOffCenterX) + (distanceToBox * distanceToBox)));
        nt::SetEntryValue ("vision/PythagDistanceToBox", nt::Value::MakeDouble(pythagDistanceToBox));
        //calculation for distance to box when it is not located in the center of the screen

        std::cout << "Angled distance to box: " << pythagDistanceToBox << std::endl;
    }

    finale.PutFrame(test);
    filterimage.PutFrame(*GetContours.GetMaskOutput());
    }

    else {
    std::cout << "Looking for switch" << std::endl;
    std::vector<std::vector<cv::Point> > contours = *Tape.GetConvexHullsOutput();
    // std::vector<std::vector<cv::Point> >::const_iterator i;

    // for (i=contours.begin(); i!=contours.end(); ++i) {
        // *i =
        // double area = contourArea(*i);
    for (unsigned int i = 0; i < contours.size(); i++) {
        //convexHull(contours[i], contours[i]);
        double area = contourArea(contours[i]);
     //std::cout << "Inner = " << ++debugCountInny << std::endl;
       if (area > largestArea) {
            secondLargestArea = largestArea;
            secondLargestAreaIndex = largestAreaIndex;
            largestArea = area;
            largestAreaIndex = i;
        }
        else if (area > secondLargestArea) {
            secondLargestArea = area;
            secondLargestAreaIndex = i;
        }

        }
        std::cout << "largestAreaIndex: " << largestAreaIndex << std::endl;
        std::cout << "secondLargestAreaIndex: " << secondLargestAreaIndex << std::endl;
    if (secondLargestAreaIndex > -1) {
        std::cout << "largestArea: " << largestArea << std::endl;

        std::vector<cv::Point> nearContour = contours[largestAreaIndex];
        std::vector<cv::Point> farContour = contours[secondLargestAreaIndex];

        cv::Scalar color( rand()&255, rand()&255, rand()&255 );
        drawContours(test, contours, largestAreaIndex, color);
        drawContours(test, contours, secondLargestAreaIndex, color);
        //delete(&contours);
        cv::Rect nearBox = boundingRect(nearContour);
        cv::Rect farBox = boundingRect(farContour);

        rectangle(test, nearBox.tl(), farBox.br(), color);

        int leftX = std::min(nearBox.tl().x, farBox.tl().x);
        int rightX = std::max(nearBox.br().x, farBox.br().x);
        int topY = std::min(nearBox.tl().y, farBox.tl().y);
        int bottomY = std::max(nearBox.br().y, farBox.br().y);

        std::cout << "Top left: (" << leftX << ", " << topY << ")" << std::endl;
        std::cout << "Top right: (" << rightX << ", " << topY << ")" << std::endl;
        std::cout << "Bottom left: (" << leftX << ", " << bottomY << ")" << std::endl;
        std::cout << "Bottom right: (" << rightX << ", " << bottomY << ")" << std::endl;
        //prints where the corners of the switch are located in the field of view with pixel coordinates

        cv::Point center = cv::Point((rightX - ((rightX - leftX) / 2)), (bottomY - ((bottomY - topY) / 2)));
        std::cout << "Center: (" << center.x << ", " << center.y << ")" << std::endl;

        int pixelsOffCenterX = ((width / 2) - center.x);
        int pixelsOffCenterY = ((height / 2) - center.y);

        double magicRatio = (16.0 / (bottomY - topY));
        //conversion from pixels to inches using the height of the tape
        double inchesOffCenterX = (pixelsOffCenterX * magicRatio);

        std::cout << "Inches off center: " << inchesOffCenterX << std::endl;
        //std::shared_ptr<nt::Value> NTOffCenter = (new nt::Value())
        //NTOffCenter.reset(inchesOffCenterX);
        nt::SetEntryValue ("vision/InchesOffCenter", nt::Value::MakeDouble(inchesOffCenterX));
        double distanceToTape = ((magicRatio * height) / (2.0 * tan((fieldOfViewY * degreesToRadians) / 2)));
        std::cout << "Distance to tape: " << distanceToTape << std::endl;
        nt::SetEntryValue ("vision/switchDistance", nt::Value::MakeDouble(distanceToTape));
        //how far away the switch is in inches
		double straightDistanceToTape = (sqrt((distanceToTape * distanceToTape) - (inchesOffCenterX * inchesOffCenterX)));
		double inchesOffCenterTape = (inchesOffCenterX - inchesOffCenterCamera);

        double angleOfTape = atan2(inchesOffCenterTape, straightDistanceToTape);
        nt::SetEntryValue ("vision/switchAngle", nt::Value::MakeDouble(angleOfTape));
        //double pythagDistanceToTape = (sqrt((inchesOffCenterX * inchesOffCenterX) + (distanceToTape * distanceToTape)));
        //nt::SetEntryValue ("vision/PythagDistanceToTape", nt::Value::MakeDouble(pythagDistanceToTape));
        //calculation for distance to switch when it is not located in the center of the screen

        std::cout << "Angled distance to tape: " << pythagDistanceToTape << std::endl;
    }

    finale.PutFrame(test);
    filterimage.PutFrame(*Tape.GetMaskOutput());
    }
    }
  return 0;
}
