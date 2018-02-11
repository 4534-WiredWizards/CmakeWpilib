#include "llvm/StringRef.h"
#include "ntcore.h"
#include <iostream>

#include <llvm/raw_ostream.h>
#include <cscore.h>
#include <opencv2/core.hpp>
#include "getContours.h"

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
  int fps = 10;
  int debugCountInny = 0;
  int debugCountOutty = 0;
  grip::getContours GetContours = grip::getContours();
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
    std::cout << "got frame at time " << time << " size " << test.size()
              << std::endl;
    //cv::flip(test, flip, 0);


    int largestArea = 0;
    int largestAreaIndex = -1;

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

        cv::Point center = cv::Point((nearBox.br().x - ((nearBox.br().x - nearBox.tl().x) / 2)), (nearBox.br().y - ((nearBox.br().y - nearBox.tl().y) / 2)));
        std::cout << "Center: (" << center.x << ", " << center.y << ")" << std::endl;

        int pixelsOffCenterX = ((width / 2) - center.x);
        int pixelsOffCenterY = ((height / 2) - center.y);

        double magicRatio = (11.0 / (nearBox.br().y - nearBox.tl().y));
        double inchesOffCenterX = (pixelsOffCenterX * magicRatio);

        std::cout << "Inches off center: " << inchesOffCenterX << std::endl;
        //std::shared_ptr<nt::Value> NTOffCenter = (new nt::Value())
        //NTOffCenter.reset(inchesOffCenterX);
        nt::SetEntryValue ("vision/InchesOffCenter", nt::Value::MakeDouble(inchesOffCenterX));

    }
    finale.PutFrame(test);
    filterimage.PutFrame(*GetContours.GetMaskOutput());

    }

  return 0;
}
