#pragma once
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d.hpp>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <map>
#include <vector>
#include <string>
#include <math.h>

namespace grip {

/**
* A representation of the different types of blurs that can be used.
*
*/
enum TapeBlurType {
	TAPEBOX, TAPEGAUSSIAN, TAPEMEDIAN, TAPEBILATERAL
};
/**
* tape class.
*
* An OpenCV pipeline generated by GRIP.
*/
class tape {
	private:
		cv::Mat blurOutput;
		cv::Mat hslThresholdOutput;
		cv::Mat maskOutput;
		std::vector<std::vector<cv::Point> > findContoursOutput;
		std::vector<std::vector<cv::Point> > convexHullsOutput;
		void blur(cv::Mat &, TapeBlurType &, double , cv::Mat &);
		void hslThreshold(cv::Mat &, double [], double [], double [], cv::Mat &);
		void mask(cv::Mat &, cv::Mat &, cv::Mat &);
		void findContours(cv::Mat &, bool , std::vector<std::vector<cv::Point> > &);
		void convexHulls(std::vector<std::vector<cv::Point> > &, std::vector<std::vector<cv::Point> > &);

	public:
		tape();
		void Process(cv::Mat& source0);
		cv::Mat* GetBlurOutput();
		cv::Mat* GetHslThresholdOutput();
		cv::Mat* GetMaskOutput();
		std::vector<std::vector<cv::Point> >* GetFindContoursOutput();
		std::vector<std::vector<cv::Point> >* GetConvexHullsOutput();
};


} // end namespace grip


