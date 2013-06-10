/*
 * File name: CCircleDetect.h
 * Date:      2010
 * Author:   Tom Krajnik, Matias Nitsche
 */

#ifndef __CCIRCLEDETECTOR_H__
#define __CCIRCLEDETECTOR_H__

//#include "CTimer.h"
#include <opencv2/opencv.hpp>
#include <math.h>
#include <vector>

namespace cv {
  class CircleDetector
  {
    public:
      class Circle;
      CircleDetector(int width, int height, float diameter_ratio = (5/14.0), int color_precision = 32, int color_step = 8);
      ~CircleDetector();
      
      Circle detect(const cv::Mat& image, const Circle& previous_circle = cv::CircleDetector::Circle());
      bool examineCircle(const cv::Mat& image, Circle& circle, int ii, float areaRatio);
      
      void improveEllipse(const cv::Mat& image, Circle& c);

      bool changeThreshold();
      bool debug,draw,drawAll;
    private:
    
      int max_circles, color_precision, color_step;

      bool track, lastTrackOK;
      int maxFailed;
      int numFailed;
      int threshold; 

      int minSize; 
      int lastThreshold; 
      int thresholdBias; 
      int maxThreshold; 

      float diameterRatio;
      int thresholdStep;
      float circularTolerance;
      float ratioTolerance;
      float centerDistanceToleranceRatio;
      int centerDistanceToleranceAbs;

      float outerAreaRatio,innerAreaRatio,areasRatio;
      int queueStart,queueEnd,queueOldStart,numSegments;
      int width,height,len,siz;
      int expand[4];
      std::vector<int> buffer, queue;
      unsigned char *ptr;
      //CTimer timer;
      int tima,timb,timc,timd,sizer,sizerAll;

      void cleanup_buffer(const Circle& c, bool fast_cleanup);
      
    public:
      class Circle {
        public:
          Circle(void) {
            x = y = 0;
            round = valid = false;
          }
          
          float x;
          float y;
          int size;
          int maxy,maxx,miny,minx;
          int mean;
          int type;
          float roundness;
          float bwRatio;
          bool round;
          bool valid;
          float m0,m1;
          float v0,v1;
          
          void draw(cv::Mat& image, const std::string& text = std::string(), cv::Scalar color = cv::Scalar(0,255,0)) const;
      };
  };
}

#endif

/* end of CCircleDetect.h */
