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

namespace cv {
  class CircleDetector
  {
    public:
      class Circle;
      CircleDetector(int width, int height, int max_circles = 10000, int color_precision = 32, int color_step = 8);
      ~CircleDetector();
      
      Circle detect(const cv::Mat& image, const Circle& previous_circle = cv::CircleDetector::Circle());
      bool examineCircle(const cv::Mat& image, Circle& circle, int ii, float areaRatio);

      bool changeThreshold();
      bool debug,draw,drawAll;
    private:
    
      int max_circles, color_precision, color_step;

      bool track;
      int maxFailed;
      int numFailed;
      int threshold; 

      int minSize; 
      int lastThreshold; 
      int thresholdBias; 
      int maxThreshold; 

      int thresholdStep;
      float circularTolerance;
      float ratioTolerance;
      float centerDistanceToleranceRatio;
      int centerDistanceToleranceAbs;

      Circle* segmentArray;

      float outerAreaRatio,innerAreaRatio,areasRatio;
      int queueStart,queueEnd,queueOldStart,numSegments;
      int width,height,len,siz;
      int expand[4];
      int *buffer;
      int *queue;
      unsigned char *ptr;
      //CTimer timer;
      int tima,timb,timc,timd,sizer,sizerAll;
      
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
      };
  };
}

#endif

/* end of CCircleDetect.h */
