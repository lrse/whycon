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
      class Context;
      
      CircleDetector(int width, int height, Context* context, float diameter_ratio = (5/14.0));
      ~CircleDetector();
      
      Circle detect(const cv::Mat& image, const Circle& previous_circle = cv::CircleDetector::Circle());
      bool examineCircle(const cv::Mat& image, Circle& circle, int ii, float areaRatio);
      
      void improveEllipse(const cv::Mat& image, Circle& c);

      bool draw;

    private:
    
      int minSize; 
      float diameterRatio;
      int thresholdStep;
      float circularTolerance;
      float ratioTolerance;
      float centerDistanceToleranceRatio;
      int centerDistanceToleranceAbs;

      float outerAreaRatio,innerAreaRatio,areasRatio;
      int width,height,len,siz;

      int threshold, threshold_counter;
      void change_threshold(void);

      int queueStart,queueEnd,queueOldStart,numSegments;

      Context* context;
      
    public:
      class Circle {
        public:
          Circle(void);
          
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

      class Context {
        public:
          Context(int _width, int _height);
          void debug_buffer(cv::Mat& img);

          std::vector<int> buffer, queue;
          int width, height;

        private:
          void cleanup(const Circle& c, bool fast_cleanup);
          friend class CircleDetector;
      };
  };
}

#endif

/* end of CCircleDetect.h */
