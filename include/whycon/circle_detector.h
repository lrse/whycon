/*
 * Date:      2010
 * Author:   Tom Krajnik, Matias Nitsche
 */

#ifndef __CIRCLE_DETECTOR_H__
#define __CIRCLE_DETECTOR_H__

#include <opencv2/opencv.hpp>
#include <math.h>
#include <vector>
#include <whycon/config.h>
#include <unordered_set>

#define WHYCON_DEFAULT_OUTER_DIAMETER 0.122
#define WHYCON_DEFAULT_INNER_DIAMETER 0.050
#define WHYCON_DEFAULT_DIAMETER_RATIO (WHYCON_DEFAULT_INNER_DIAMETER/WHYCON_DEFAULT_OUTER_DIAMETER)

namespace cv {
  class CircleDetector
  {
    public:
      class Circle;
      class Context;
      
      CircleDetector(int width, int height, Context* context, float diameter_ratio = WHYCON_DEFAULT_DIAMETER_RATIO);
      ~CircleDetector();
      
      Circle detect(const cv::Mat& image, bool& fast_cleanup_possible, const Circle& previous_circle = cv::CircleDetector::Circle());
      bool examineCircle(const cv::Mat& image, Circle& circle, int ii, float areaRatio, bool search_in_window);
      void cover_last_detected(cv::Mat& image);
      
      void improveEllipse(const cv::Mat& image, Circle& c);
      int get_threshold(void) const;

    private:
    
      int minSize, maxSize; 
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
      inline int threshold_pixel(uchar* ptr);

      int queueStart,queueEnd,queueOldStart,numSegments;

      bool use_local_window;
      float local_window_multiplier;
      int local_window_width, local_window_height, local_window_x, local_window_y;

      Context* context;
      int detector_id, BLACK, WHITE, UNKNOWN;
      int initial_segment_id;

      inline bool is_unclassified(int pixel_class);

    public:
      class Circle {
        public:
          Circle(void);
          
          float x, y;
          int size;
          int maxy,maxx,miny,minx;
          int mean;
          int type;
          float roundness, bwRatio;
          bool round, valid;
          float m0,m1; // axis dimensions
          float v0,v1; // axis (v0,v1) and (v1,-v0)

          void write(cv::FileStorage& fs) const;
          void read(const cv::FileNode& node);
          
          void draw(cv::Mat& image, const std::string& text1 = std::string(), const std::string& text2 = std::string(), cv::Vec3b color = cv::Vec3b(0,255,0), float thickness = 1) const;
      };

      class Context {
        public:
          Context(int _width, int _height);
          void debug_buffer(const cv::Mat& image, cv::Mat& img);

          void cleanup_buffer(void);
          void cleanup_buffer(const Circle& c);
          void reset(void);

          std::vector<int> buffer, queue;
          int width, height;

          int next_detector_id;
          std::unordered_set<int> valid_segment_ids;
          int total_segments;

      };
  };
}


#endif
