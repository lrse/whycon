#ifndef __CIRCLE_LOCALIZER_H__
#define __CIRCLE_LOCALIZER_H__

#include <vector>
#include <tbb/parallel_for.h>
#include <tbb/blocked_range.h>
#include "circle_detector.h"

namespace cv {
  class ManyCircleDetector {
    public:
      ManyCircleDetector(int number_of_circles, int width, int height);
      ~ManyCircleDetector(void);
      
      bool initialize(const cv::Mat& image);
      bool detect(const cv::Mat& image);
      
      std::vector<CircleDetector::Circle> circles;

      // for parallel computation
      class Functor {
        public:
          Functor(ManyCircleDetector& localizer, const cv::Mat& image);
          void operator()(tbb::blocked_range<int>& r) const;
          
          const cv::Mat& image;
          ManyCircleDetector& detector;
          std::vector<CircleDetector::Circle>& circles;
          std::vector<CircleDetector>& detectors;          
      };
      
    private:
      int width, height, number_of_circles;
      std::vector<CircleDetector> detectors;
      CircleDetector::Context context;
  };
}

#endif
