#ifndef __CIRCLE_LOCALIZER_H__
#define __CIRCLE_LOCALIZER_H__

#include <vector>
#include <boost/thread/thread.hpp>
#include <boost/thread/barrier.hpp>
#include <tbb/parallel_for.h>
#include <tbb/blocked_range.h>
#include "circle_detector.h"

namespace cv {
  class CircleLocalizer {
    public:
      CircleLocalizer(int number_of_circles, int width, int height);
      ~CircleLocalizer(void);
      
      bool initialize(const cv::Mat& image);
      void localize(const cv::Mat& image);
      void localize_individual(int id);      
      
      void set_scales(const CircleDetector::Circle& x, const CircleDetector::Circle& center, const CircleDetector::Circle& y);
      void load_calibration(const cv::Mat& k, const cv::Mat& dist_coeff);
      void compute_position(const CircleDetector::Circle& circle);
      
      cv::Mat current_image;
      std::vector<CircleDetector::Circle> circles;
      
      class Functor {
        public:
          Functor(CircleLocalizer& localizer);
          void operator()(tbb::blocked_range<int>& r) const;
          
          cv::Mat& image;
          CircleLocalizer& localizer;
          std::vector<CircleDetector::Circle>& circles;
          std::vector<CircleDetector>& detectors;          
      };
      
    private:
      bool stop;
      std::vector< boost::shared_ptr<boost::thread> > threads;
      boost::barrier start_barrier, end_barrier;
      int width, height, number_of_circles;
      std::vector<CircleDetector> detectors;
      
      
  };
}

#endif
