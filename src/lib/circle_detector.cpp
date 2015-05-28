#include <cstdio>
#include <whycon/circle_detector.h>
using namespace std;

#define min(a,b) ((a) < (b) ? (a) : (b))
#define max(a,b) ((a) > (b) ? (a) : (b))

#define ENABLE_RANDOMIZED_THRESHOLD
#define MAX_SEGMENTS 10000 // TODO: necessary?
#define CIRCULARITY_TOLERANCE 0.01

cv::CircleDetector::CircleDetector(int _width, int _height, Context* _context, float _diameter_ratio) :
	context(_context)
{
	minSize = 10;
  maxSize = 100*100; // TODO: test!
	centerDistanceToleranceRatio = 0.1;
	centerDistanceToleranceAbs = 5;
	circularTolerance = 0.3;
	ratioTolerance = 1.0;
	
	//initialization - fixed params
	width = _width;
	height = _height;
	len = width*height;
	siz = len*3;
  diameterRatio = _diameter_ratio;
	float areaRatioInner_Outer = diameterRatio*diameterRatio;
	outerAreaRatio = M_PI*(1.0-areaRatioInner_Outer)/4;
	innerAreaRatio = M_PI/4.0;
	areasRatio = (1.0-areaRatioInner_Outer)/areaRatioInner_Outer;

  threshold = (3 * 256) / 2;
  threshold_counter = 0;

  use_local_window = false;
  local_window_multiplier = 2.5;
}

cv::CircleDetector::~CircleDetector()
{
}

int cv::CircleDetector::get_threshold(void) const
{
  return threshold;
}

void cv::CircleDetector::change_threshold(void)
{
  //int old_threshold = threshold;
  #if !defined(ENABLE_RANDOMIZED_THRESHOLD)
  threshold_counter++;
	int d = threshold_counter;
  int div = 1;
	while (d > 1){
		d /= 2;
		div *= 2;
	}
	int step = 256 / div;
	threshold = 3 * (step * (threshold_counter - div) + step/2);
  if (step <= 16) threshold_counter = 0;
  #else
  threshold = (rand() % 48) * 16;
  #endif
  WHYCON_DEBUG("threshold changed to " << threshold);
}

/* This thresholds the given pixel (RGB) returning BLACK or WHITE codes */
inline int cv::CircleDetector::threshold_pixel(uchar* ptr)
{
  return ((ptr[0]+ptr[1]+ptr[2]) > threshold) + BLACK;
}

bool cv::CircleDetector::examineCircle(const cv::Mat& image, cv::CircleDetector::Circle& circle, int ii, float areaRatio, bool search_in_window)
{
  //int64_t ticks = cv::getTickCount();  
  // get shorter names for elements in Context
	vector<int>& buffer = context->buffer;
  vector<int>& queue = context->queue;

  int vx,vy;
	queueOldStart = queueStart;
	int position = 0;
	int pos;	
	bool result = false;
	int type = buffer[ii];
	int maxx,maxy,minx,miny;
  int pixel_class;

  WHYCON_DEBUG("examine (type " << type << ") at " << ii / width << "," << ii % width << " (numseg " << context->total_segments << ")");

	int segment_id = context->total_segments++;
	buffer[ii] = segment_id;
	circle.x = ii % width;
	circle.y = ii / width;
	minx = maxx = circle.x;
	miny = maxy = circle.y;
	circle.valid = false;
	circle.round = false;
	//push segment coords to the queue
	queue[queueEnd++] = ii;
	//and until queue is empty
	while (queueEnd > queueStart){
		//pull the coord from the queue
		position = queue[queueStart++];
		//search neighbours

    int position_x = position % width;
    int position_y = position / width;

    if ((search_in_window && position_x + 1 < min(local_window_x + local_window_width, width)) ||
        (!search_in_window && position_x + 1 < width))
    {
      pos = position + 1;
      pixel_class = buffer[pos];
      if (is_unclassified(pixel_class)) {
        uchar* ptr = &image.data[pos*3];
        pixel_class = threshold_pixel(ptr);
        if (pixel_class != type) buffer[pos] = pixel_class;
      }
      if (pixel_class == type) {
        queue[queueEnd++] = pos;
        maxx = max(maxx,pos%width);
        buffer[pos] = segment_id;
      }
    }
    
    if ((search_in_window && position_x - 1 >= local_window_x) ||
        (!search_in_window && position_x - 1 >= 0))
    {
      pos = position-1;
      pixel_class = buffer[pos];
      if (is_unclassified(pixel_class)) {
        uchar* ptr = &image.data[pos*3];
        pixel_class = threshold_pixel(ptr);
        if (pixel_class != type) buffer[pos] = pixel_class;
      }
      if (pixel_class == type) {
        queue[queueEnd++] = pos;
        minx = min(minx,pos%width);
        buffer[pos] = segment_id;
      }
    }

    if ((search_in_window && position_y - 1 >= local_window_y) ||
        (!search_in_window && position_y - 1 >= 0))
    {
      pos = position-width;
      pixel_class = buffer[pos];
      if (is_unclassified(pixel_class)) {
        uchar* ptr = &image.data[pos*3];
        pixel_class = threshold_pixel(ptr);
        if (pixel_class != type) buffer[pos] = pixel_class;
      }
      if (pixel_class == type) {
        queue[queueEnd++] = pos;
        miny = min(miny,pos/width);
        buffer[pos] = segment_id;
      }
    }

		if ((search_in_window && position_y + 1 < min(local_window_y + local_window_height, height)) ||
				(!search_in_window && position_y + 1 < height))
		{
			pos = position+width;
			pixel_class = buffer[pos];
			if (is_unclassified(pixel_class)) {
				uchar* ptr = &image.data[pos*3];
				pixel_class = threshold_pixel(ptr);
				if (pixel_class != type) buffer[pos] = pixel_class;
			}
			if (pixel_class == type) {
				queue[queueEnd++] = pos;
				maxy = max(maxy,pos/width);
				buffer[pos] = segment_id;
			}
		}

    //if (queueEnd-queueOldStart > maxSize) return false;
  }

	//once the queue is empty, i.e. segment is complete, we compute its size 
	circle.size = queueEnd-queueOldStart;
	if (circle.size > minSize){
		//and if its large enough, we compute its other properties 
		circle.maxx = maxx;
		circle.maxy = maxy;
		circle.minx = minx;
		circle.miny = miny;
		circle.type = -type;
		vx = (circle.maxx-circle.minx+1);
		vy = (circle.maxy-circle.miny+1);
		circle.x = (circle.maxx+circle.minx)/2;
		circle.y = (circle.maxy+circle.miny)/2;
		circle.roundness = vx*vy*areaRatio/circle.size;
		//we check if the segment is likely to be a ring 
		if (fabsf(circle.roundness - 1) < circularTolerance)
		{
			//if its round, we compute yet another properties 
			circle.round = true;

      // TODO: mean computation could be delayed until the inner ring also satisfies above condition, right?
			circle.mean = 0;
			for (int p = queueOldStart;p<queueEnd;p++){
				pos = queue[p];
				circle.mean += image.data[pos*3]+image.data[pos*3+1]+image.data[pos*3+2];
			}
			circle.mean = circle.mean/circle.size;
			result = true;
			WHYCON_DEBUG("valid segment of " << circle.size << " pixels, with size " << vx << " x " << vy << " with mean " << circle.mean);
		} else WHYCON_DEBUG("not round enough (" << circle.roundness << ") vx/vy " << vx << " x " << vy << " ctr " << circle.x << " " << circle.y << " " << circle.size << " " << areaRatio);
	}
	else WHYCON_DEBUG("not large enough (" << circle.size << "/" << minSize << ")");

  //double delta = (double)(cv::getTickCount() - ticks) / cv::getTickFrequency();
  //cout << "examineCircle: " << delta << " " << " fps: " << 1/delta << " pix: " << circle.size << " " << threshold << endl;

	return result;
}

/* Returns if the corresponding pixel needs to be classified by the current detector or not */
inline bool cv::CircleDetector::is_unclassified(int pixel_class)
{
	if (pixel_class < 0) {
		return (pixel_class != BLACK && pixel_class != WHITE);
	}
	else {
		/* this pixel belongs to a segment (connected component which was already examined) with a corresponding segment id, classify it
		 * if the segment was found by another detector but only if it does not belong to a valid circle */
		return (pixel_class < initial_segment_id && context->valid_segment_ids.find(pixel_class) == context->valid_segment_ids.end());
	}

}

cv::CircleDetector::Circle cv::CircleDetector::detect(const cv::Mat& image, bool& fast_cleanup_possible, const cv::CircleDetector::Circle& previous_circle)
{
  /* this allows to differentiate segments found by this detector from others, and know how many segments where found in this call */
  initial_segment_id = context->total_segments;

  /* obtain unique id, used to generate BLACK/WHITE/UNKNOWN identifiers for this detector */
  detector_id = context->next_detector_id++;
  BLACK = -3 * detector_id   - 3;
  WHITE = -3 * detector_id   - 2;
  UNKNOWN = -3 * detector_id - 1;

  WHYCON_DEBUG("detector id " << detector_id << " B/W/U " << BLACK << "/" << WHITE << "/" << UNKNOWN);
  WHYCON_DEBUG("threshold " << threshold);
  WHYCON_DEBUG("initial segment id " << initial_segment_id);

  vector<int>& buffer = context->buffer;
  vector<int>& queue = context->queue;

	int pos = (height-1)*width;
  int ii = 0;
	int start = 0;
  Circle inner, outer;

	bool search_in_window = false;
	int local_x, local_y;
	if (previous_circle.valid){
		WHYCON_DEBUG("starting with previously valid circle at " << previous_circle.x << "," << previous_circle.y);
		ii = ((int)previous_circle.y)*width+(int)previous_circle.x;
		start = ii;

		if (use_local_window) {
			local_window_width = local_window_multiplier * (previous_circle.maxx - previous_circle.minx);
			local_window_height = local_window_multiplier * (previous_circle.maxy - previous_circle.miny);
			/* top-left corner of window */
			local_window_x = MAX((int)(previous_circle.x - local_window_width * 0.5), 0);
			local_window_y = MAX((int)(previous_circle.y - local_window_height * 0.5), 0);
			/* initial x,y position of search */
			local_x = (int)previous_circle.x;
			local_y = (int)previous_circle.y;
			search_in_window = true;
			WHYCON_DEBUG("window " << local_window_width << " x " << local_window_height << " : " << local_window_x << " , " << local_window_y);
		}
	}

  //cout << "detecting (thres " << threshold << ") at " << ii << endl;

	do
	{
		if ((context->total_segments - initial_segment_id) > MAX_SEGMENTS) { WHYCON_DEBUG("reached maximum number of segments"); break; }
    
    /* if current position needs to be thresholded */
    int pixel_class = buffer[ii];
    if (is_unclassified(pixel_class)){
      //cout << "unclassified pixel at ii" << endl;
			uchar* ptr = &image.data[ii*3];
      pixel_class = threshold_pixel(ptr);
      if (pixel_class == BLACK) buffer[ii] = pixel_class; // only tag black pixels, to avoid dirtying the buffer outside the ellipse
      // NOTE: the inner white area will not initially be tagged, but once the inner circle is processed, it will
		}
		//cout << "pixel " << ii << " class " << pixel_class << endl;

		//cout << pixel_class << " " << ii << endl;

    // if the current pixel is detected as "black"
    if (pixel_class == BLACK) {
			queueEnd = 0;
			queueStart = 0;
			//cout << "black pixel at " << ii << endl;
      
			// check if looks like the outer portion of the ring
			if (examineCircle(image, outer, ii, outerAreaRatio, search_in_window)){
				pos = outer.y * width + outer.x; // jump to the middle of the ring

				WHYCON_DEBUG("found valid outer, looking for white at " << pos << " id: " << context->total_segments - 1);

        // treshold the middle of the ring and check if it is detected as "white"
        pixel_class = buffer[pos];
        if (is_unclassified(pixel_class)){
					uchar* ptr = &image.data[pos*3];
					pixel_class = threshold_pixel(ptr);
          buffer[pos] = pixel_class;
				}
				if (pixel_class == WHITE) {

          // check if it looks like the inner portion
          if (examineCircle(image, inner, pos, innerAreaRatio, search_in_window)){
            // it does, now actually check specific properties to see if it is a valid target
						if (
								((float)outer.size/areasRatio/(float)inner.size - ratioTolerance < 1.0 &&
                 (float)outer.size/areasRatio/(float)inner.size + ratioTolerance > 1.0) && 
								 (fabsf(inner.x - outer.x) <= centerDistanceToleranceAbs + centerDistanceToleranceRatio * ((float)(outer.maxx - outer.minx))) &&
								 (fabsf(inner.y - outer.y) <= centerDistanceToleranceAbs + centerDistanceToleranceRatio * ((float)(outer.maxy - outer.miny)))
						   )
            {
							float cm0,cm1,cm2;
							cm0 = cm1 = cm2 = 0;
							inner.x = outer.x;
							inner.y = outer.y;

              // computer centroid
							float sx = 0;
              float sy = 0;
							queueOldStart = 0;
							for (int p = 0;p<queueEnd;p++){
								pos = queue[p];
								sx += pos % width;
                sy += pos / width;
							}
              // update pixel-based position oreviously computed
							inner.x = sx / queueEnd;
							inner.y = sy / queueEnd;
							outer.x = sx / queueEnd;
							outer.y = sy / queueEnd;

              // compute covariance
							for (int p = 0;p<queueEnd;p++){
								pos = queue[p];
								float tx = pos % width - outer.x;
								float ty = pos / width - outer.y;
								cm0 += tx * tx; 
								cm2 += ty * ty; 
								cm1 += tx * ty;
							}

							float fm0,fm1,fm2;
							fm0 = ((float)cm0)/queueEnd; // cov(x,x)
							fm1 = ((float)cm1)/queueEnd; // cov(x,y)
							fm2 = ((float)cm2)/queueEnd; // cov(y,y)

              float trace = fm0 + fm2; // sum of elements in diag.
              float det = trace * trace - 4*(fm0 * fm2 - fm1 * fm1);
              if (det > 0) det = sqrt(det); else det = 0;                    //yes, that can happen as well:(
              float f0 = (trace + det)/2;
              float f1 = (trace - det)/2;
              inner.m0 = sqrt(f0);
              inner.m1 = sqrt(f1);
              if (fm1 != 0) {                               //aligned ?
                inner.v0 = -fm1/sqrt(fm1*fm1+(fm0-f0)*(fm0-f0)); // no-> standard calculatiion
                inner.v1 = (fm0-f0)/sqrt(fm1*fm1+(fm0-f0)*(fm0-f0));
              }
              else{
                inner.v0 = inner.v1 = 0;
                if (fm0 > fm2) inner.v0 = 1.0; else inner.v1 = 1.0;   // aligned, hm, is is aligned with x or y ?
              }
              
							inner.bwRatio = (float)outer.size/inner.size;
	
              // TODO: purpose? should be removed? if next if fails, it will go over and over to the same place until number of segments
              // reaches max, right?
              //ii = start - 1; // track position
              
							float circularity = M_PI*4*(inner.m0)*(inner.m1)/queueEnd;
							if (fabsf(circularity - 1) < CIRCULARITY_TOLERANCE){
								outer.valid = inner.valid = true; // at this point, the target is considered valid
                /*inner_id = numSegments; outer_id = numSegments - 1;*/
                threshold = (outer.mean + inner.mean) / 2; // use a new threshold estimate based on current detection
                //cout << "threshold set to average: " << threshold << endl;
								
                //pixel leakage correction
								float r = diameterRatio*diameterRatio;
								float m0o = sqrt(f0);
								float m1o = sqrt(f1);
								float ratio = (float)inner.size/(outer.size + inner.size);
								float m0i = sqrt(ratio)*m0o;
								float m1i = sqrt(ratio)*m1o;
								float a = (1-r);
								float b = -(m0i+m1i)-(m0o+m1o)*r;
								float c = (m0i*m1i)-(m0o*m1o)*r;
							 	float t = (-b-sqrt(b*b-4*a*c))/(2*a);
								m0i-=t;m1i-=t;m0o+=t;m1o+=t;
								
								inner.m0 = sqrt(f0)+t;
								inner.m1 = sqrt(f1)+t;
                inner.minx = outer.minx;
                inner.maxx = outer.maxx;
                inner.maxy = outer.maxy;
                inner.miny = outer.miny;

                WHYCON_DEBUG("found inner segment " << context->total_segments - 1);
                break;
							}
            }
					}
				}
			}
		}

		if (search_in_window) {
			local_x++;
			if (local_x >= local_window_x + local_window_width || local_x >= width) { local_x = local_window_x; local_y++; }
			if (local_y >= local_window_y + local_window_height || local_y >= height) { local_y = local_window_y; local_x = local_window_x; }
			ii = local_y * width + local_x;
		}
		else {
			ii++;
			if (ii >= len) ii = 0;
		}
	} while (ii != start);

	// draw
	if (inner.valid)
		threshold_counter = 0;
  else
    change_threshold(); // update threshold for next run. inner is what user receives

  /*cv::namedWindow("buffer", CV_WINDOW_NORMAL);
  cv::Mat buffer_img;
  context->debug_buffer(image, buffer_img);
  cv::imshow("buffer", buffer_img);*/
  //cv::waitKey();

  /* if the current call found a valid match, and only two segments were found during the search (inner/outer)
    then, only the bounding box area of the outer ellipse needs to be cleaned in 'buffer' */
  fast_cleanup_possible = (inner.valid && (context->total_segments - initial_segment_id) == 2);

  WHYCON_DEBUG("processed segments " << (context->total_segments - initial_segment_id));
  
  if (!inner.valid) WHYCON_DEBUG("detection failed");
  else WHYCON_DEBUG("detected at " << inner.x << " " << inner.y);
	return inner;
}


void cv::CircleDetector::cover_last_detected(cv::Mat& image)
{
  const vector<int>& queue = context->queue;
  for (int i = queueOldStart; i < queueEnd; i++) {
    int pos = queue[i];
    uchar* ptr = image.data + 3*pos;
    *ptr = 255; ptr++;
    *ptr = 255; ptr++;
    *ptr = 255;
  }
}

cv::CircleDetector::Circle::Circle(void)
{
  x = y = 0;
  round = valid = false;
}

void cv::CircleDetector::Circle::draw(cv::Mat& image, const std::string& text, cv::Vec3b color, float thickness) const
{
  for (float e = 0; e < 2 * M_PI; e += 0.05) {
    float fx = x + cos(e) * v0 * m0 * 2 + v1 * m1 * 2 * sin(e);
    float fy = y + cos(e) * v1 * m0 * 2 - v0 * m1 * 2 * sin(e);
    int fxi = (int)(fx + 0.5);
    int fyi = (int)(fy + 0.5);
    if (fxi >= 0 && fxi < image.cols && fyi >= 0 && fyi < image.rows)
      image.at<cv::Vec3b>(fyi, fxi) = color;
  }
  
  float scale = image.size().width / 1800.0f;
  //float thickness = scale * 3.0;
  //if (thickness < 1) thickness = 1;
  cv::putText(image, text.c_str(), cv::Point(x + 2 * m0, y + 2 * m1), CV_FONT_HERSHEY_SIMPLEX, scale, cv::Scalar(color), thickness, CV_AA);
  cv::line(image, cv::Point(x + v0 * m0 * 2, y + v1 * m0 * 2), cv::Point(x - v0 * m0 * 2, y - v1 * m0 * 2), cv::Scalar(color), 1, 8);
  cv::line(image, cv::Point(x + v1 * m1 * 2, y - v0 * m1 * 2), cv::Point(x - v1 * m1 * 2, y + v0 * m1 * 2), cv::Scalar(color), 1, 8); 
}

void cv::CircleDetector::Circle::write(cv::FileStorage& fs) const {
  fs << "{" << "x" << x << "y" << y << "size" << size <<
    "maxy" << maxy << "maxx" << maxx << "miny" << miny << "minx" << minx <<
    "mean" << mean << "type" << type << "roundness" << roundness << "bwRatio" << bwRatio <<
    "round" << round << "valid" << valid << "m0" << m0 << "m1" << m1 << "v0" << v0 << "v1" << v1 << "}";
}

void cv::CircleDetector::Circle::read(const cv::FileNode& node)
{
  x = (float)node["x"];
  y = (float)node["y"];
  size = (int)node["size"];
  maxy = (int)node["maxy"];
  maxx = (int)node["maxx"];
  miny = (int)node["miny"];
  minx = (int)node["minx"];
  mean = (int)node["mean"];
  type = (int)node["type"];
  roundness = (float)node["roundness"];
  bwRatio = (float)node["bwRatio"];
  round = (int)node["round"];
  valid = (int)node["valid"];
  m0 = (float)node["m0"];
  m1 = (float)node["m1"];
  v0 = (float)node["v0"];
  v1 = (float)node["v1"];
}

cv::CircleDetector::Context::Context(int _width, int _height)
{
  width = _width;
  height = _height;
  int len = width * height;
  buffer.resize(len);
  queue.resize(len);

  cleanup_buffer();
  reset();
}

void cv::CircleDetector::Context::reset(void)
{
	next_detector_id = 0;
	valid_segment_ids.clear();
	total_segments = 0;
}

void cv::CircleDetector::Context::cleanup_buffer(void)
{
	WHYCON_DEBUG("clean whole buffer");
	memset(&buffer[0], -1, sizeof(int)*buffer.size());
}

void cv::CircleDetector::Context::cleanup_buffer(const Circle& c) {
  if (c.valid) // TODO: necessary?
  {
    // zero only parts modified when detecting 'c'
		int ix = max(c.minx - 2,1);
		int ax = min(c.maxx + 2, width - 2);
		int iy = max(c.miny - 2,1);
		int ay = min(c.maxy + 2, height - 2);
		for (int y = iy; y < ay; y++){
			int pos = y * width; 
			for (int x = ix; x < ax; x++) buffer[pos + x] = 0; // TODO: user ptr and/or memset
		}    
  }
}

void cv::CircleDetector::Context::debug_buffer(const cv::Mat& image, cv::Mat& out)
{
  std::map<int, cv::Vec3b> colors;
  for (int i = 0; i < total_segments; i++) colors[i] = cv::Vec3b(rand() / (float)RAND_MAX * 255.0, rand() / (float)RAND_MAX * 255.0, rand() / (float)RAND_MAX * 255.0);

  out.create(height, width, CV_8UC3);
  cv::Vec3b* out_ptr = out.ptr<cv::Vec3b>(0);
  const cv::Vec3b* im_ptr = image.ptr<cv::Vec3b>(0);
  out = cv::Scalar(128,128,128);
  for (uint i = 0; i < out.total(); i++, ++out_ptr, ++im_ptr) {
    /*if (buffer[i] == -1) *ptr = cv::Vec3b(0,0,0);
    else if (buffer[i] == -2) *ptr = cv::Vec3b(255,255,255);*/
    //else if (buffer[i] < 0) *ptr = cv::Vec3b(0, 255, 0);
    if (buffer[i] >= 0) *out_ptr = colors[buffer[i]];
    else {
      int pixel_class = (-(buffer[i] + 1) % 3);
      if (pixel_class == 0) *out_ptr = cv::Vec3b(0, 255, 0); // UNKNOWN
      else if (pixel_class == 1) *out_ptr = cv::Vec3b(255, 0, 0); // WHITE
      else *out_ptr = cv::Vec3b(0, 0, 255); // BLACK
    }
  }
}
