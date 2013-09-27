#include <cstdio>
#include "circle_detector.h"
using namespace std;

#define min(a,b) ((a) < (b) ? (a) : (b))
#define max(a,b) ((a) > (b) ? (a) : (b))

#define MAX_SEGMENTS 10000 // TODO: necessary?
#define CIRCULARITY_TOLERANCE 0.02

cv::CircleDetector::CircleDetector(int _width,int _height, Context* _context, float _diameter_ratio) : context(_context)
{
	minSize = 10;
  maxSize = 100*100; // TODO: test!
	centerDistanceToleranceRatio = 0.1;
	centerDistanceToleranceAbs = 5;
	circularTolerance = 0.3;
	ratioTolerance = 1.0;
  draw = false;
	
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
  // << "outerRatio/innerRatio " << outerAreaRatio << " " << innerAreaRatio << " " << diameterRatio << endl;

  threshold = (3 * 256) / 2;
  threshold_counter = 0;
}

cv::CircleDetector::~CircleDetector()
{
}

void cv::CircleDetector::change_threshold(void)
{
  threshold_counter++;
	int d = threshold_counter;
  int div = 1;
	while (d > 1){
		d /= 2;
		div *= 2;
	}
	int step = 256 / div;
	threshold = 3 * (step * (threshold_counter - div) + step/2);
  cout << "changed threshold to " << threshold << endl;
  if (step <= 16) threshold_counter = 0;
}

#if 0
bool cv::CircleDetector::examineCircle(const cv::Mat& image, cv::CircleDetector::Circle& circle, int ii, float areaRatio)
{
  int64_t ticks = cv::getTickCount();  
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
  uint sum;

  //cout << "examine (type " << type << ") at " << ii / width << "," << ii % width << " (numseg " << numSegments << ")" << endl;
  
	buffer[ii] = ++numSegments;
	circle.x = ii%width; 
	circle.y = ii/width;
	minx = maxx = circle.x;
	miny = maxy = circle.y;
	circle.valid = false;
	circle.round = false;
  circle.size = 0;
  circle.mean = 0;
	//push segment coords to the queue
	queue[queueEnd++] = ii;
	//and until queue is empty
	while (queueEnd > queueStart){
		//pull the coord from the queue
		position = queue[queueStart++];
		//search neighbours

    pos = position;
    int direction = 1; // go east first
    do {
      pos += direction; // go in the current direction (east/west)
      pixel_class = buffer[pos];
      // check if current pixel is thresholded. if not, do it
      if (pixel_class == 0) {
        uchar* ptr = &image.data[pos*3];
        sum = (ptr[0]+ptr[1]+ptr[2]);
        pixel_class = (sum > threshold)-2;
        if (pixel_class != type) buffer[pos] = pixel_class;
      }

      // if current pixel is of desired type, label it as belonging to current segment.
      if (pixel_class == type) {
        //queue[queueEnd++] = pos;
        circle.size++;
        if (direction == 1)
          maxx = max(maxx,pos%width);
        else
          minx = min(minx,pos%width);
        buffer[pos] = numSegments;
        circle.mean += sum;
      }
      else {
        // else, go to west now if was going east. otherwise stop
        if (direction == 1) { pos = position; direction = -1; continue; }
        else break;
      }

      // check north of this pixel
      {
        int north_pos = pos-width;
        pixel_class = buffer[north_pos];
        if (pixel_class == 0) {
          uchar* ptr = &image.data[north_pos*3];
          sum = (ptr[0]+ptr[1]+ptr[2]);
          pixel_class = (sum > threshold)-2;
          if (pixel_class != type) buffer[north_pos] = pixel_class;
        }
        // north pixel is of desired type, add to queue and label it. otherwise remember the type
        if (pixel_class == type) {
          circle.size++;
          queue[queueEnd++] = north_pos;
          miny = min(miny,north_pos/width);
          buffer[north_pos] = numSegments;
        }
      }

      // same thing for south pixel
      {
        int south_pos = pos+width;
        pixel_class = buffer[south_pos];
        if (pixel_class == 0) {
          uchar* ptr = &image.data[south_pos*3];
          pixel_class = ((ptr[0]+ptr[1]+ptr[2]) > threshold)-2;
          if (pixel_class != type) buffer[south_pos] = pixel_class;
        }
        if (pixel_class == type) {
          circle.size++;
          queue[queueEnd++] = south_pos;
          maxy = max(maxy,south_pos/width);
          buffer[south_pos] = numSegments;
        }
      }
    } while (true);
  }

	//once the queue is empty, i.e. segment is complete, we compute its size 
	//circle.size = queueEnd-queueOldStart;
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
		if (circle.roundness - circularTolerance < 1.0 && circle.roundness + circularTolerance > 1.0)
		{
			//if its round, we compute yet another properties 
			circle.round = true;
			/*circle.mean = 0;
			for (int p = queueOldStart;p<queueEnd;p++){
				pos = queue[p];
				circle.mean += image.data[pos*3]+image.data[pos*3+1]+image.data[pos*3+2];
			}*/
			circle.mean = circle.mean/circle.size;
			result = true;
      //cout << "segment size " << circle.size << " " << vx << " " << vy << endl;
		}
	}

  double delta = (double)(cv::getTickCount() - ticks) / cv::getTickFrequency();
    //cout << "examineCircle2: " << delta << " " << " fps: " << 1/delta << " pix: " << circle.size << " " << threshold << endl;

	return result;
}
#else
bool cv::CircleDetector::examineCircle(const cv::Mat& image, cv::CircleDetector::Circle& circle, int ii, float areaRatio)
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

  //cout << "examine (type " << type << ") at " << ii / width << "," << ii % width << " (numseg " << numSegments << ")" << endl;
  
	buffer[ii] = ++numSegments;
	circle.x = ii%width; 
	circle.y = ii/width;
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

    pos = position+1;
    pixel_class = buffer[pos];
    if (pixel_class == 0) {
      uchar* ptr = &image.data[pos*3];
      pixel_class = ((ptr[0]+ptr[1]+ptr[2]) > threshold)-2;
      if (pixel_class != type) buffer[pos] = pixel_class;
    }
    if (pixel_class == type) {
      queue[queueEnd++] = pos;
      maxx = max(maxx,pos%width);
      buffer[pos] = numSegments;
    }
    
		pos = position-1;
		pixel_class = buffer[pos];
    if (pixel_class == 0) {
      uchar* ptr = &image.data[pos*3];
      pixel_class = ((ptr[0]+ptr[1]+ptr[2]) > threshold)-2;
      if (pixel_class != type) buffer[pos] = pixel_class;
    }
    if (pixel_class == type) {
      queue[queueEnd++] = pos;
      minx = min(minx,pos%width);
      buffer[pos] = numSegments;
    }
    
    pos = position-width;
		pixel_class = buffer[pos];
    if (pixel_class == 0) {
      uchar* ptr = &image.data[pos*3];
      pixel_class = ((ptr[0]+ptr[1]+ptr[2]) > threshold)-2;
      if (pixel_class != type) buffer[pos] = pixel_class;
    }
    if (pixel_class == type) {
      queue[queueEnd++] = pos;
      miny = min(miny,pos/width);
      buffer[pos] = numSegments;
    }

		pos = position+width;
		pixel_class = buffer[pos];
    if (pixel_class == 0) {
      uchar* ptr = &image.data[pos*3];
      pixel_class = ((ptr[0]+ptr[1]+ptr[2]) > threshold)-2;
      if (pixel_class != type) buffer[pos] = pixel_class;
    }
    if (pixel_class == type) {
      queue[queueEnd++] = pos;
      maxy = max(maxy,pos/width);
      buffer[pos] = numSegments;
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
		if (circle.roundness - circularTolerance < 1.0 && circle.roundness + circularTolerance > 1.0)
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
      // << "segment size " << circle.size << " " << vx << " " << vy << endl;
		}// else cout << "not round enough (" << circle.roundness << ") vx/vy " << vx << " " << vy << " ctr " << circle.x << " " << circle.y << " " << circle.size << " " << areaRatio << endl;
	}

  //double delta = (double)(cv::getTickCount() - ticks) / cv::getTickFrequency();
  //cout << "examineCircle: " << delta << " " << " fps: " << 1/delta << " pix: " << circle.size << " " << threshold << endl;

	return result;
}
#endif

cv::CircleDetector::Circle cv::CircleDetector::detect(const cv::Mat& image, const cv::CircleDetector::Circle& previous_circle)
{
  vector<int>& buffer = context->buffer;
  vector<int>& queue = context->queue;

	int pos = (height-1)*width;
  int ii = 0;
	int start = 0;
  Circle inner, outer;
  //int outer_id;

	if (previous_circle.valid){
		ii = ((int)previous_circle.y)*width+(int)previous_circle.x;
		start = ii;
	}

  //cout << "detecting (thres " << threshold << ") at " << ii << endl;

  numSegments = 0;
	do
	{
    if (numSegments > MAX_SEGMENTS) break;
    //if (start != 0) cout << "it " << ii << endl;
    
    // if current position needs to be thresholded
    int pixel_class = buffer[ii];
		if (pixel_class == 0){
			uchar* ptr = &image.data[ii*3];
      //cout << "value: " << (ptr[0]+ptr[1]+ptr[2]) << endl;
      pixel_class = ((ptr[0]+ptr[1]+ptr[2]) > threshold)-2;
      if (pixel_class == -2) buffer[ii] = pixel_class; // only tag black pixels, to avoid dirtying the buffer outside the ellipse
      // NOTE: the inner white area will not initially be tagged, but once the inner circle is processed, it will
		}

    //cout << pixel_class << endl;

    // if the current pixel is detected as "black"
		if (pixel_class == -2){
			queueEnd = 0;
			queueStart = 0;
      // << "black pixel " << ii << endl;
      
			// check if looks like the outer portion of the ring
			if (examineCircle(image, outer, ii, outerAreaRatio)){
				pos = outer.y * width + outer.x; // jump to the middle of the ring

        // treshold the middle of the ring and check if it is detected as "white"
        pixel_class = buffer[pos];
				if (pixel_class == 0){
					uchar* ptr = &image.data[pos*3];
					pixel_class = ((ptr[0]+ptr[1]+ptr[2]) > threshold)-2;
          buffer[pos] = pixel_class;
				}
				if (pixel_class == -1){

          // check if it looks like the inner portion
					if (examineCircle(image, inner, pos, innerAreaRatio)){
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
							ii = start - 1; // track position 
              
							float circularity = M_PI*4*(inner.m0)*(inner.m1)/queueEnd;
							if (fabsf(circularity - 1) < CIRCULARITY_TOLERANCE){
								outer.valid = inner.valid = true; // at this point, the target is considered valid
                /*inner_id = numSegments; outer_id = numSegments - 1;*/
                threshold = (outer.mean + inner.mean) / 2; // use a new threshold estimate based on current detection
								
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
                break;
							}
            }
					}
				}
			}
		}
		ii++;
		if (ii >= len) ii = 0;
	} while (ii != start);

	// draw
	if (inner.valid){
		threshold_counter = 0;
    if (draw) {
      for (int i = queueOldStart; i < queueEnd; i++) {
        int pos = queue[i];
        uchar* ptr = image.data + 3*pos;
        *ptr = 255; ptr++;
        *ptr = 255; ptr++;
        *ptr = 255;
      }
    }
	}
  else {
    change_threshold(); // update threshold for next run. inner is what user receives
	}

  /*cv::Mat buffer_img;
  context->debug_buffer(buffer_img);
  cv::imshow("buffer", buffer_img);*/

  // if this is not the first call (there was a previous valid circle where search started),
  // the current call found a valid match, and only two segments were found during the search (inner/outer)
  // then, only the bounding box area of the outer ellipse needs to be cleaned in 'buffer'
  bool fast_cleanup = (previous_circle.valid && numSegments == 2 && inner.valid); 
  context->cleanup(outer, fast_cleanup);
  
  if (!inner.valid) cout << "detection failed" << endl;
	return inner;
}

void cv::CircleDetector::improveEllipse(const cv::Mat& image, Circle& c)
{
  cv::Mat subimg;
  int delta = 10;
  cout << image.rows << " x " << image.cols << endl;
  cv::Range row_range(max(0, c.miny - delta), min(c.maxy + delta, image.rows));
  cv::Range col_range(max(0, c.minx - delta), min(c.maxx + delta, image.cols));
  cout << row_range.start << " " << row_range.end << " " << col_range.start << " " << col_range.end << endl;
  image(row_range, col_range).copyTo(subimg);
  cv::Mat cannified;
  cv::Canny(subimg, cannified, 4000, 8000, 5, true);
  
  /*cv::namedWindow("bleh");
  cv::imshow("bleh", subimg);
  cv::waitKey();*/
  
  std::vector< std::vector<cv::Point> > contours;
  cv::findContours(cannified, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
  if (contours.empty() || contours[0].size() < 5) return;
  
  cv::Mat contour_img;
  subimg.copyTo(contour_img);
  cv::drawContours(contour_img, contours, 0, cv::Scalar(255,0,255), 1);
  
  /*cv::namedWindow("bleh2");
  cv::imshow("bleh2", contour_img);
  cv::waitKey();*/
  
  
  cv::RotatedRect rect = cv::fitEllipse(contours[0]);
  cout << "old: " << c.x << " " << c.y << " " << c.m0 << " " << c.m1 << " " << c.v0 << " " << c.v1 << endl;
  c.x = rect.center.x + col_range.start;
  c.y = rect.center.y + row_range.start;
  /*float max_size = max(rect.size.width, rect.size.height);
  float min_size = min(rect.size.width, rect.size.height);*/
  c.m0 = rect.size.width * 0.25;      
  c.m1 = rect.size.height * 0.25;
  c.v0 = cos(rect.angle / 180.0 * M_PI);
  c.v1 = sin(rect.angle / 180.0 * M_PI);
  cout << "new: " << c.x << " " << c.y << " " << c.m0 << " " << c.m1 << " " << c.v0 << " " << c.v1 << endl;
  
  /*cv::Mat ellipse_img;
  image(row_range, col_range).copyTo(subimg);
  subimg.copyTo(ellipse_img);
  cv::ellipse(ellipse_img, rect, cv::Scalar(255,0,255));
  cv::namedWindow("bleh3");
  cv::imshow("bleh3", ellipse_img);
  cv::waitKey();*/
}

cv::CircleDetector::Circle::Circle(void)
{
  x = y = 0;
  round = valid = false;
}

void cv::CircleDetector::Circle::draw(cv::Mat& image, const std::string& text, cv::Scalar color, float thickness) const
{
  cv::ellipse(image, cv::Point(x, y), cv::Size((int)m0 * 2, (int)m1 * 2), atan2(v1, v0)  * 180.0 / M_PI, 0, 360, color, thickness, CV_AA);
  float scale = image.size().width / 1800.0f;
  //float thickness = scale * 3.0;
  //if (thickness < 1) thickness = 1;
  cv::putText(image, text.c_str(), cv::Point(x + 2 * m0, y + 2 * m1), CV_FONT_HERSHEY_SIMPLEX, scale, color, thickness, CV_AA);
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
  cleanup(Circle(), false);
}

void cv::CircleDetector::Context::cleanup(const Circle& c, bool fast_cleanup) {
  if (c.valid && fast_cleanup)
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
  else {
    cout << "clean whole buffer" << endl;
		memset(&buffer[0], 0, sizeof(int)*buffer.size());

    //image delimitation
		for (int i = 0;i<width;i++){
			buffer[i] = -1000;	
			buffer[(height - 1) * width + i] = -1000;
		}
		for (int i = 0;i<height;i++){
			buffer[width * i] = -1000;	
			buffer[width * i + width - 1] = -1000;
		}
  }
}

void cv::CircleDetector::Context::debug_buffer(cv::Mat& img)
{
  img.create(height, width, CV_8UC3);
  cv::Vec3b* ptr = img.ptr<cv::Vec3b>(0);
  img = cv::Scalar(128,128,128);
  for (uint i = 0; i < img.total(); i++, ++ptr) {
    if (buffer[i] == -1) *ptr = cv::Vec3b(0,0,0);
    else if (buffer[i] == -2) *ptr = cv::Vec3b(255,255,255);
    //else if (buffer[i] < 0) *ptr = cv::Vec3b(0, 255, 0);
    else if (buffer[i] > 0) *ptr = cv::Vec3b(255, 0, 255);
  }
}
