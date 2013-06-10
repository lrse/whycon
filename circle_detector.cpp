#include <cstdio>
#include "circle_detector.h"
using namespace std;

#define min(a,b) ((a) < (b) ? (a) : (b))
#define max(a,b) ((a) > (b) ? (a) : (b))

#define MAX_SEGMENTS 10000

//Variable initialization
cv::CircleDetector::CircleDetector(int _width,int _height, float _diameter_ratio, int _color_precision, int _color_step)
{
  color_precision = _color_precision;
  color_step = _color_step;
        
	debug = false;
	lastTrackOK = false;
	draw = false; 
	drawAll = true;
	maxFailed = 0;
	minSize = 10;
	thresholdStep = 256;
	maxThreshold = 3*256;
	centerDistanceToleranceRatio = 0.1;
	centerDistanceToleranceAbs = 5;
	circularTolerance = 0.3;
	ratioTolerance = 1.0;
	threshold = maxThreshold/2;       
	numFailed = maxFailed;
	track = true;

	//initialization - fixed params
	width = _width;
	height = _height;
	len = width*height;
	siz = len*3;
  buffer.resize(len);
  queue.resize(len);
  diameterRatio = _diameter_ratio;
	float areaRatioInner_Outer = diameterRatio*diameterRatio;
	outerAreaRatio = M_PI*(1.0-areaRatioInner_Outer)/4;
	innerAreaRatio = M_PI/4.0;
	areasRatio = (1.0-areaRatioInner_Outer)/areaRatioInner_Outer;

	tima = timb = timc =timd = sizer = sizerAll = 0;
  segmentArray.resize(MAX_SEGMENTS);
}

cv::CircleDetector::~CircleDetector()
{
}

bool cv::CircleDetector::changeThreshold()
{
	int div = 1;
	int dum = numFailed;
	while (dum > 1){
		dum = dum/2;
		div*=2;
	}
	int step = 256/div;
	threshold = 3*(step*(numFailed-div)+step/2);
	fprintf(stdout,"Threshold: %i %i %i\n",div,numFailed,threshold/3);
	return step > 16;
}

bool cv::CircleDetector::examineCircle(const cv::Mat& image, cv::CircleDetector::Circle& circle, int ii,float areaRatio)
{
	int vx,vy;
	queueOldStart = queueStart;
	int position = 0;
	int pos;	
	bool result = false;
	int type = buffer[ii];
	int maxx,maxy,minx,miny;

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
		if (buffer[pos] == 0){
			 ptr = &image.data[pos*3];
			 buffer[pos]=((ptr[0]+ptr[1]+ptr[2]) > threshold)-2;
		}
		if (buffer[pos] == type){
			queue[queueEnd++] = pos;
			maxx = max(maxx,pos%width);
			buffer[pos] = numSegments;
		}
		pos = position-1;
		if (buffer[pos] == 0){
			 ptr = &image.data[pos*3];
			 buffer[pos]=((ptr[0]+ptr[1]+ptr[2]) > threshold)-2;
		}
		if (buffer[pos] == type){
			queue[queueEnd++] = pos;
			minx = min(minx,pos%width);
			buffer[pos] = numSegments;
		}
		pos = position-width;
		if (buffer[pos] == 0){
			 ptr = &image.data[pos*3];
			 buffer[pos]=((ptr[0]+ptr[1]+ptr[2]) > threshold)-2;
		}
		if (buffer[pos] == type){
			queue[queueEnd++] = pos;
			miny = min(miny,pos/width);
			buffer[pos] = numSegments;
		}
		pos = position+width;
		if (buffer[pos] == 0){
			 ptr = &image.data[pos*3];
			 buffer[pos]=((ptr[0]+ptr[1]+ptr[2]) > threshold)-2;
		}
		if (buffer[pos] == type){
			queue[queueEnd++] = pos;
			maxy = max(maxy,pos/width);
			buffer[pos] = numSegments;
		}
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
			circle.mean = 0;
			for (int p = queueOldStart;p<queueEnd;p++){
				pos = queue[p];
				circle.mean += image.data[pos*3]+image.data[pos*3+1]+image.data[pos*3+2];
			}
			circle.mean = circle.mean/circle.size;
			result = true;	
		}
	}
	return result;
}

cv::CircleDetector::Circle cv::CircleDetector::detect(const cv::Mat& image, const cv::CircleDetector::Circle& previous_circle)
{
	Circle result;
	numSegments = 0;
  /*cv::Mat original_image;
  image.copyTo(original_image);*/

	//image delimitation
	int pos = (height-1)*width;
  int ii = 0;
	int start = 0;
	bool cont = true;

  if (!previous_circle.valid || !track || !lastTrackOK || true){
		memset(&buffer[0],0,sizeof(int)*len);
		//image delimitation
		for (int i = 0;i<width;i++){
			buffer[i] = -1000;	
			buffer[pos+i] = -1000;
		}
		for (int i = 0;i<height;i++){
			buffer[width*i] = -1000;	
			buffer[width*i+width-1] = -1000;
		}
	}

	if (previous_circle.valid && track){
		ii = ((int)previous_circle.y)*width+(int)previous_circle.x;
		start = ii;
	}
  
	while (cont) 
	{
		if (buffer[ii] == 0){
			ptr = &image.data[ii*3];
			buffer[ii]=((ptr[0]+ptr[1]+ptr[2]) > threshold)-2;
		}
		if (numSegments < MAX_SEGMENTS && buffer[ii] == -2){
			//new segment found
			queueEnd = 0;
			queueStart = 0;
			//if the segment looks like a ring, we check its inside area
			if (examineCircle(image,segmentArray[numSegments],ii,outerAreaRatio)){
				pos = segmentArray[numSegments-1].y*width+segmentArray[numSegments-1].x;
				if (buffer[pos] == 0){
					ptr = &image.data[pos*3];
					buffer[pos]=((ptr[0]+ptr[1]+ptr[2]) > threshold)-2;
				}
				if (numSegments < MAX_SEGMENTS && buffer[pos] == -1){
					if (examineCircle(image,segmentArray[numSegments],pos,innerAreaRatio)){
						//the inside area is a circle. now what is the area ratio of the black and white ? also, are the circles concentric ?

						if (debug) printf("Area ratio %i/%i is %.3f %.3f %.3f\n",numSegments-2,numSegments-1,(float)segmentArray[numSegments-2].size/segmentArray[numSegments-1].size,areasRatio,segmentArray[numSegments-2].size/areasRatio/segmentArray[numSegments-1].size);
						if (
								((float)segmentArray[numSegments-2].size/areasRatio/(float)segmentArray[numSegments-1].size - ratioTolerance < 1.0 && (float)segmentArray[numSegments-2].size/areasRatio/(float)segmentArray[numSegments-1].size + ratioTolerance > 1.0) && 
								(fabsf(segmentArray[numSegments-1].x-segmentArray[numSegments-2].x) <= centerDistanceToleranceAbs+centerDistanceToleranceRatio*((float)(segmentArray[numSegments-2].maxx-segmentArray[numSegments-2].minx))) &&
								(fabsf(segmentArray[numSegments-1].y-segmentArray[numSegments-2].y) <= centerDistanceToleranceAbs+centerDistanceToleranceRatio*((float)(segmentArray[numSegments-2].maxy-segmentArray[numSegments-2].miny)))

						   ){
							float tx,ty,cm0,cm1,cm2;
							cm0=cm1=cm2=0;
							segmentArray[numSegments-1].x = segmentArray[numSegments-2].x;
							segmentArray[numSegments-1].y = segmentArray[numSegments-2].y;

							float sx = 0;
              float sy = 0;
							queueOldStart = 0;
							for (int p = 0;p<queueEnd;p++){
								pos = queue[p];
								sx += pos%width;
                sy += pos/width;
							}
							segmentArray[numSegments-1].x = sx/queueEnd;
							segmentArray[numSegments-1].y = sy/queueEnd;
							segmentArray[numSegments-2].x = sx/queueEnd;
							segmentArray[numSegments-2].y = sy/queueEnd;

							for (int p = 0;p<queueEnd;p++){
								pos = queue[p];
								tx = pos%width-segmentArray[numSegments-2].x;
								ty = pos/width-segmentArray[numSegments-2].y;
								cm0+=tx*tx; 
								cm2+=ty*ty; 
								cm1+=tx*ty;
								//buffer[pos] = 0; 
							}
							float fm0,fm1,fm2;
							fm0 = ((float)cm0)/queueEnd;
							fm1 = ((float)cm1)/queueEnd;
							fm2 = ((float)cm2)/queueEnd;
							float f0 = ((fm0+fm2)+sqrtf((fm0+fm2)*(fm0+fm2)-4*(fm0*fm2-fm1*fm1)))/2;
							float f1 = ((fm0+fm2)-sqrtf((fm0+fm2)*(fm0+fm2)-4*(fm0*fm2-fm1*fm1)))/2;
							segmentArray[numSegments-1].m0 = sqrtf(f0);
							segmentArray[numSegments-1].m1 = sqrtf(f1);
							segmentArray[numSegments-1].v0 = -fm1/sqrtf(fm1*fm1+(fm0-f0)*(fm0-f0));
							segmentArray[numSegments-1].v1 = (fm0-f0)/sqrtf(fm1*fm1+(fm0-f0)*(fm0-f0));
							segmentArray[numSegments-1].bwRatio = (float)segmentArray[numSegments-2].size/segmentArray[numSegments-1].size;

							if (track) ii = start -1;
							sizer+=segmentArray[numSegments-2].size + segmentArray[numSegments-1].size; //for debugging
							sizerAll+=len; 								    //for debugging
							float circularity = M_PI*4*(segmentArray[numSegments-1].m0)*(segmentArray[numSegments-1].m1)/queueEnd;
							if (circularity < 1.02 && circularity > 0.98){
								segmentArray[numSegments-2].valid = segmentArray[numSegments-1].valid = true;
                threshold = (segmentArray[numSegments-2].mean+segmentArray[numSegments-1].mean)/2;
								if (debug) fprintf(stdout,"Circularity: %i %03f %03f %03f \n",queueEnd,M_PI*4*(segmentArray[numSegments-1].m0)*(segmentArray[numSegments-1].m1)/queueEnd,M_PI*4*(segmentArray[numSegments-1].m0)*(segmentArray[numSegments-1].m1),segmentArray[numSegments-1].x/1000.0);
                
                //pixel leakage correction
								float r = diameterRatio*diameterRatio;
								float m0o = sqrt(f0);
								float m1o = sqrt(f1);
								float ratio = (float)segmentArray[numSegments-1].size/(segmentArray[numSegments-2].size + segmentArray[numSegments-1].size);
								float m0i = sqrt(ratio)*m0o;
								float m1i = sqrt(ratio)*m1o;
								float a = (1-r);
								float b = -(m0i+m1i)-(m0o+m1o)*r;
								float c = (m0i*m1i)-(m0o*m1o)*r;
							 	float t = (-b-sqrt(b*b-4*a*c))/(2*a);
							 	//float t = (m0i-diameterRatio*m0o)/(diameterRatio+1);
								m0i-=t;m1i-=t;m0o+=t;m1o+=t;
								fprintf(stdout,"UUU: %f %f %f %f %f\n",t,ratio,(m1i-t)/(m1o+t)*0.14,(m0i-t)/(m0o+t)*0.14,(m0o*m1o-m0i*m1i)/(m0i*m1i));
								segmentArray[numSegments-1].m0 = sqrt(f0)+t;
								segmentArray[numSegments-1].m1 = sqrt(f1)+t;
                segmentArray[numSegments-1].minx = segmentArray[numSegments-2].minx;
                segmentArray[numSegments-1].maxx = segmentArray[numSegments-2].maxx;
                segmentArray[numSegments-1].maxy = segmentArray[numSegments-2].maxy;
                segmentArray[numSegments-1].miny = segmentArray[numSegments-2].miny;
							
							}
            }
					}
				}
			}
		}
		ii++;
		if (ii >= len) ii = 0;
		cont = (ii != start);
	}

  for (int i = 0;i< numSegments;i++) {
		if (segmentArray[i].size > minSize && segmentArray[i].valid) result = segmentArray[i];
			//if (debug) fprintf(stdout,"Segment %i Type: %i Pos: %.2f %.2f Area: %i Vx: %i Vy: %i Mean: %i Thr: %i Eigen: %03f %03f %03f Roundness: %03f\n",i,segmentArray[i].type,segmentArray[i].x,segmentArray[i].y,segmentArray[i].size,segmentArray[i].maxx-segmentArray[i].minx,segmentArray[i].maxy-segmentArray[i].miny,segmentArray[i].mean,threshold,segmentArray[i].m0,segmentArray[i].m1,M_PI*4*segmentArray[i].m0*segmentArray[i].m1,segmentArray[i].roundness);
	}
  
  if (numSegments == 2 && segmentArray[0].valid && segmentArray[1].valid) lastTrackOK = true; else lastTrackOK = false;

	//Drawing results 
	if (result.valid){
		lastThreshold = threshold;
		drawAll = false;
		numFailed = 0;	
	}else if (numFailed < maxFailed){
		if (numFailed++%2 == 0) changeThreshold(); else threshold = lastThreshold;
		if (debug) drawAll = true;
	}else{
		numFailed++;
		if (changeThreshold()==false) numFailed = 0;
		if (debug) drawAll = true;
	}
  if (draw) {
    for (int i = 0;i<len;i++){
      int j = buffer[i];
      if (j > 0){
        if (drawAll || segmentArray[j-1].valid){
          image.data[i*3+j%3] = 0;
          image.data[i*3+(j+1)%3] = 0;
          image.data[i*3+(j+2)%3] = 0;
        }
      }
    }
  }
  
  //if (result.valid) improveEllipse(original_image, result); 
  cout << "here" << endl;
	return result;
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

void cv::CircleDetector::Circle::draw(cv::Mat& image, const std::string& text, cv::Scalar color) const
{
  cv::ellipse(image, cv::Point(x, y), cv::Size((int)m0 * 2, (int)m1 * 2), atan2(v1, v0)  * 180.0 / M_PI, 0, 360, color, 2, CV_AA);
  float scale = image.size().width / 1600.0f;
  float thickness = scale * 3.0;
  cv::putText(image, text.c_str(), cv::Point(x + 2 * m0, y + 2 * m1), CV_FONT_HERSHEY_SIMPLEX, scale, cv::Scalar(255,255,0), thickness, CV_AA);
}
