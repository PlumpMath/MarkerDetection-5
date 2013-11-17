#include <opencv2/opencv.hpp>
#include <stdio.h>
#include <string.h>
#include <cstdlib>
#include <sstream>
#include <stdlib.h>

//read me https://code.ros.org/trac/opencv/browser/trunk/opencv/modules/imgproc/src/hough.cpp?rev=3934
//and me http://citeseerx.ist.psu.edu/viewdoc/download?doi=10.1.1.59.4239&rep=rep1&type=pdf

using namespace cv;
using namespace std;

void on_trackbar( int, void* );
vector<Vec2f> detectRectanglePatterns(std::vector<cv::Vec2f> lines, double Tq, double Tp, double Tl);
Mat contours;
Mat img;
Mat gray;
const int canny= 5*255;
int cannyMin = 0;
int cannyMax = 0;
Mat dd;
cv::Point2f computeIntersect(cv::Vec4i a, cv::Vec4i b)
{
	int x1 = a[0], y1 = a[1], x2 = a[2], y2 = a[3], x3 = b[0], y3 = b[1], x4 = b[2], y4 = b[3];
	float denom;

	if (float d = ((float)(x1 - x2) * (y3 - y4)) - ((y1 - y2) * (x3 - x4)))
	{
		cv::Point2f pt;
		pt.x = ((x1 * y2 - y1 * x2) * (x3 - x4) - (x1 - x2) * (x3 * y4 - y3 * x4)) / d;
		pt.y = ((x1 * y2 - y1 * x2) * (y3 - y4) - (y1 - y2) * (x3 * y4 - y3 * x4)) / d;
		return pt;
	}
	else
		return cv::Point2f(-1, -1);
}

int main()
{
	img = imread("cut0.png");

	img.copyTo(dd);
	cvtColor(img, gray, CV_RGB2GRAY);
	//								namedWindow("Canny", 1);
	//								createTrackbar( "low: ", "Canny", &cannyMin, canny, on_trackbar );
	//								createTrackbar( "high: ", "Canny", &cannyMax, canny, on_trackbar );
	//								on_trackbar( canny, 0 );

	//get the edges
	//	blur(gray, gray, Size(3, 3), Point(0,0), 0);


	Mat edges;
	Canny(gray, edges, 74, 22, 3);

	//	imshow("canny", edges);
	//	waitKey(0);

	std::vector<cv::Vec4i> lines;
	vector<Vec2f> s_lines;
	//	cv::HoughLinesP(edges, lines, 1, CV_PI/180, 20, 10, 10);
	HoughLines( edges, s_lines, 1, CV_PI/180, 30);

	//	for( size_t i = 0; i < s_lines.size(); i++ )
	//	{
	//		float r = s_lines[i][0], t = s_lines[i][1];
	//		double cos_t = cos(t), sin_t = sin(t);
	//		double x0 = r*cos_t, y0 = r*sin_t;
	//		double alpha = 1000;
	//
	//		Point pt1( cvRound(x0 + alpha*(-sin_t)), cvRound(y0 + alpha*cos_t) );
	//		Point pt2( cvRound(x0 - alpha*(-sin_t)), cvRound(y0 - alpha*cos_t) );
	//		line( dd, pt1, pt2, Scalar(255,0,0), 1);
	//		cout << "rho: " << r << ", theta: " << t << endl;
	//		imshow("", dd);
	//		waitKey(0);
	//	}


	//vizualize found lines
	//	for( size_t i = 0; i < lines.size(); i++ )
	//	{
	//		Vec4i l = lines[i];
	//		float length = sqrt((l[2] - l[0]) * (l[2] - l[0]) + (l[3] - l[1])*(l[3] - l[1]));
	//		if (length > 30)
	//			line( dd, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0,0,255), 2);
	//	}
	//	cout << "line size: " << lines .size() << endl;

	//	imshow("visualize lines", dd);


	detectRectanglePatterns(s_lines, 0.02, 0.02, 0.5);


	waitKey(0);
	return 0;
}

class MyLine{
public:

	int name;
	float rho = 0.0f;
	float theta = 0.0f;

	vector<int> parallelLines;
	vector<int> perpendicularLines;

	Scalar getColor() {
		return CV_RGB(rand() % 256, rand() % 256, rand() % 256);
	}
};


MyLine getLineParameters(float rho, float theta, int num)
{
	MyLine _line;

	_line.rho = rho;
	_line.theta = theta;

	_line.name = num;

	return _line;
}



//-------------------------------------------------------------------
vector<Vec2f> detectRectanglePatterns(std::vector<cv::Vec2f> lines, double Tq, double Tp, double Tl){
	vector<Vec2f> dst;

	//store each line values
	vector<MyLine> linearray;
	for( size_t i = 0; i < lines.size(); i++ )
	{
		MyLine ll = getLineParameters(lines[i][0], lines[i][1], i);
		linearray.push_back(ll);
	}

	int pair = 1;
	for( size_t i = 0; i < lines.size(); i++ )
	{
		MyLine* firstLine = &linearray[i];
		//find pair
		for( size_t j = 0; j < lines.size(); j++ )
		{
			if (i != j)
			{
				MyLine* secondLine = &linearray[j];

				//check 3 equations

				if ( abs(firstLine-> theta - secondLine-> theta) < Tq)
				{
					//						cout << "m: " << firstLine->theta  << " second m: " << secondLine->theta<< ", name: "<< firstLine->name <<", is parallel to " << secondLine->name<< ", "<<endl;
					firstLine->parallelLines.push_back(j);
				}
				else if (abs(firstLine->theta - secondLine->theta) > 1.56 || abs(firstLine->theta - secondLine->theta) < 1.6)
				{
					firstLine->perpendicularLines.push_back(j);
				}
			}
		}

		Scalar color = firstLine->getColor();

		if (firstLine->parallelLines.size() > 1 &&
				firstLine->perpendicularLines.size() > 1)
		{
			cout << "line: " << firstLine->name <<" par: " << firstLine->parallelLines.size() << ", perp: " << firstLine->perpendicularLines.size() << endl;
			//draw lines here
			for (int u = 0;u < firstLine->parallelLines.size(); u ++){
				int number = firstLine->parallelLines[u];

				MyLine _tmp = getLineParameters(lines[number][0], lines[number][1], number);
				Vec2f l = lines[firstLine->parallelLines[u]];

				float r = firstLine->rho, t = firstLine->theta;
				double cos_t = cos(t), sin_t = sin(t);
				double x0 = r*cos_t, y0 = r*sin_t;
				double alpha = 1000;

				Point pt1( cvRound(x0 + alpha*(-sin_t)), cvRound(y0 + alpha*cos_t) );
				Point pt2( cvRound(x0 - alpha*(-sin_t)), cvRound(y0 - alpha*cos_t) );
				line( img, pt1, pt2, color, 1);

				int fontFace = FONT_HERSHEY_SCRIPT_SIMPLEX;
				double fontScale = 0.5;
				int thickness = 1;
				std::ostringstream lineNo1, lineNo2;
				lineNo1 << pair << " - " << _tmp.name;
				cv::putText(img, lineNo1.str(), Point(l[0], l[1]), fontFace, fontScale, Scalar::all(255), thickness, 8);
			}

			for (int u = 0;u < firstLine->perpendicularLines.size(); u ++){
				int number = firstLine->perpendicularLines[u];

				MyLine _tmp = getLineParameters(lines[number][0], lines[number][1], number);
				Vec2f l = lines[firstLine->perpendicularLines[u]];

				float r = firstLine->rho, t = firstLine->theta;
				double cos_t = cos(t), sin_t = sin(t);
				double x0 = r*cos_t, y0 = r*sin_t;
				double alpha = 100;

				Point pt1( cvRound(x0 + alpha*(-sin_t)), cvRound(y0 + alpha*cos_t) );
				Point pt2( cvRound(x0 - alpha*(-sin_t)), cvRound(y0 - alpha*cos_t) );
				line( img, pt1, pt2, color, 1);

				int fontFace = FONT_HERSHEY_SCRIPT_SIMPLEX;
				double fontScale = 0.5;
				int thickness = 1;
				std::ostringstream lineNo1, lineNo2;
				lineNo1 << pair << " - " << _tmp.name;
				cv::putText(img, lineNo1.str(), Point(l[0], l[1]), fontFace, fontScale, Scalar::all(255), thickness, 8);
			}
			imshow("pair", img);
			waitKey();
			pair++;
		}
	}

	return dst;
}
//************************************************



void on_trackbar( int, void* )
{
	Canny(gray, contours, cannyMin, cannyMax, 3);
	imshow("Canny", contours);
}
