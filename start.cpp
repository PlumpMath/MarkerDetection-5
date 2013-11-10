#include <opencv2/opencv.hpp>
#include <stdio.h>
#include <string.h>
#include <cstdlib>
#include <sstream>

using namespace cv;
using namespace std;

void on_trackbar( int, void* );
void drawCorners(Mat dst, vector<Point2f> corners);
vector<Point2f> removeUselessCorners(vector<Point2f> corners, int sizeOfCornerMin, int sizeOfCornerMax);
vector<Vec4i> findParallelLines(std::vector<cv::Vec4i> lines, int max, int min);
Mat contours;
Mat img;
const int canny= 4*255;
int cannyMin = 0;
int cannyMax = 0;

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
	img = imread("0.png");
	Mat gray;
	cvtColor(img, gray, CV_RGB2GRAY);
	//		namedWindow("Canny", 1);
	//		createTrackbar( "low: ", "Canny", &cannyMin, canny, on_trackbar );
	//		createTrackbar( "high: ", "Canny", &cannyMax, canny, on_trackbar );
	//		on_trackbar( canny, 0 );

	//	HoughLineDetection(img, 195, 350);
	//	//HoughLineDetectionPrbabilistic(img, 195, 350);

	//get the edges
	Mat edges;
	Canny(gray, edges, 700, 550, 5);
	imshow("canny", edges);

	std::vector<cv::Vec4i> lines;
	cv::HoughLinesP(edges, lines, 1, CV_PI/360, 20, 70, 10);

	//vizualize found lines
	for( size_t i = 0; i < lines.size(); i++ )
	{
		Vec4i l = lines[i];
		line( img, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0,0,255), 2);
	}
	cout << "line size: " << lines .size() << endl;

	imshow("img", img);

	std::vector<cv::Point2f> corners;
	for (int i = 0; i < lines.size(); i++)
	{
		for (int j = i+1; j < lines.size(); j++)
		{
			cv::Point2f pt = computeIntersect(lines[i], lines[j]);
			if (pt.x >= 0 && pt.y >= 0) corners.push_back(pt);
		}
	}
	cout << "amount of corners: " << corners.size()<< endl;

	vector<Point2f> newCorners = removeUselessCorners(corners, 30, 45);
	cout << "amount of new corners: " << newCorners.size()<< endl;
	//	drawCorners(img, corners);
	//
	//	// Get mass center
	//	Point2f center(0,0);
	//	for (int i = 0; i < corners.size(); i++)
	//		center += corners[i];
	//	center *= (1. / corners.size());
	//
	//	circle(img, center, 3, CV_RGB(255,255,255), 2);
	//	imshow("center", img);
	waitKey(0);
	return 0;
}

vector<Point2f> removeUselessCorners(vector<Point2f> corners, int sizeOfCornerMin, int sizeOfCornerMax)
						{
	vector<Point2f> dst;
	for (int i = 0; i < corners.size(); i++)
	{
		bool add = false;
		Point2f current_corner = corners[i];
		for (int j = 0; j < corners.size(); j++)
		{
			if (j != i)
			{
				if (abs(current_corner.x - corners[j].x) <= sizeOfCornerMax &&
						abs(current_corner.x - corners[j].x) > sizeOfCornerMin &&
						abs(current_corner.y - corners[j].y) <= sizeOfCornerMax &&
						abs(current_corner.y - corners[j].y) > sizeOfCornerMin)
				{
					add = true;
					break;
				}
			}

		}
		if (add) dst.push_back(current_corner);
	}
	return dst;
						}

void drawCorners(Mat dst, vector<Point2f> corners)
{
	circle(dst, corners[0], 3, CV_RGB(255,0,0), 2);
	circle(dst, corners[1], 3, CV_RGB(0,255,0), 2);
	circle(dst, corners[2], 3, CV_RGB(0,0,255), 2);
	circle(dst, corners[3], 3, CV_RGB(0,0,0), 2);
	imshow("corners", dst);
}

vector<Vec4i> findParallelLines(std::vector<cv::Vec4i> lines, int max, int min)
				{
	vector<Vec4i> dst;
	for( size_t i = 0; i < lines.size(); i++ )
	{
		//transfer to Cartesian coordinates
		float rho = lines[i][0], theta = lines[i][1];
		Point pt1, pt2;
		double a = cos(theta), b = sin(theta);
		double x0 = a*rho, y0 = b*rho;
		pt1.x = cvRound(x0 + 1000*(-b)); //the first point
		pt1.y = cvRound(y0 + 1000*(a)); //the first point
		pt2.x = cvRound(x0 - 1000*(-b)); //the second point
		pt2.y = cvRound(y0 - 1000*(a)); //the second point
		cout << "x1: " << pt1.x;
		bool found = false;
		//find second point
		for( size_t j = 0; j < lines.size(); j++ )
		{
			if (i != j)
			{
				float rho = lines[i][0], theta = lines[i][1];
				Point pt1, pt2;
				double a = cos(theta), b = sin(theta);
				double x0 = a*rho, y0 = b*rho;
				pt1.x = cvRound(x0 + 1000*(-b)); //the first point
				pt1.y = cvRound(y0 + 1000*(a)); //the first point
				pt2.x = cvRound(x0 - 1000*(-b)); //the second point
				pt2.y = cvRound(y0 - 1000*(a)); //the second point
			}
		}
	}

	return dst;
				}

void on_trackbar( int, void* )
{
	Canny(img, contours, cannyMin, cannyMax);
	imshow("Canny", contours);
}
