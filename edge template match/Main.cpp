//***********************************************************************
// Project		    : GeoMatch
// Author           : Shiju P K
// Email			: shijupk@gmail.com
// Created          : 10-01-2010
//
// File Name		: main.cpp
// Last Modified By : Shiju P K
// Last Modified On : 13-07-2010
// Description      : Defines the entry point for the console application.
//
// Copyright        : (c) . All rights reserved.
//***********************************************************************
//

#include "stdafx.h"
#include <iostream>
#include <time.h>

#include "GeoMatch.h"
#include "CommandParser.h"

using namespace std;
int main(int argc, char** argv)
{
	struct GeoMatch GM;			
	int lowThreshold ,highThreashold;	
	double minScore, greediness;		

	double total_time =0;
	cv::Mat templateImage = cv::imread("2.png", 0);
	cv::Mat searchImage = cv::imread("1.png", 0);
	if (templateImage.data == NULL || searchImage.data == NULL)
	{
		return 0;
	}
	//初始化
	lowThreshold = 40;
	highThreashold = 80;//get high threshold
	minScore = 0.7;
	greediness = 0.8;
	
	cv::Mat grayTemplateImg = cv::Mat(templateImage.size(), CV_8UC1);
	// Convert color image to gray image.
	if(templateImage.channels() == 3)
	{
		cv::cvtColor(templateImage, grayTemplateImg, cv::COLOR_RGB2GRAY);
	}
	else
	{
		grayTemplateImg = templateImage.clone();
	}
	cout<< "\n Edge Based Template Matching Program\n";
	cout<< " ------------------------------------\n";
	
	double time = cv::getTickCount();
	if(!GM.CreateGeoMatchModel(grayTemplateImg,lowThreshold,highThreashold))
	{
		cout<<"ERROR: could not create model...";
		return 0;
	}
	GM.DrawContours(templateImage, 0);
	cout<<" Shape model created.."<<"with  Low Threshold = "<<lowThreshold<<" High Threshold = "<<highThreashold<<endl;
	cv::Size searchSize = cv::Size( searchImage.cols, searchImage.rows );
	cv::Mat graySearchImg = cv::Mat(searchSize, CV_8UC1);

	// Convert color image to gray image. 
	if (searchImage.channels() == 3)
	{
		cv::cvtColor(searchImage, graySearchImg, cv::COLOR_RGB2GRAY);
	}
	else
	{
		graySearchImg = searchImage.clone();
	}
	cout<<" Finding Shape Model.."<<" Minumum Score = "<< minScore <<" Greediness = "<<greediness<<"\n\n";
	cout<< " ------------------------------------\n";
	clock_t start_time1 = clock();
	//score = GM.FindGeoMatchModel(graySearchImg,minScore,greediness,&result);
	cv::Mat dst = cv::Mat::zeros(graySearchImg.size(), CV_32FC1);
	GM.FindGeoMatchModel(graySearchImg, dst, minScore, greediness);

	cv::Mat binary = dst > 0.6;
	int an = 5;
	cv::Mat element = getStructuringElement(cv::MORPH_RECT, cv::Size(an * 2 + 1, an * 2 + 1), cv::Point(an, an));
	cv::morphologyEx(binary, binary, cv::MORPH_CLOSE, element);
	std::vector<std::vector<cv::Point> > contours;
	cv::findContours(binary, contours, cv::RETR_CCOMP, cv::CHAIN_APPROX_NONE);
	if (contours.size() <= 0)
	{
		return 0;
	}
	
	cv::Rect tempRect;
	double minVal, maxVal; 
	cv::Point minLoc, maxLoc, matchLoc;
	
	cv::Mat tt = cv::Mat(graySearchImg.size(), CV_8UC1);
	tt = graySearchImg.clone();
	for (int i = 0; i < contours.size(); i++)
	{
		tempRect = cv::boundingRect(cv::Mat(contours[i]));
		cv::minMaxLoc(dst(tempRect), &minVal, &maxVal, &minLoc, &maxLoc, cv::Mat());
		maxLoc += cv::Point(tempRect.x, tempRect.y);
		//cv::circle(tt, maxLoc, 3, cv::Scalar(255));
		GM.DrawContours(tt, maxLoc, 0);
	}

	time = (cv::getTickCount() - time) * 1000.0 / cv::getTickFrequency();
	printf("检测耗费时间为:%f毫秒", time);
	return 1;
}


void WrongUsage()
{
	cout<< "\n Edge Based Template Matching Program\n" ;
	cout<< " ------------------------------------\n" ;
	cout<< "\nProgram arguments:\n\n";
	cout<< "     -t Template image name (image to be searched)\n\n" ;
	cout<< "     -h High Threshold (High threshold for creating template model)\n\n" ;
	cout<< "     -l Low Threshold (Low threshold for creating template model)\n\n" ;
	cout<< "     -s Search image name (image we are trying to find)\n\n" ;
	cout<< "     -m Minumum score (Minimum score required to proceed with search [0.0 to 1.0])\n\n" ;
	cout<< "     -g greediness (heuistic parameter to terminate search [0.0 to 1.0] )\n\n" ;

	cout<< "Example: GeoMatch -t Template.jpg -h 100 -l 10 -s Search1.jpg -m 0.7 -g 0.5 \n\n" ;
}

