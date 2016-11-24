//***********************************************************************
// Project		    : GeoMatch
// Author           : Shiju P K
// Email			: shijupk@gmail.com
// Created          : 10-01-2010
//
// File Name		: GeoMatch.h
// Last Modified By : Shiju P K
// Last Modified On : 13-07-2010
// Description      : class to implement edge based template matching
//
// Copyright        : (c) . All rights reserved.
//***********************************************************************

#pragma once
#include <math.h>
#include <opencv2/opencv.hpp>

class GeoMatch
{
private:
	int				m_noOfCordinates;		//Number of elements in coordinate array
	cv::Point		*m_cordinates;		//Coordinates array to store model points	
	int				m_modelHeight;		//Template height
	int				m_modelWidth;			//Template width
	double			*m_edgeMagnitude;		//gradient magnitude
	double			*m_edgeDerivativeX;	//gradient in X direction
	double			*m_edgeDerivativeY;	//radient in Y direction	
	cv::Point		m_centerOfGravity;	//Center of gravity of template 
	
	bool			m_modelDefined;
	
	void CreateDoubleMatrix(double **&matrix,CvSize size);
	void ReleaseDoubleMatrix(double **&matrix,int size);
public:
	GeoMatch(void);
	GeoMatch(const void* templateArr);
	~GeoMatch(void);

	int CreateGeoMatchModel(cv::Mat& src, double, double);
	void GeoMatch::DrawContours(cv::Mat& source, cv::Point COG, int color);
	void DrawContours(cv::Mat& source, int color);
	void FindGeoMatchModel(cv::Mat& src, cv::Mat& dst, double minScore, double greediness);

};
