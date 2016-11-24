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
#include <cv.h>
#include <cxcore.h>
#include <highgui.h>
#include <math.h>

class GeoMatch
{
private:
	int				noOfCordinates;		//Number of elements in coordinate array
	CvPoint			*cordinates;		//Coordinates array to store model points	
	int				modelHeight;		//Template height
	int				modelWidth;			//Template width
	double			*edgeMagnitude;		//gradient magnitude
	double			*edgeDerivativeX;	//gradient in X direction
	double			*edgeDerivativeY;	//radient in Y direction	
	CvPoint			centerOfGravity;	//Center of gravity of template 
	
	bool			modelDefined;
	
	void CreateDoubleMatrix(double **&matrix,CvSize size);
	void ReleaseDoubleMatrix(double **&matrix,int size);
public:
	GeoMatch(void);
	GeoMatch(const void* templateArr);
	~GeoMatch(void);

	int CreateGeoMatchModel(const void* templateArr,double,double);
	double FindGeoMatchModel(const void* srcarr,double minScore,double greediness, CvPoint *resultPoint);
	void DrawContours(IplImage* pImage,CvPoint COG,CvScalar,int);
	void DrawContours(IplImage* pImage,CvScalar,int);
};
