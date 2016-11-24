//***********************************************************************
// Project		    : GeoMatch
// Author           : Shiju P K
// Email			: shijupk@gmail.com
// Created          : 10-01-2010
//
// File Name		: GeoMatch.cpp
// Last Modified By : Shiju P K
// Last Modified On : 13-07-2010
// Description      : class to implement edge based template matching
//
// Copyright        : (c) . All rights reserved.
//***********************************************************************

#include "StdAfx.h"
#include "GeoMatch.h"


GeoMatch::GeoMatch(void)
{
	m_noOfCordinates = 0;  // Initilize  no of cppodinates in model points
	m_modelDefined = false; 
}


int GeoMatch::CreateGeoMatchModel(cv::Mat& src, double maxContrast, double minContrast)
{
	if (src.type() != CV_8UC1)
	{
		return 0;
	}
	cv::Mat gx, gy ;	//Matrix to store X Y derivative
	cv::Mat nmsEdges;	//Matrix to store temp restult
	cv::Size Ssize;
	Ssize.width = src.cols;
	Ssize.height = src.rows;

	//类成员变量初始化
	m_modelHeight = src.rows;			//Save Template height
	m_modelWidth = src.cols;			//Save Template width
	m_noOfCordinates = 0;													//initialize	
	m_cordinates =  new cv::Point[m_modelWidth * m_modelHeight];			//Allocate memory for coorinates of selected points in template image
	m_edgeMagnitude = new double[m_modelWidth * m_modelHeight];				//Allocate memory for edge magnitude for selected points
	m_edgeDerivativeX = new double[m_modelWidth * m_modelHeight];			//Allocate memory for edge X derivative for selected points
	m_edgeDerivativeY = new double[m_modelWidth * m_modelHeight];			////Allocate memory for edge Y derivative for selected points

	//Sobel边缘检测
	cv::Sobel(src, gx, CV_16S, 1, 0, 3, 1, 0);		//gradient in X direction			
	cv::Sobel(src, gy, CV_16S, 0, 1, 3, 1, 0);		//gradient in Y direction
	
	double fdx,fdy;	
    double MagG, DirG;
	double MaxGradient=FLT_MIN;
	double direction;
	int *orients = new int[ Ssize.height *Ssize.width];
	int i, j, count = 0; // count variable;
	
	double **magMat;
	CreateDoubleMatrix(magMat ,Ssize);
	for( i = 1; i < Ssize.height-1; i++ )
	{
		short* _sdx = gx.ptr<short>(i);
		short* _sdy = gy.ptr<short>(i);
		for( j = 1; j < Ssize.width-1; j++ )
		{ 		 
			fdx = _sdx[j]; fdy = _sdy[j];        // read x, y derivatives
			MagG = sqrt((float)(fdx * fdx) + (float)(fdy * fdy)); //Magnitude = Sqrt(gx^2 +gy^2)
			direction = cvFastArctan((float)fdy, (float)fdx);	 //Direction = invtan (Gy / Gx)
			magMat[i][j] = MagG;
			if (MagG  >MaxGradient)
			{
				MaxGradient = MagG; // get maximum gradient value for normalizing.
			}
			// get closest angle from 0, 45, 90, 135 set
			if ( (direction>0 && direction < 22.5) || (direction >157.5 && direction < 202.5) || (direction>337.5 && direction<360)  )
				direction = 0;
			else if ( (direction>22.5 && direction < 67.5) || (direction >202.5 && direction <247.5)  )
				direction = 45;
			else if ( (direction >67.5 && direction < 112.5)||(direction>247.5 && direction<292.5) )
				direction = 90;
			else if ( (direction >112.5 && direction < 157.5)||(direction>292.5 && direction<337.5) )
				direction = 135;
			else 
				direction = 0;
				
			orients[count] = (int)direction;
			count++;
		}
	}
	
	count = 0; // init count
	// non maximum suppression
	double leftPixel,rightPixel;
	nmsEdges = cv::Mat(src.size(), CV_32FC1);		//create Matrix to store Final nmsEdges
	for( i = 1; i < Ssize.height-1; i++ )
	{
		float* ptr = nmsEdges.ptr<float>(i);
		for( j = 1; j < Ssize.width-1; j++ )
		{
			switch (orients[count])
			{
			case 0:
				leftPixel = magMat[i][j - 1];
				rightPixel = magMat[i][j + 1];
				break;
			case 45:
				leftPixel = magMat[i - 1][j + 1];
				rightPixel = magMat[i + 1][j - 1];
				break;
			case 90:
				leftPixel = magMat[i - 1][j];
				rightPixel = magMat[i + 1][j];
				break;
			case 135:
				leftPixel = magMat[i - 1][j - 1];
				rightPixel = magMat[i + 1][j + 1];
				break;
			}
			// compare current pixels value with adjacent pixels
			if ((magMat[i][j] < leftPixel) || (magMat[i][j] < rightPixel))
			{
				ptr[j] = 0;
			}
			else
			{
				ptr[j] = (float)(magMat[i][j] / MaxGradient * 255);
			}            
			count++;
		}
	}

	int RSum=0,CSum=0;
	int curX,curY;
	int flag=1;

	//Hysterisis threshold
	for( i = 1; i < Ssize.height - 1; i++ )
    {
		short* _sdx = gx.ptr<short>(i);
		short* _sdy = gy.ptr<short>(i);
		float* _ptrB = nmsEdges.ptr<float>(i-1);
		float* _ptr = nmsEdges.ptr<float>(i);
		float* _ptrF = nmsEdges.ptr<float>(i+1);
		for( j = 1; j < Ssize.width - 1; j++ )
        {
			fdx = _sdx[j]; fdy = _sdy[j];
			MagG = sqrt(fdx * fdx + fdy * fdy); //Magnitude = Sqrt(gx^2 +gy^2)
			DirG = cvFastArctan((float)fdy, (float)fdx);	 //Direction = tan(y/x)
			flag=1;
			if (_ptr[j] < minContrast)
			{
				_ptr[j] = 0;
				flag = 0; // remove from edge
			}
			else if (_ptr[j] < maxContrast)
			{
				if (_ptrB[j - 1] < maxContrast && _ptrB[j] < maxContrast && _ptrB[j + 1] < maxContrast
					&& _ptr[j - 1] < maxContrast && _ptr[j + 1] < maxContrast 
					&& _ptrF[j - 1] < maxContrast && _ptrF[j] < maxContrast && _ptrF[j + 1] < maxContrast)
				{
					_ptr[j] = 0;
					flag = 0;
				}
			}
			
			// save selected edge information
			curX = j;	curY = i;
			if(flag != 0)
			{
				if(fdx!=0 || fdy!=0)
				{		
					RSum=RSum + curY;	CSum=CSum + curX; // Row sum and column sum for center of gravity
					
					m_cordinates[m_noOfCordinates].x = curX;
					m_cordinates[m_noOfCordinates].y = curY;
					m_edgeDerivativeX[m_noOfCordinates] = fdx;
					m_edgeDerivativeY[m_noOfCordinates] = fdy;
					//handle divide by zero
					if(MagG!=0)
						m_edgeMagnitude[m_noOfCordinates] = 1/MagG;  // gradient magnitude 
					else
						m_edgeMagnitude[m_noOfCordinates] = 0;
															
					m_noOfCordinates++;
				}
			}
		}
	}

	m_centerOfGravity.x = CSum / m_noOfCordinates; // center of gravity
	m_centerOfGravity.y = RSum / m_noOfCordinates ;	// center of gravity
		
	// change coordinates to reflect center of gravity
	for(int m = 0;m < m_noOfCordinates; m++)
	{
		m_cordinates[m].x -= m_centerOfGravity.x;
		m_cordinates[m].y -= m_centerOfGravity.y;
	}
	
	// free alocated memories
	delete[] orients;
	ReleaseDoubleMatrix(magMat ,Ssize.height);
	m_modelDefined = true;
	return 1;
}

void GeoMatch::FindGeoMatchModel(cv::Mat& src, cv::Mat& dst, double minScore, double greediness)
{
	dst = cv::Mat::zeros(src.size(), CV_32FC1);
	if (src.type() != CV_8UC1 || !m_modelDefined)
	{
		return;
	}
	double resultScore = 0;
	double partialSum = 0;
	double sumOfCoords = 0;
	double partialScore;
	int i, j, m;			// count variables
	double iTx, iTy, iSx, iSy;
	double gradMag;
	int curX, curY;

	double **matGradMag;  //Gradient magnitude matrix
	cv::Size Ssize;
	Ssize.width = src.cols;
	Ssize.height = src.rows;
	CreateDoubleMatrix(matGradMag, Ssize); // create image to save gradient magnitude  values
	cv::Mat Sdx, Sdy;
	cv::Sobel(src, Sdx, CV_16S, 1, 0, 3, 1, 0);  // find X derivatives
	cv::Sobel(src, Sdy, CV_16S, 0, 1, 3, 1, 0); // find Y derivatives

	// stoping criterias to search for model
	double normMinScore = minScore / m_noOfCordinates; // precompute minumum score 
	double normGreediness = ((1 - greediness * minScore) / (1 - greediness)) / m_noOfCordinates; // precompute greedniness 

	for (i = 0; i < Ssize.height; i++)
	{
		short* _Sdx = Sdx.ptr<short>(i);
		short* _Sdy = Sdy.ptr<short>(i);

		for (j = 0; j < Ssize.width; j++)
		{
			iSx = _Sdx[j];  // X derivative of Source image
			iSy = _Sdy[j];  // Y derivative of Source image

			gradMag = sqrt((iSx*iSx) + (iSy*iSy)); //Magnitude = Sqrt(dx^2 +dy^2)

			if (gradMag != 0) // hande divide by zero
				matGradMag[i][j] = 1 / gradMag;   // 1/Sqrt(dx^2 +dy^2)
			else
				matGradMag[i][j] = 0;

		}
	}
	for (i = 0; i < Ssize.height; i++)
	{
		float* ptr = dst.ptr<float>(i);
		for (j = 0; j < Ssize.width; j++)
		{
			partialSum = 0; // initilize partialSum measure
			for (m = 0; m < m_noOfCordinates; m++)
			{
				curX = j + m_cordinates[m].x;	// template X coordinate
				curY = i + m_cordinates[m].y; // template Y coordinate
				iTx = m_edgeDerivativeX[m];	// template X derivative
				iTy = m_edgeDerivativeY[m];    // template Y derivative

				if (curX < 0 || curY < 0 || curY > Ssize.height - 1 || curX > Ssize.width - 1)
					continue;

				iSx = Sdx.at<short>(curY, curX);// get curresponding  X derivative from source image
				iSy = Sdy.at<short>(curY, curX);// get curresponding  Y derivative from source image
				if ((iSx != 0 || iSy != 0) && (iTx != 0 || iTy != 0))
				{
					partialSum = partialSum + ((iSx*iTx) + (iSy*iTy))*(m_edgeMagnitude[m] * matGradMag[curY][curX]);

				}
				sumOfCoords = m + 1;
				partialScore = partialSum / sumOfCoords;
				// check termination criteria
				// if partial score score is less than the score than needed to make the required score at that position
				// break serching at that coordinate.
				if (partialScore < (MIN((minScore - 1) + normGreediness*sumOfCoords, normMinScore*  sumOfCoords)))
					break;

			}
			ptr[j] = partialScore;
		}
	}

	// free used resources and return score
	ReleaseDoubleMatrix(matGradMag, Ssize.height);
}
// destructor
GeoMatch::~GeoMatch(void)
{
	delete[] m_cordinates ;
	delete[] m_edgeMagnitude;
	delete[] m_edgeDerivativeX;
	delete[] m_edgeDerivativeY;
}

//allocate memory for doubel matrix
void GeoMatch::CreateDoubleMatrix(double **&matrix,CvSize size)
{
	matrix = new double*[size.height];
	for(int iInd = 0; iInd < size.height; iInd++)
		matrix[iInd] = new double[size.width];
}
// release memory
void GeoMatch::ReleaseDoubleMatrix(double **&matrix,int size)
{
	for(int iInd = 0; iInd < size; iInd++) 
        delete[] matrix[iInd]; 
}


// draw contours around result image
void GeoMatch::DrawContours(cv::Mat& source, cv::Point COG, int color)
{
	cv::Point point;
	point.y = COG.x;
	point.x = COG.y;
	for(int i = 0; i < m_noOfCordinates; i++)
	{	
		point.x = m_cordinates[i].x + COG.x;
		point.y = m_cordinates[i].y + COG.y;
		if (point.x >= 0 && point.x < source.cols && point.y >= 0 && point.y < source.rows)
		{
			source.at<uchar>(point) = color;
		}
	}
}

// draw contour at template image
void GeoMatch::DrawContours(cv::Mat& source, int color)
{
	cv::Point point;
	for(int i = 0; i < m_noOfCordinates; i++)
	{
		point.x = m_cordinates[i].x + m_centerOfGravity.x;
		point.y = m_cordinates[i].y + m_centerOfGravity.y;
		source.at<uchar>(point) = color;
	}
}

