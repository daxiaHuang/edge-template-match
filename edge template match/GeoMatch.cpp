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
	noOfCordinates = 0;  // Initilize  no of cppodinates in model points
	modelDefined = false; 
}


int GeoMatch::CreateGeoMatchModel(const void *templateArr,double maxContrast,double minContrast)
{

	CvMat *gx = 0;		//Matrix to store X derivative
	CvMat *gy = 0;		//Matrix to store Y derivative
	CvMat *nmsEdges = 0;		//Matrix to store temp restult
	CvSize Ssize;
		
	// Convert IplImage to Matrix for integer operations
	CvMat srcstub, *src = (CvMat*)templateArr;
	src = cvGetMat( src, &srcstub );
	if(CV_MAT_TYPE( src->type ) != CV_8UC1)
	{	
		return 0;
	}
	
	// set width and height
	Ssize.width =  src->width;
	Ssize.height= src->height;
	modelHeight =src->height;		//Save Template height
	modelWidth =src->width;			//Save Template width

	noOfCordinates=0;											//initialize	
	cordinates =  new CvPoint[ modelWidth *modelHeight];		//Allocate memory for coorinates of selected points in template image
	
	edgeMagnitude = new double[ modelWidth *modelHeight];		//Allocate memory for edge magnitude for selected points
	edgeDerivativeX = new double[modelWidth *modelHeight];			//Allocate memory for edge X derivative for selected points
	edgeDerivativeY = new double[modelWidth *modelHeight];			////Allocate memory for edge Y derivative for selected points


	// Calculate gradient of Template
	gx = cvCreateMat( Ssize.height, Ssize.width, CV_16SC1 );		//create Matrix to store X derivative
	gy = cvCreateMat( Ssize.height, Ssize.width, CV_16SC1 );		//create Matrix to store Y derivative
	cvSobel( src, gx, 1,0, 3 );		//gradient in X direction			
	cvSobel( src, gy, 0, 1, 3 );	//gradient in Y direction
	
	nmsEdges = cvCreateMat( Ssize.height, Ssize.width, CV_32F);		//create Matrix to store Final nmsEdges
	const short* _sdx; 
	const short* _sdy; 
	double fdx,fdy;	
    double MagG, DirG;
	double MaxGradient=-99999.99;
	double direction;
	int *orients = new int[ Ssize.height *Ssize.width];
	int count = 0,i,j; // count variable;
	
	double **magMat;
	CreateDoubleMatrix(magMat ,Ssize);
	
	for( i = 1; i < Ssize.height-1; i++ )
    {
    	for( j = 1; j < Ssize.width-1; j++ )
        { 		 
				_sdx = (short*)(gx->data.ptr + gx->step*i);
				_sdy = (short*)(gy->data.ptr + gy->step*i);
				fdx = _sdx[j]; fdy = _sdy[j];        // read x, y derivatives

				MagG = sqrt((float)(fdx*fdx) + (float)(fdy*fdy)); //Magnitude = Sqrt(gx^2 +gy^2)
				direction =cvFastArctan((float)fdy,(float)fdx);	 //Direction = invtan (Gy / Gx)
				magMat[i][j] = MagG;
				
				if(MagG>MaxGradient)
					MaxGradient=MagG; // get maximum gradient value for normalizing.

				
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
	
	count=0; // init count
	// non maximum suppression
	double leftPixel,rightPixel;
	
	for( i = 1; i < Ssize.height-1; i++ )
    {
		for( j = 1; j < Ssize.width-1; j++ )
        {
				switch ( orients[count] )
                {
                   case 0:
                        leftPixel  = magMat[i][j-1];
                        rightPixel = magMat[i][j+1];
                        break;
                    case 45:
                        leftPixel  = magMat[i-1][j+1];
						rightPixel = magMat[i+1][j-1];
                        break;
                    case 90:
                        leftPixel  = magMat[i-1][j];
                        rightPixel = magMat[i+1][j];
                        break;
                    case 135:
                        leftPixel  = magMat[i-1][j-1];
                        rightPixel = magMat[i+1][j+1];
                        break;
				 }
				// compare current pixels value with adjacent pixels
                if (( magMat[i][j] < leftPixel ) || (magMat[i][j] < rightPixel ) )
					(nmsEdges->data.ptr + nmsEdges->step*i)[j]=0;
                else
                    (nmsEdges->data.ptr + nmsEdges->step*i)[j]=(uchar)(magMat[i][j]/MaxGradient*255);
			
				count++;
			}
		}
	

	int RSum=0,CSum=0;
	int curX,curY;
	int flag=1;

	//Hysterisis threshold
	for( i = 1; i < Ssize.height-1; i++ )
    {
		for( j = 1; j < Ssize.width; j++ )
        {
			_sdx = (short*)(gx->data.ptr + gx->step*i);
			_sdy = (short*)(gy->data.ptr + gy->step*i);
			fdx = _sdx[j]; fdy = _sdy[j];
				
			MagG = sqrt(fdx*fdx + fdy*fdy); //Magnitude = Sqrt(gx^2 +gy^2)
			DirG =cvFastArctan((float)fdy,(float)fdx);	 //Direction = tan(y/x)
		
			////((uchar*)(imgGDir->imageData + imgGDir->widthStep*i))[j]= MagG;
			flag=1;
			if(((double)((nmsEdges->data.ptr + nmsEdges->step*i))[j]) < maxContrast)
			{
				if(((double)((nmsEdges->data.ptr + nmsEdges->step*i))[j])< minContrast)
				{
					
					(nmsEdges->data.ptr + nmsEdges->step*i)[j]=0;
					flag=0; // remove from edge
					////((uchar*)(imgGDir->imageData + imgGDir->widthStep*i))[j]=0;
				}
				else
				{   // if any of 8 neighboring pixel is not greater than max contraxt remove from edge
					if( (((double)((nmsEdges->data.ptr + nmsEdges->step*(i-1)))[j-1]) < maxContrast)	&&
						(((double)((nmsEdges->data.ptr + nmsEdges->step*(i-1)))[j]) < maxContrast)		&&
						(((double)((nmsEdges->data.ptr + nmsEdges->step*(i-1)))[j+1]) < maxContrast)	&&
						(((double)((nmsEdges->data.ptr + nmsEdges->step*i))[j-1]) < maxContrast)		&&
						(((double)((nmsEdges->data.ptr + nmsEdges->step*i))[j+1]) < maxContrast)		&&
						(((double)((nmsEdges->data.ptr + nmsEdges->step*(i+1)))[j-1]) < maxContrast)	&&
						(((double)((nmsEdges->data.ptr + nmsEdges->step*(i+1)))[j]) < maxContrast)		&&
						(((double)((nmsEdges->data.ptr + nmsEdges->step*(i+1)))[j+1]) < maxContrast)	)
					{
						(nmsEdges->data.ptr + nmsEdges->step*i)[j]=0;
						flag=0;
						////((uchar*)(imgGDir->imageData + imgGDir->widthStep*i))[j]=0;
					}
				}
				
			}
			
			// save selected edge information
			curX=i;	curY=j;
			if(flag!=0)
			{
				if(fdx!=0 || fdy!=0)
				{		
					RSum=RSum+curX;	CSum=CSum+curY; // Row sum and column sum for center of gravity
					
					cordinates[noOfCordinates].x = curX;
					cordinates[noOfCordinates].y = curY;
					edgeDerivativeX[noOfCordinates] = fdx;
					edgeDerivativeY[noOfCordinates] = fdy;
					
					//handle divide by zero
					if(MagG!=0)
						edgeMagnitude[noOfCordinates] = 1/MagG;  // gradient magnitude 
					else
						edgeMagnitude[noOfCordinates] = 0;
															
					noOfCordinates++;
				}
			}
		}
	}

	centerOfGravity.x = RSum /noOfCordinates; // center of gravity
	centerOfGravity.y = CSum/noOfCordinates ;	// center of gravity
		
	// change coordinates to reflect center of gravity
	for(int m=0;m<noOfCordinates ;m++)
	{
		int temp;

		temp=cordinates[m].x;
		cordinates[m].x=temp-centerOfGravity.x;
		temp=cordinates[m].y;
		cordinates[m].y =temp-centerOfGravity.y;
	}
	
	////cvSaveImage("Edges.bmp",imgGDir);
	
	// free alocated memories
	delete[] orients;
	////cvReleaseImage(&imgGDir);
	cvReleaseMat( &gx );
    cvReleaseMat( &gy );
	cvReleaseMat(&nmsEdges);

	ReleaseDoubleMatrix(magMat ,Ssize.height);
	
	modelDefined=true;
	return 1;
}


double GeoMatch::FindGeoMatchModel(const void* srcarr,double minScore,double greediness,CvPoint *resultPoint)
{
	CvMat *Sdx = 0, *Sdy = 0;
	
	double resultScore=0;
	double partialSum=0;
	double sumOfCoords=0;
	double partialScore;
	const short* _Sdx;
	const short* _Sdy;
	int i,j,m ;			// count variables
	double iTx,iTy,iSx,iSy;
	double gradMag;    
	int curX,curY;

	double **matGradMag;  //Gradient magnitude matrix
	
	CvMat srcstub, *src = (CvMat*)srcarr;
	src = cvGetMat( src, &srcstub );
	if(CV_MAT_TYPE( src->type ) != CV_8UC1 || !modelDefined)
	{
		return 0;
	}

	// source image size
	CvSize Ssize;
	Ssize.width =  src->width;
	Ssize.height= src->height;
	
	CreateDoubleMatrix(matGradMag ,Ssize); // create image to save gradient magnitude  values
		
	Sdx = cvCreateMat( Ssize.height, Ssize.width, CV_16SC1 ); // X derivatives
	Sdy = cvCreateMat( Ssize.height, Ssize.width, CV_16SC1 ); // y derivatives
	
	cvSobel( src, Sdx, 1, 0, 3 );  // find X derivatives
	cvSobel( src, Sdy, 0, 1, 3 ); // find Y derivatives
		
	// stoping criterias to search for model
	double normMinScore = minScore /noOfCordinates; // precompute minumum score 
	double normGreediness = ((1- greediness * minScore)/(1-greediness)) /noOfCordinates; // precompute greedniness 
		
	for( i = 0; i < Ssize.height; i++ )
    {
		 _Sdx = (short*)(Sdx->data.ptr + Sdx->step*(i));
		 _Sdy = (short*)(Sdy->data.ptr + Sdy->step*(i));
		
		 for( j = 0; j < Ssize.width; j++ )
		{
				iSx=_Sdx[j];  // X derivative of Source image
				iSy=_Sdy[j];  // Y derivative of Source image

				gradMag=sqrt((iSx*iSx)+(iSy*iSy)); //Magnitude = Sqrt(dx^2 +dy^2)
							
				if(gradMag!=0) // hande divide by zero
					matGradMag[i][j]=1/gradMag;   // 1/Sqrt(dx^2 +dy^2)
				else
					matGradMag[i][j]=0;
				
		}
	}
	for( i = 0; i < Ssize.height; i++ )
    {
			for( j = 0; j < Ssize.width; j++ )
             { 
				 partialSum = 0; // initilize partialSum measure
				 for(m=0;m<noOfCordinates;m++)
				 {
					 curX	= i + cordinates[m].x ;	// template X coordinate
					 curY	= j + cordinates[m].y ; // template Y coordinate
					 iTx	= edgeDerivativeX[m];	// template X derivative
					 iTy	= edgeDerivativeY[m];    // template Y derivative

					 if(curX<0 ||curY<0||curX>Ssize.height-1 ||curY>Ssize.width-1)
						 continue;
					 
					 _Sdx = (short*)(Sdx->data.ptr + Sdx->step*(curX));
					 _Sdy = (short*)(Sdy->data.ptr + Sdy->step*(curX));
						
					 iSx=_Sdx[curY]; // get curresponding  X derivative from source image
					 iSy=_Sdy[curY];// get curresponding  Y derivative from source image
						
					if((iSx!=0 || iSy!=0) && (iTx!=0 || iTy!=0))
					 {
						 //partial Sum  = Sum of(((Source X derivative* Template X drivative) + Source Y derivative * Template Y derivative)) / Edge magnitude of(Template)* edge magnitude of(Source))
						 partialSum = partialSum + ((iSx*iTx)+(iSy*iTy))*(edgeMagnitude[m] * matGradMag[curX][curY]);
									
					 }

					sumOfCoords = m + 1;
					partialScore = partialSum /sumOfCoords ;
					// check termination criteria
					// if partial score score is less than the score than needed to make the required score at that position
					// break serching at that coordinate.
					if( partialScore < (MIN((minScore -1) + normGreediness*sumOfCoords,normMinScore*  sumOfCoords)))
						break;
									
				}
				if(partialScore > resultScore)
				{
					resultScore = partialScore; //  Match score
					resultPoint->x = i;			// result coordinate X		
					resultPoint->y = j;			// result coordinate Y
				}
			} 
		}
	
	// free used resources and return score
	ReleaseDoubleMatrix(matGradMag ,Ssize.height);
	cvReleaseMat( &Sdx );
	cvReleaseMat( &Sdy );
	
	return resultScore;
}
// destructor
GeoMatch::~GeoMatch(void)
{
	delete[] cordinates ;
	delete[] edgeMagnitude;
	delete[] edgeDerivativeX;
	delete[] edgeDerivativeY;
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
void GeoMatch::DrawContours(IplImage* source,CvPoint COG,CvScalar color,int lineWidth)
{
	CvPoint point;
	point.y=COG.x;
	point.x=COG.y;
	for(int i=0; i<noOfCordinates; i++)
	{	
		point.y=cordinates[i].x + COG.x;
		point.x=cordinates[i].y + COG.y;
		cvLine(source,point,point,color,lineWidth);
	}
}

// draw contour at template image
void GeoMatch::DrawContours(IplImage* source,CvScalar color,int lineWidth)
{
	CvPoint point;
	for(int i=0; i<noOfCordinates; i++)
	{
		point.y=cordinates[i].x + centerOfGravity.x;
		point.x=cordinates[i].y + centerOfGravity.y;
		cvLine(source,point,point,color,lineWidth);
	}
}

