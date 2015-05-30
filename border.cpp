#include "header.3.0.h"

//border.cpp
//功能：对选中区域边界处理

Mat *bondary;
//将mask区域的边界存入points中
void reFreshBond(Mat &newBondary)
{
    Mat & mask = *_mask;
    Mat showRes(mask.rows, mask.cols, mask.type(), cv::Scalar(0)); //showRes
    
    int row = mask.rows;
    int col = mask.cols;
    
    for (int i = 0; i < row; ++i)
    {
        for (int j = 0; j < col; ++j)
        {
            if (mask.at<uchar>(i, j) == 0)
                continue;
            
            bool isEdge = false;
            
            if (i - 1 >= 0 && mask.at<uchar>(i - 1, j) == 0)
                isEdge = true;
            else if (i + 1 < row && mask.at<uchar>(i + 1, j) == 0)
                isEdge = true;
            else if (j - 1 >= 0 && mask.at<uchar>(i, j - 1) == 0)
                isEdge = true;
            else if (j + 1 < col && mask.at<uchar>(i, j + 1) == 0)
                isEdge = true;
            
            if (isEdge)
            {
                showRes.at<uchar>(i, j) = 255; //showRes
            }
            
        }
    }
    showRes.copyTo(newBondary);
}
void initBorder(set<BondPoint> & points)
{
    Mat & mask = *_mask;
    
    points.clear();
    
    Mat showRes(mask.rows,mask.cols,mask.type(),cv::Scalar(0)); //showRes
    
    int row = mask.rows;
    int col = mask.cols;
    
    for(int i = 0; i < row; ++i)
    {
        for(int j = 0; j < col; ++j)
        {
            if(mask.at<uchar>(i,j) == 0)
                continue;
            
            bool isEdge = false;
            
            if(i - 1 >= 0 && mask.at<uchar>(i-1,j) == 0)
                isEdge = true;
            else if(i + 1 < row && mask.at<uchar>(i+1,j) == 0)
                isEdge = true;
            else if(j - 1 >= 0 && mask.at<uchar>(i,j-1) == 0)
                isEdge = true;
            else if(j + 1 < col && mask.at<uchar>(i,j+1) == 0)
                isEdge = true;
            
            if(isEdge)
            {
                showRes.at<uchar>(i,j)=255; //showRes
                //cout << "isedge " << i << ' ' << j << endl;
                points.insert(BondPoint(i,j));
            }
            
        }
    }
    bondary = new Mat;
    showRes.copyTo(*bondary);
    imshow("boundary",*bondary);
    waitKey(0);
}

void initPriorLevel(set<BondPoint> & points, set<pPatch> & PriorLevel)
{
    Mat & src = _src;
    Confidence & c = *_c;
    
    if(points.empty())
        return;
    
    set<BondPoint>::iterator itPoint = points.begin();
    set<BondPoint>::iterator itPointEnd = points.end();
    
    for(;itPoint != itPointEnd; ++itPoint)
    {
        pPatch & ptr=(pPatch &)(itPoint->ptr);
        pPatch crt=new Patch(*itPoint);
        PriorLevel.insert(crt);
        ptr=crt;
    }
}