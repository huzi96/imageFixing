#include "header.3.0.h"


void ConfidenceTest(Confidence & c)
{
    c.showImg();
    
    int x = 16;
    int y = 28;
    int size = 485;
    
    double tt = c.getAverConfidence(x,y,size);
    
    double res = 0;
    for(int i = 0; i < size; i++)
        for(int j = 0; j < size; j++)
            res += c[i+x][j+y];
    res /= size*size;
    
    cout << "test 'getAverConfidence' : " << (res == tt) << endl;
    
    return;
}


void Confidence::showImg() const
{
    Mat show(_row,_col,0,cv::Scalar(0));
    
    cv::Mat_<uchar>::iterator it = show.begin<uchar>();
    
    for(int i = 0; i < _row * _col; i++,it++)
    {
        *it = 255 * (*(_val+i));
    }
    
    //cv::imshow("Confidence",show);
    return;
}

int Confidence::row() const
{
    return _row;
}

int Confidence::col() const
{
    return _col;
}

Confidence::Confidence(const cv::Mat & mask)
{
    _row = mask.rows;
    _col = mask.cols;
    
    _val = new double[_row * _col];
    memset(_val,0,sizeof(0));
    
    cv::Mat_<uchar>::const_iterator it = mask.begin<uchar>();
    cv::Mat_<uchar>::const_iterator itEnd = mask.end<uchar>();
    
    double * ptrVal = _val;
    
    for(;it != itEnd; it++, ptrVal++)
    {
        if(*it == 255) //被选中
            *ptrVal = 0;
        else
            *ptrVal = 1;
    }
}

double * Confidence::operator[](int r) const
{
    return _val + r*_col;
}

Confidence::~Confidence() { delete [] _val; }

double Confidence::getAverConfidence(int x,int y,int size) const
{
    double res = 0;
    for(int i = 0; i < size; i++)
    {
        double * ptr = _val + (x+i)*_col + y;
        for(int j = 0; j < size; j++, ptr++)
            res += *ptr;
    }
    res /= (size*size);
    return res;
}