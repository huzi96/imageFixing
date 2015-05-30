#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/gpu/gpu.hpp>
#include <iostream>
#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <fstream>
#include <algorithm>
#include <string>
#include <vector>
#include <map>
#include <ctime>
#include <set>
#include <queue>

using namespace std;
using namespace cv;

class Patch;
class Confidence;
class BondPoint;

extern Mat _src;
extern Mat * _mask;
extern Confidence * _c;

extern Mat * bondary;

double calculateRoundP(Mat &mask, Mat &src,const BondPoint &p);


class Confidence //信任度
{
public:
    Confidence(const cv::Mat & mask);
    ~Confidence();
    
    void showImg() const;
    inline int row() const;
    inline int col() const;
    double * operator[](int) const;
    
    double getAverConfidence(int,int,int) const;
private:
    //Confidence(const Confidence &){} //禁止公有调用复制构造函数
    
    int _row;
    int _col;
    double * _val;
};

class BondPoint
{
public:
    int _x;
    int _y;
    
    set<Patch>::iterator ptr; //指向其在按优先级排列的set中的位置
    
    BondPoint(int x,int y):_x(x),_y(y){}
    
    bool operator < (const BondPoint & p) const
    {
        if(_x == p._x)
            return _y < p._y;
        else
            return _x < p._x;
    }
    
};

class Patch
{
public:
    Patch(const BondPoint & point):_x(point._x),_y(point._y)
    {
        double d = calculateRoundP(*_mask, _src, point); //计算沿等照度线的强度函数
        _infoEdge = std::exp(d*d - 1);
        
        _averConfidence = _c->getAverConfidence(xZero(),yZero(),_size);
        
        _priorVal = 0.4 * _averConfidence + 0.6 * _infoEdge;
    }
    //是否优化，则不传递参数c，而是传值
    
    bool operator < (const Patch & p)const
    {
        return _priorVal > p._priorVal;
    }
    
    inline int xZero() { return _x - _size/2; }
    inline int yZero() { return _y - _size/2; }
    inline int size() { return _size; }
    inline double infoEdge() { return _infoEdge; }
    
    
private:
    int _x; //_x,_y表示中心点坐标
    int _y;
    static const int _size = 9; //【NOTICE】:Patch出界怎么办！！！！
    
    //int haveKnown; 已经知道的块数
    
    double _priorVal; //优先值 P(p) = 0.4*C(p) + 0.6*E(p)
    double _infoEdge;  // E(p) = e^(d^2 - 1) 强边缘信息 取值为0~1
    double _averConfidence; //均信任值C(p) 取值为0~1
};


void renewBond(Patch & prior, set<BondPoint> & points, set<Patch> & priorLevel, Mat & mask, Mat & boundary,Mat &newBondary);

void Search(Patch & prior, Mat & src, Mat & mask, Confidence & c);

void ConfidenceTest(Confidence & c); //测试用


void chooseArea(Mat * mask, Mat & src);
void initBorder(set<BondPoint> & points);
void initPriorLevel(set<BondPoint> & points, set<Patch> & PriorLevel);
void reFreshBond(Mat &newBondary);