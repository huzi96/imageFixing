#include "header.3.0.h"



void getGradiant(Mat &src, Mat &grad)
{
    Mat grad_x, grad_y,src_gray;
    Mat abs_grad_x, abs_grad_y;
    cv::cvtColor(src, src_gray, CV_RGB2GRAY);
    
    /// Gradient X
    //Scharr( src_gray, grad_x, ddepth, 1, 0, scale, delta, BORDER_DEFAULT );
    cv::Sobel(src, grad_x, src.depth(), 1, 0, 1, 1, 0, BORDER_DEFAULT);
    cv::convertScaleAbs(grad_x, abs_grad_x);
    
    /// Gradient Y
    //Scharr( src_gray, grad_y, ddepth, 0, 1, scale, delta, BORDER_DEFAULT );
    cv::Sobel(src, grad_y, src.depth(), 0, 1, 1, 1, 0, BORDER_DEFAULT);
    cv::convertScaleAbs(grad_y, abs_grad_y);
    
    /// Total Gradient (approximate)
    cv::addWeighted(abs_grad_x, 0.5, abs_grad_y, 0.5, 0, grad);
}

void renewBond(Patch & prior, set<BondPoint> & points, set<pPatch> & priorLevel, Mat & mask, Mat & boundary,Mat &newBondary)
{
    int xZero = prior.xZero()-1;
    int yZero = prior.yZero()-1;
    int size = prior.size();
    
    //注意size的含义！！！
    Mat newROI = newBondary(Rect(yZero, xZero, size+2, size+2));
    Mat bondROI = boundary(Rect(yZero, xZero, size+2, size+2));
    
    int row = newROI.rows;
    int col = newROI.cols;
    
    set<BondPoint>::iterator itPoint;
    
    /*
    namedWindow("old");
    imshow("old",newROI);
    waitKey(0);
    imshow("old", bondROI);
    waitKey(0);
     */
    for (int i = 0; i < row; ++i)
    {
        uchar * ptrNew = newROI.ptr<uchar>(i);
        uchar * ptrBound = bondROI.ptr<uchar>(i);
        
        
        for (int j = 0; j < col; ++j, ++ptrNew, ++ptrBound)
        {
            if (*ptrBound == 255) //旧边界
            {
                
                set<BondPoint>::iterator goal = points.find(BondPoint(xZero + i, yZero + j));
                
                pPatch ruin = goal->ptr;
                set<pPatch>::iterator ruinIT=priorLevel.find(ruin);
                priorLevel.erase(ruinIT);
                points.erase(goal);
                //cout << "delete " << xZero + i << ' ' << yZero + j << endl;
            }
        }
    }
    for (int i = 0; i < row; ++i)
    {
        uchar * ptrNew = newROI.ptr<uchar>(i);
        uchar * ptrBound = bondROI.ptr<uchar>(i);
        
        
        for (int j = 0; j < col; ++j, ++ptrNew, ++ptrBound)
        {
            if (*ptrNew == 255) //更新边界
            {
                BondPoint crt(xZero+i,yZero+j);
                crt.ptr=new Patch(crt);
                priorLevel.insert(crt.ptr);
                points.insert(crt);
                
                //cout << "add " << xZero + i << ' ' << yZero + j << endl;
            }
        }
    }
    
    
    //如果该方格的边界上的点是mask中选择的区域，那么这个方格的边界就变为边界点。并将方格内部的原有边界点删除
    //那么要同时更新mask和boundary图
}





double getSimilar(Patch & prior, int x, int y, Mat & mask, Mat & src)
{
    int size = prior.size();
    int pointCNT = 0;
    Point anchor(x, y);
    double sigmaOfPixel = 0;
    //Mat patchROI(mask, Rect(prior.xZero(), prior.yZero(), prior.size(), prior.size()));
    
    
    for (int i = 0; i < size; i++)
    {
        for (int j = 0; j < size; j++)
        {
            //
            Point patchAnchor(prior.xZero()+i, prior.yZero()+j);
            Point p(x + i, y + j);
            if (mask.at<uchar>(p)==255)
            {
                return -100000;
            }
            if (mask.at<uchar>(patchAnchor)!=255)
            {
                pointCNT++;
                for (int k = 0; k < 3; k++)
                {
                    sigmaOfPixel += ((src.at<Vec3b>(anchor)[k] - src.at<Vec3b>(patchAnchor)[k])
                                     *(src.at<Vec3b>(anchor)[k] - src.at<Vec3b>(patchAnchor)[k]));
                }
            }
        }
    }
    sigmaOfPixel /= 1000000;
    //cout << sigmaOfPixel << endl;
    //没有算方差哟
    return sigmaOfPixel;
}

void Search(Patch & prior, Mat & src, Mat & mask, Confidence & c)
{
    int chDist = 100 * prior.infoEdge() + 7;
    
    int size = prior.size(); //匹配块的大小
    
    //搜索范围(记录将匹配框的左上角点范围)
    int xZeroROI = prior.xZero() - chDist - size / 2;
    if (xZeroROI < 0) xZeroROI = 0; //之后要再取5*5的方格
    int yZeroROI = prior.yZero() - chDist - size / 2;
    if (yZeroROI < 0) yZeroROI = 0;
    
    int rowROI = (xZeroROI + chDist * 2 + 1 + size) < src.rows ? chDist * 2 + 1 : src.rows - size - xZeroROI; //搜索尺寸
    int colROI = (yZeroROI + chDist * 2 + 1 + size) < src.cols ? chDist * 2 + 1 : src.cols - size - yZeroROI;
    
    
    double dBest = 0;
    int xBest, yBest;
    
    for (int i = xZeroROI; i < xZeroROI + rowROI; i++) //枚举左上角
        for (int j = yZeroROI; j < yZeroROI + colROI; j++)
        {
            double d = getSimilar(prior, i, j, mask, src);
            double tmp;
            tmp = exp(-d*d);
            //cout << "d " << d << endl;;
            if (tmp > dBest)
            {
                dBest = d;
                xBest = i, yBest = j;//记录左上角点;
            }
        }
    cout<<"dbest "<<dBest<<endl;
    
    double newConfidence = c.getAverConfidence(xBest, yBest, size) * std::exp(-dBest*dBest);
    
    cout << "newConficence" << newConfidence << endl;
    //填充该点，并更新置信度
    for (int i = 0; i < size; i++)
    {
        for (int j = 0; j < size; j++)
        {
            if (mask.at<uchar>(prior.xZero()+i,prior.yZero()+j)==255)
            {
                for (int k = 0; k < 3; k++)
                {
                
                    mask.at<uchar>(prior.xZero() + i, prior.yZero() + j) = 0;
                    src.at<cv::Vec3b>(prior.xZero() + i, prior.yZero() + j)[k] = src.at<cv::Vec3b>(xBest + i, yBest + j)[k];
                }
               
            }
            
            c[i + prior.xZero()][j + prior.yZero()] = newConfidence;
        }
    }
}




const double sqrt2 = cv::sqrt(2.0);

const int kernelSize = 3;

double exert(double Operator[3][3], Mat &gray_image)
{
    double tmp = 0;
    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            tmp += Operator[i][j] * gray_image.at<uchar>(i, j);
        }
    }
    tmp /= (4 + 2 * sqrt2);
    return tmp;
}

double calculateRoundP(Mat &mask, Mat &src, const BondPoint &p)
{
    //cout << "start" << endl;
    double operatorX[3][3] = {
        {-1,0,1},
        {-sqrt2,0,sqrt2},
        {-1,0,1}
    };
    double operatorY[3][3] = {
        { -1, sqrt2, -1 },
        { 0, 0, 0},
        { 1, sqrt2,1 }
    };
    Mat gray_image;
    //这里不判断算子是否出界
    //Point anchor(p._x - 1, p._y - 1);
    int x = p._x - 1;
    int y = p._y - 1;
    Mat srcROI(src, Rect(y, x, 3, 3));//ROI小心大坑，坐标
    Mat maskROI(mask, Rect(y, x, 3, 3));
    
    //namedWindow("testm");
    //imshow("testm", maskROI);
    //waitKey(0);
    cvtColor(srcROI, gray_image, CV_BGR2GRAY);
    uchar center = gray_image.at<uchar>(0, 0);
    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            if (maskROI.at<uchar>(i,j)==255)
            {
                gray_image.at<uchar>(i, j) = center;
            }
        }
    }
    double dX, dY, nX, nY;
    dX = exert(operatorX, gray_image);
    dY = exert(operatorY, gray_image);
    nX = exert(operatorX, maskROI);
    nY = exert(operatorY, maskROI);
    //cout << gray_image << endl;
    //cout << maskROI << endl;
    //cout << dX << ' ' << dY << ' ' << nX << ' ' << nY << endl;
    double res = (dX*nY - dY*nX) / sqrt((dX*dX + dY*dY)*(nX*nX + nY*nY));
    return res;
}
