#include "header.3.0.h"

//全局变量均用 _名字，在函数中使用时进行引用

Mat _src;
//src(source)为原图
Mat * _mask;
//mask为单通道矩阵，记录修改区域，255表示需要区域，0为不需修改区域
Confidence * _c;

int main()
{
    Mat & src = _src;
    src = imread("test7.jpg");//读入目标图像
    if(!_src.data) //读取文件失败
    {
        cout << "loading error." << endl;
        system("pause");
        return 1;
    }
    //以上需要封装
    Mat * & mask = _mask;
    mask = new Mat(src.rows, src.cols, CV_8UC1, Scalar(0));//建立一个原图大小的单通道图像矩阵
    //CV_8UC1表示8-bit无符号单通道矩阵
    
    chooseArea(mask,src); //在src上选中相应区域，输出至mask，255表示选择区域，0为未选择区域
    
    ///set<BondPoint> * points = new set<BondPoint>;
    set<BondPoint> points;
    initBorder(points); //将边界点插入points（按x,y排序）
    
    //Confidence * & c = _c;
    _c = new Confidence(*mask);
    ConfidenceTest(*_c);
    
    
    set<pPatch> priorLevel; //boundary points
    initPriorLevel(points,priorLevel); //计算边界点周围数据，导入set
    
    int n = 30;
    
    namedWindow("test");
    while(!points.empty()) //仍有未填充的点
    {
        cout << "size " << points.size() << endl;;
        //break;
        
        if(priorLevel.empty())
            break;
        
        pPatch prior = *priorLevel.begin();
        Patch tmp = *prior;
        Search(tmp,src,*mask,*_c); //查找最优点匹配，并覆盖
        Mat newBondary;
        reFreshBond(newBondary);
        
        imshow("test", src);
        waitKey(0);
        renewBond(tmp,points,priorLevel,*mask,*bondary,newBondary); //更新边界
        newBondary.copyTo(*bondary);
    }
    
    imshow("result",_src);
    waitKey(0);
    
    delete points;
    delete priorLevel;
    delete _mask;
    
    return 0;
}