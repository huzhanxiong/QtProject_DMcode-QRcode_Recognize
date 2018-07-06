#include "dm_recognize.h"
#include <QApplication>
#include <QDebug>

#include <opencv2/opencv.hpp>
#include <QZXing.h>


using namespace cv;
using namespace std;



class DM_message
{
public:
    //DM_message();
    ~DM_message();

public:
    Point2f point_center;
    Point2f points[4];
    uint8_t tabPoint_num = 0;
    float angle;
    Mat code_image;
    string code_string;
};


DM_message::~DM_message()
{

}



class QR_message
{
public:
    //QR_message();
    ~QR_message();

public:
    Point2f point_center;
    Point2f points[4];
    uint8_t tabPoint_num = 0;
    float angle;
    Mat code_image;
    string code_string;
};


QR_message::~QR_message()
{

}


vector<DM_message> DM_find(Mat &DM_image, vector<Point2f> points_change);
vector<QR_message> QR_find(Mat &img_QRcode, vector<Point2f> points_changex);
QImage Mat2QImage(Mat cvImg);
QImage MatToQImage(const cv::Mat& mat);
bool calcSafeRect(const RotatedRect& roi_rect, const Mat src, Rect_<float>& safeBoundRect);


int main(int argc, char *argv[])
{
    //QApplication a(argc, argv);
    //DM_recognize w;
    //w.show();

    vector<Point2f> points_change;	//标准marker坐标四个点
    Size markerSize(200, 200);	//标准marker大小
    points_change.push_back(Point2f(0, 0));
    points_change.push_back(Point2f(markerSize.width-1, 0));
    points_change.push_back(Point2f(markerSize.width-1, markerSize.height-1));
    points_change.push_back(Point2f(0, markerSize.height-1));

    vector<Point2f> points_changex;	//标准marker坐标四个点
    Size markerSizex(100, 100);	//标准marker大小
    points_changex.push_back(Point2f(0, 0));
    points_changex.push_back(Point2f(markerSizex.width-1, 0));
    points_changex.push_back(Point2f(markerSizex.width-1, markerSizex.height-1));
    points_changex.push_back(Point2f(0, markerSizex.height-1));


    Mat DM_image = imread("../DM_recognize/img_DMcode/test3.png");
    if(0)
    if(!DM_image.empty())
    {
        vector<DM_message> DM_message_all;

        DM_message_all = DM_find(DM_image, points_change);
        qDebug() << "DM_message_all's size:" <<DM_message_all.size();

        if(DM_message_all.size())
        {
            vector<DM_message>::iterator itc = DM_message_all.begin();

            int name_num = 0;
            while(itc != DM_message_all.end())
            {
                DM_message dm_message = *itc;
                itc++;
                name_num++;

                qDebug() << "message:" << dm_message.code_string.c_str() << " rotate angle:" << dm_message.angle
                         << " center point:" << dm_message.point_center.x << "," << dm_message.point_center.y;

                for(int i=0; i<4; i++)
                    line(DM_image, dm_message.points[i], dm_message.points[(i+1)%4], Scalar(255,0,0), 2, LINE_8);
                circle(DM_image, dm_message.points[dm_message.tabPoint_num], 5, Scalar(0,255,0), -1, LINE_8);
                circle(DM_image, dm_message.point_center, 3, Scalar(0,0,255), 2, LINE_8);

                /*string name = "pic_";
                name += to_string(name_num);
                imshow(name, dm_message.code_image);*/
            }
        }
        imshow("DM_image", DM_image);
    }


    Mat QR_image = imread("../DM_recognize/img_QRcode/codex.png");
    if(1)
    if(!QR_image.empty())
    {
        vector<QR_message> QR_message_all;

        QR_message_all = QR_find(QR_image, points_changex);
        qDebug() << "QR_message_all's size:" << QR_message_all.size();

        if(QR_message_all.size())
        {
            vector<QR_message>::iterator itc = QR_message_all.begin();

            int name_num = 0;
            while(itc != QR_message_all.end())
            {
                QR_message qr_message = *itc;
                itc++;
                name_num++;

                qDebug() << "message:" << qr_message.code_string.c_str() << " rotate angle:" << qr_message.angle
                         << " center point:" << qr_message.point_center.x << "," << qr_message.point_center.y;

                for(int i=0; i<4; i++)
                    line(QR_image, qr_message.points[i], qr_message.points[(i+1)%4], Scalar(255,0,0), 2, LINE_8);
                circle(QR_image, qr_message.points[qr_message.tabPoint_num], 5, Scalar(0,255,0), -1, LINE_8);
                circle(QR_image, qr_message.point_center, 3, Scalar(0,0,255), 2, LINE_8);

                /*string name = "pic_";
                name += to_string(name_num);
                imshow(name, dm_message.code_image);*/
            }
        }
        imshow("QR_image", QR_image);
    }


    while(waitKey(0) != 49)
        break;
    //return a.exec();
}



vector<DM_message> DM_find(Mat &DM_image, vector<Point2f> points_change)
{
    vector<DM_message> DM_allFind;

    Mat DM_gray;
    Mat DM_threshold;
    Mat DM_imagex = DM_image.clone();

    cvtColor(DM_image, DM_gray, CV_BGRA2GRAY);
    blur(DM_gray, DM_gray, Size(1,1));	//均值滤波 / 高斯滤波
    //Laplacian(grayimg, grayimg, CV_8U, 3, 1, 0, BORDER_CONSTANT);	//拉普拉斯算子滤波
    //imshow("DM_gray", DM_gray);

    //adaptiveThreshold(grayimg, thresholdimg, 255, ADAPTIVE_THRESH_GAUSSIAN_C, THRESH_BINARY_INV, 7, 7);
    threshold(DM_gray, DM_threshold, 0, 255, THRESH_BINARY | THRESH_OTSU);
    //imshow("DM_threshold", DM_threshold);

    vector< vector<Point> > contours;
    findContours(DM_threshold,
               contours,               // a vector of contours
               CV_RETR_LIST,
               CV_CHAIN_APPROX_NONE);  // all pixels of each contours

    //qDebug() << contours.size();
    /*Mat DM_image_x = DM_image.clone(); //= Mat::zeros(DM_image.size(), CV_8U);
    DM_image_x.setTo(255);
    drawContours(DM_image_x, contours, -1, Scalar(0), 3, LINE_8);
    imshow("DM_image_x", DM_image_x);*/


    vector< vector<Point> >::iterator itc = contours.begin();
    vector<RotatedRect> second_rects;
    while (itc != contours.end())
    {
        RotatedRect mr = minAreaRect(Mat(*itc));
        ++itc;

        Point2f approxCurve[4];
        mr.points(approxCurve);

        //判断各顶点距离是否足够大
        float minDist = 1e10;
        for(int i=0; i<4; i++)
        {
            //求当前四边形各顶点之间最短距离
            Point vec = approxCurve[i] - approxCurve[(i+1)%4];
            float squaredDistance = vec.dot(vec);
            squaredDistance = sqrt(squaredDistance);
            minDist = min(minDist, squaredDistance);
        }
        //当四边形大小合适，则将该四边形maker放入possibleMarkers容器内
        int m_length = 33;
        float w = mr.size.width;
        float h = mr.size.height;
        float rate = min(w, h) / max(w, h);
        if(rate > 0.85 && w > m_length && h > m_length && mr.boundingRect().area() < DM_imagex.cols*DM_imagex.rows)
        {
            second_rects.push_back(mr);
        }
    }
    //qDebug() << second_rects.size();

    /*vector<RotatedRect>::iterator itc_R = second_rects.begin();
    while(itc_R != second_rects.end())
    {
        RotatedRect rotateR = *itc_R;
        Point2f points[4];
        rotateR.points(points);
        for(int i=0; i<4; i++)
            line(DM_image, points[i], points[(i+1)%4], Scalar(255,0,0), 2, LINE_8);
        itc_R++;
    }*/

    vector<RotatedRect>::iterator itcx = second_rects.begin();
    int image_num = 0;

    while(itcx != second_rects.end())
    {
       RotatedRect mr = *itcx;
        itcx++;
        //qDebug() << mr.angle;
        Point2f points[4];
        mr.points(points);
        //for(int i=0; i<4; i++)
            //line(DM_image, points[i], points[(i+1)%4], Scalar(255,0,0), 2, LINE_8);

        int8_t DM_top = -1, DM_button = -1, DM_left = -1, DM_right = -1;
        uint8_t deflection_num = 7;

        Mat imgx;
        Rect_<float> safeBoundRect;
        calcSafeRect(mr, DM_threshold, safeBoundRect);
        Mat dm_imgx = DM_threshold(safeBoundRect);

        vector<Point2f> points_start;
        for(int i=0; i<4; i++)
            points_start.push_back(points[i]);

        image_num++;
        string pic_name;
        pic_name = "img_" + to_string(image_num);

        Mat M = getPerspectiveTransform(points_start, points_change);	//透视变化矩阵获取
        warpPerspective(DM_gray, imgx, M, Size(200, 200));	//得到矩形标记

        //blur(imgx, imgx, Size(1,1));
        threshold(imgx, imgx, 0, 255, THRESH_BINARY | THRESH_OTSU);
        //定义核
        //Mat element = getStructuringElement(MORPH_RECT, Size(5, 5));
        //进行形态学操作
        //morphologyEx(imgx,imgx, MORPH_OPEN, element);
        //imshow(pic_name, imgx);

        //qDebug() << imgx.cols << " " << imgx.rows << " " << imgx.channels();
        //string img_data;
        /*for(int i=0; i<imgx.rows; i++)
        {
            string img_data;
            for(int j=0; j<imgx.cols; j++)
                img_data += to_string(imgx.data[i*imgx.cols+j]) + " ";
            qDebug() << img_data.c_str();
        }*/


        Mat cell_top = imgx(Rect(3, 3, imgx.cols-7, 2));
        int nZ_top = countNonZero(cell_top);
        //qDebug() << nZ_top << " " << abs(nZ_top - cell_top.cols*cell_top.rows*cell_top.channels()/2) <<
                    //" " << imgx.cols/deflection_num;
        if(nZ_top < imgx.cols/deflection_num)
            DM_top = 1;
        else if(abs(nZ_top - cell_top.cols*cell_top.rows*cell_top.channels()/2) < imgx.cols/deflection_num)
            DM_top = 0;
        //qDebug() << DM_top;

        Mat cell_button = imgx(Rect(5, imgx.rows-7, imgx.cols-6, 2));
        int nZ_button = countNonZero(cell_button);
        //qDebug() << nZ_button << " " << abs(nZ_button - cell_button.cols*cell_button.rows*cell_button.channels()/2) <<
                    //" " << imgx.cols/deflection_num;
        if(nZ_button < imgx.cols/deflection_num)
            DM_button = 1;
        else if(abs(nZ_button - cell_button.cols*cell_button.rows*cell_button.channels()/2) < imgx.cols/deflection_num)
            DM_button = 0;
        //qDebug() << DM_button;

        Mat cell_left = imgx(Rect(3, 3, 3, imgx.rows-6));
        int nZ_left = countNonZero(cell_left);
        //qDebug() << nZ_left;
        if(nZ_left < imgx.cols/deflection_num)
            DM_left = 1;
        else if(abs(nZ_left - cell_left.cols*cell_left.rows*cell_left.channels()/2) < imgx.cols/deflection_num)
            DM_left = 0;
        //qDebug() << DM_left;

        Mat cell_right = imgx(Rect(imgx.cols-6, 3, 2, imgx.rows-6));
        int nZ_right = countNonZero(cell_right);
        //qDebug() << nZ_right;
        if(nZ_right < imgx.cols/deflection_num)
            DM_right = 1;
        else if(abs(nZ_right - cell_right.cols*cell_right.rows*cell_right.channels()/2) < imgx.cols/deflection_num)
            DM_right = 0;
        //qDebug() << DM_right;
        //qDebug() << " ";

        if((DM_top == 0 && DM_button == 1 || DM_top == 1 && DM_button == 0) || (DM_left == 0 && DM_right == 1 || DM_left == 1 && DM_right == 0))
        {
            //qDebug() << DM_top << " " << DM_button << " " << DM_left << " " << DM_right;
            uint8_t tabPoint_num = 0;
            if(DM_top == 1 && DM_button == 0 && DM_left == 1 && DM_right == 0)
                tabPoint_num = 0;
            else if(DM_top ==1 && DM_button == 0 && DM_left == 0 && DM_right == 1)
                tabPoint_num = 1;
            else if(DM_top ==0 && DM_button == 1 && DM_left == 0 && DM_right == 1)
                tabPoint_num = 2;
            else if(DM_top ==0 && DM_button == 1 && DM_left == 1 && DM_right == 0)
                tabPoint_num = 3;
            //qDebug() << "rotate_num:" << tabPoint_num;

            /*Mat imgx = DM_image.clone();
            circle(imgx, points[0], 6, Scalar(0,0,255), -1, LINE_8);
            circle(imgx, points[1], 6, Scalar(0,255,0), -1, LINE_8);
            circle(imgx, points[2], 6, Scalar(255,0,0), -1, LINE_8);
            imshow("img", imgx);*/

            Point2f point_center;
            for (int j=0; j<4; j++)
            {
                point_center += points[j];
            }
            point_center.x =  point_center.x/4;
            point_center.y =  point_center.y/4;

            double k0 = -(points[(tabPoint_num+3)%4].x - points[tabPoint_num].x) / sqrt((points[(tabPoint_num+3)%4].x - points[tabPoint_num].x)*(points[(tabPoint_num+3)%4].x - points[tabPoint_num].x) + (points[(tabPoint_num+3)%4].y - points[tabPoint_num].y)*(points[(tabPoint_num+3)%4].y - points[tabPoint_num].y));
            double k1 = -(points[(tabPoint_num+2)%4].x - points[(tabPoint_num+1)%4].x) / sqrt((points[(tabPoint_num+2)%4].x - points[(tabPoint_num+1)%4].x)*(points[(tabPoint_num+2)%4].x - points[(tabPoint_num+1)%4].x) + (points[(tabPoint_num+2)%4].y - points[(tabPoint_num+1)%4].y)*(points[(tabPoint_num+2)%4].y - points[(tabPoint_num+1)%4].y));
            double k2 = (points[(tabPoint_num+1)%4].y - points[tabPoint_num].y) / sqrt((points[(tabPoint_num+1)%4].y - points[tabPoint_num].y)*(points[(tabPoint_num+1)%4].y - points[tabPoint_num].y) + (points[(tabPoint_num+1)%4].x - points[tabPoint_num].x)*(points[(tabPoint_num+1)%4].x - points[tabPoint_num].x));
            double k3 = (points[(tabPoint_num+2)%4].y - points[(tabPoint_num+3)%4].y) / sqrt((points[(tabPoint_num+2)%4].y - points[(tabPoint_num+3)%4].y)*(points[(tabPoint_num+2)%4].y - points[(tabPoint_num+3)%4].y) + (points[(tabPoint_num+2)%4].x - points[(tabPoint_num+3)%4].x)*(points[(tabPoint_num+2)%4].x - points[(tabPoint_num+3)%4].x));

            double k_h = (k0 + k1) / 2;
            double k_v = (k2 + k3) / 2;
            double angle1 = acos(k_h);
            double angle2 = acos(k_v);
            //qDebug() << angle1*180/3.14 << " " << angle2*180/3.14;

            float angle;
            if(points[tabPoint_num].y <= points[(tabPoint_num+3)%4].y)
                angle = 180 - (angle1+angle2)/2*180/3.14;
            else
                angle = 180 + (angle1+angle2)/2*180/3.14;

            //if(angle <= 180)    angle = 180 + angle;
            //else angle = 360 - angle;

            /** for(int i=0; i<4; i++)
                line(DM_image, points[i], points[(i+1)%4], Scalar(255,0,0), 2, LINE_8);
            circle(DM_image, points[tabPoint_num], 5, Scalar(0,255,0), -1, LINE_8);
            circle(DM_image, point_center, 3, Scalar(0,0,255), 2, LINE_8);
            qDebug() << angle;**/

            DM_message message_find;
            for(int i=0; i<=3; i++)
                message_find.points[i] = points[i];
            message_find.point_center = point_center;
            message_find.tabPoint_num = tabPoint_num;
            message_find.angle = angle;
            //message_find.code_image = imgx;
            //imshow(pic_name, dm_imgx);

            QImage qImg = MatToQImage(imgx);
            QZXing decoder;
            QString qrmsg = decoder.decodeImage(qImg);

            if(!qrmsg.isEmpty())
            {
                message_find.code_image = imgx;
                message_find.code_string = qrmsg.toStdString();
                DM_allFind.push_back(message_find);
            }
            else
            {
                QImage qImg = MatToQImage(dm_imgx);
                QZXing decoder;
                QString qrmsg = decoder.decodeImage(qImg);
                if(!qrmsg.isEmpty())
                {
                    message_find.code_image = dm_imgx;
                    message_find.code_string = qrmsg.toStdString();
                    DM_allFind.push_back(message_find);
                }
            }
        }
    }

    return DM_allFind;
}



vector<QR_message> QR_find(Mat &img_QRcode, vector<Point2f> points_changex)
{
    vector<QR_message> QR_allFind;

    Mat img_grag, img_binary;
    vector<RotatedRect> QR_rect;

    cvtColor(img_QRcode, img_grag, COLOR_BGR2GRAY);
    threshold(img_grag, img_binary, 0, 255, THRESH_BINARY | THRESH_OTSU);


    Mat img_binary_clone = img_binary.clone();
    //定义核
    Mat element = getStructuringElement(MORPH_RECT, Size(11, 11));
    //进行形态学操作
    morphologyEx(img_binary, img_binary_clone, MORPH_OPEN, element);
    //imshow("img_binary_clone", img_binary_clone);

    vector< vector<Point> > contours;
    findContours(img_binary_clone,
               contours,               // a vector of contours
               CV_RETR_LIST,
               CV_CHAIN_APPROX_NONE);  // all pixels of each contours

    for(size_t t=0; t<contours.size(); t++)
    {
        double area = contourArea(contours[t]);
        if(area < 800) continue;
        RotatedRect rect = minAreaRect(contours[t]);

        float w = rect.size.width;
        float h = rect.size.height;
        float rate = min(w, h) / max(w, h);
        if(rate > 0.85 && w > 33 && h > 33 && rect.boundingRect().area() < img_binary.cols*img_binary.rows)
        {
            /*Point2f points[4];
            rect.points(points);
            for(int i=0; i<4; i++)
                line(img_QRcode, points[i], points[(i+1)%4], Scalar(255,0,0), 2, LINE_8);*/
            QR_rect.push_back(rect);
        }
    }


    vector<RotatedRect>::iterator itcx = QR_rect.begin();
    Point2f true_points[4];
    while(itcx != QR_rect.end())
    {   
        RotatedRect qr_rect = *itcx;
        itcx++;


        if(1)
        {
            Mat find_point = img_binary_clone(qr_rect.boundingRect());
            vector< vector<Point> > contours;
            findContours(find_point,
                       contours,               // a vector of contours
                       CV_RETR_LIST,
                       CV_CHAIN_APPROX_NONE);  // all pixels of each contours

            for(size_t t=0; t<contours.size(); t++)
            {
                double area = contourArea(contours[t]);
                if(area < 800) continue;
                RotatedRect rect = minAreaRect(contours[t]);

                float w = rect.size.width;
                float h = rect.size.height;
                float rate = min(w, h) / max(w, h);
                if(rate > 0.85 && w > 33 && h > 33 && rect.boundingRect().area() == find_point.cols*find_point.rows)
                {
                    Point2f points[4];
                    rect.points(points);

                    for(int i=0; i<=3; i++)
                        true_points[i] = points[i];
                    break;
                }
            }
        }


        vector<RotatedRect> rect_small;

        Mat qr_imgx = img_binary(qr_rect.boundingRect());

        vector< vector<Point> > contours;
        vector<Vec4i> hireachy;
        findContours(qr_imgx, contours, hireachy, RETR_LIST, CHAIN_APPROX_SIMPLE, Point());

        /*Mat qr_imgy = img_QRcode(qr_rect.boundingRect());
        drawContours(qr_imgy, contours, -1, Scalar(0,255,0), 1, LINE_8);
        //image_num++;
        string pic_name;
        pic_name = "img_" + to_string(image_num);*/
        //imshow(pic_name, qr_imgy);

        for(size_t t=0; t<contours.size(); t++)
        {
            double area = contourArea(contours[t]);
            if(area < 100) continue;
            RotatedRect rect = minAreaRect(contours[t]);

            float w = rect.size.width;
            float h = rect.size.height;
            float rate = min(w, h) / max(w, h);
            if(rate > 0.85 && w < qr_imgx.cols/3 && h < qr_imgx.rows/3)
            {
                Point2f points[4];
                rect.points(points);

                //for(int i=0; i<4; i++)
                    //line(qr_imgy, points[i], points[(i+1)%4], Scalar(255,0,0), 2, LINE_8);
                //imshow(pic_name, qr_imgy);

                Mat imgx;

                vector<Point2f> points_start;
                for(int i=0; i<4; i++)
                    points_start.push_back(points[i]);

                Mat M = getPerspectiveTransform(points_start, points_changex);	//透视变化矩阵获取
                warpPerspective(qr_imgx, imgx, M, Size(100, 100));	//得到矩形标记


                int8_t QR_top = -1, QR_button = -1, QR_topx = -1, QR_buttonx = -1;

                Mat cell_top = imgx(Rect(imgx.cols/14, imgx.rows/14, imgx.cols/7*6, imgx.rows/19));
                int nZ_top = countNonZero(cell_top);
                //qDebug() << nZ_top << " " << (cell_top.cols*cell_top.rows)/8;

                if(nZ_top < (cell_top.cols*cell_top.rows)/8)
                    QR_top = 1;

                Mat cell_button = imgx(Rect(imgx.cols/14, imgx.rows/14*13, imgx.cols/7*6, imgx.rows/19));
                int nZ_button = countNonZero(cell_button);
                //qDebug() << nZ_button << " " << (cell_button.cols*cell_button.rows)/5;
                if(nZ_button < (cell_button.cols*cell_button.rows)/5)
                    QR_button = 1;

                Mat cell_topx = imgx(Rect(imgx.cols/7, imgx.rows/7, imgx.cols/7*5, imgx.rows/7));
                int nZ_topx = countNonZero(cell_topx);
                //qDebug() << nZ_topx << " " << (cell_topx.cols*cell_topx.rows)/9*7;
                if(nZ_topx > (cell_topx.cols*cell_topx.rows)/9*7)
                    QR_topx = 1;

                Mat cell_buttonx = imgx(Rect(imgx.cols/7, imgx.rows/7*5, imgx.cols/7*5, imgx.rows/7));
                int nZ_buttonx = countNonZero(cell_buttonx);
                //qDebug() << nZ_buttonx << " " << (cell_buttonx.cols*cell_buttonx.rows)/9*7;
                if(nZ_buttonx > (cell_buttonx.cols*cell_buttonx.rows)/9*7)
                    QR_buttonx = 1;
                //qDebug() << " ";
                if(QR_top == 1 && QR_button == 1 && QR_topx == 1 && QR_buttonx == 1)
                {

                    //for(int i=0; i<4; i++)
                        //line(img_QRcode, points[i], points[(i+1)%4], Scalar(255,0,0), 2, LINE_8);
                    //circle(img_QRcode, points[0], 6, Scalar(0,255,0), -1, LINE_8);
                    /*image_num++;
                    string pic_name;
                    pic_name = "img_" + to_string(image_num);

                    imshow(pic_name, imgx);*/

                    rect_small.push_back(rect);
                    //qDebug() << rect_small.size();
                }
            }
        }

        if(rect_small.size() == 3)
        {
            vector<Point2f> rect_center;
            for(int i=0; i<3; i++)
            {
                RotatedRect rect = rect_small[i];
                Point2f points[4];
                Point2f center;
                rect.points(points);

                for(int i=0; i<=3; i++)
                    center += points[i];

                center.x = center.x/4;
                center.y = center.y/4;

                rect_center.push_back(center);
            }


            Point2f right_point;
            bool point_find = 0;
            for(int i=0; i<3; i++)
            {
                float a = sqrt(pow(rect_center[i].x - rect_center[(i+1)%3].x, 2) +
                                pow(rect_center[i].y - rect_center[(i+1)%3].y, 2));
                float b = sqrt(pow(rect_center[i].x - rect_center[(i+2)%3].x, 2) +
                                pow(rect_center[i].y - rect_center[(i+2)%3].y, 2));
                float c = sqrt(pow(rect_center[(i+1)%3].x - rect_center[(i+2)%3].x, 2) +
                                pow(rect_center[(i+1)%3].y - rect_center[(i+2)%3].y, 2));

                double anglex = acos( (pow(a, 2) + pow(b, 2) - pow(c, 2)) / (2*a*b) ) * 180/3.14;
                //qDebug() << "anglex:" << anglex;

                if(abs(anglex - 90) < 5)
                {
                    right_point = rect_center[i];
                    point_find = 1;
                }
            }

            uint8_t right_num;
            if(point_find)
            {
                //uint8_t right_num;
                Point2f points[4];
                qr_rect.points(points);

                float minDist = 1e10;
                for(int i=0; i<=3; i++)
                {
                    Point vec = right_point - true_points[i];
                    float squaredDistance = vec.dot(vec);
                    squaredDistance = sqrt(squaredDistance);
                    if(squaredDistance < minDist)
                    {
                        minDist = squaredDistance;
                        right_num = i;
                    }
                }


                Point2f point_center;
                for (int j=0; j<4; j++)
                {
                    point_center += points[j];
                }
                point_center.x =  (int)point_center.x/4;
                point_center.y =  (int)point_center.y/4;

                double k0 = -(points[(right_num+3)%4].x - points[right_num].x) / sqrt((points[(right_num+3)%4].x - points[right_num].x)*(points[(right_num+3)%4].x - points[right_num].x) + (points[(right_num+3)%4].y - points[right_num].y)*(points[(right_num+3)%4].y - points[right_num].y));
                double k1 = -(points[(right_num+2)%4].x - points[(right_num+1)%4].x) / sqrt((points[(right_num+2)%4].x - points[(right_num+1)%4].x)*(points[(right_num+2)%4].x - points[(right_num+1)%4].x) + (points[(right_num+2)%4].y - points[(right_num+1)%4].y)*(points[(right_num+2)%4].y - points[(right_num+1)%4].y));
                double k2 = (points[(right_num+1)%4].y - points[right_num].y) / sqrt((points[(right_num+1)%4].y - points[right_num].y)*(points[(right_num+1)%4].y - points[right_num].y) + (points[(right_num+1)%4].x - points[right_num].x)*(points[(right_num+1)%4].x - points[right_num].x));
                double k3 = (points[(right_num+2)%4].y - points[(right_num+3)%4].y) / sqrt((points[(right_num+2)%4].y - points[(right_num+3)%4].y)*(points[(right_num+2)%4].y - points[(right_num+3)%4].y) + (points[(right_num+2)%4].x - points[(right_num+3)%4].x)*(points[(right_num+2)%4].x - points[(right_num+3)%4].x));

                double k_h = (k0 + k1) / 2;
                double k_v = (k2 + k3) / 2;
                double angle1 = acos(k_h);
                double angle2 = acos(k_v);
                //qDebug() << angle1*180/3.14 << " " << angle2*180/3.14;

                float angle;
                if(points[right_num].y <= points[(right_num+3)%4].y)
                    angle = 180 - (angle1+angle2)/2*180/3.14;
                else
                    angle = 180 + (angle1+angle2)/2*180/3.14;

                /*qDebug() << angle;
                for(int i=0; i<4; i++)
                    line(img_QRcode, points[i], points[(i+1)%4], Scalar(255,0,0), 2, LINE_8);
                circle(img_QRcode, points[right_num], 6, Scalar(0,255,0), 3, LINE_8);*/


                QR_message message_find;
                for(int i=0; i<=3; i++)
                    message_find.points[i] = points[i];
                message_find.point_center = point_center;
                message_find.tabPoint_num = right_num;
                message_find.angle = angle;


                message_find.code_image = qr_imgx;

                QImage qImg = MatToQImage(qr_imgx);
                QZXing decoder;
                QString qrmsg = decoder.decodeImage(qImg);

                //if(!qrmsg.isEmpty())
                {
                    message_find.code_string = qrmsg.toStdString();
                    QR_allFind.push_back(message_find);
                }
            }
        }
    }

    return QR_allFind;
}



/** QImage Mat2QImage(Mat cvImg)
{
    QImage qImg;
    if(cvImg.channels() == 3)
    {
        qImg = QImage((const unsigned char*)(cvImg.data),
                      cvImg.cols, cvImg.rows,
                      cvImg.cols * cvImg.channels(),
                      QImage::Format_RGB888).rgbSwapped();
    }
    else
    {
        qImg = QImage((const unsigned char*)(cvImg.data),
                      cvImg.cols, cvImg.rows,
                      cvImg.cols * cvImg.channels(),
                      QImage::Format_Indexed8);
    }
    return qImg;
} **/



QImage MatToQImage(const cv::Mat& mat)
{
    // 8-bits unsigned, NO. OF CHANNELS = 1
    if(mat.type() == CV_8UC1)
    {
        QImage image(mat.cols, mat.rows, QImage::Format_Indexed8);
        // Set the color table (used to translate colour indexes to qRgb values)
        image.setColorCount(256);
        for(int i = 0; i < 256; i++)
        {
            image.setColor(i, qRgb(i, i, i));
        }
        // Copy input Mat
        uchar *pSrc = mat.data;
        for(int row = 0; row < mat.rows; row ++)
        {
            uchar *pDest = image.scanLine(row);
            memcpy(pDest, pSrc, mat.cols);
            pSrc += mat.step;
        }
        return image;
    }
    // 8-bits unsigned, NO. OF CHANNELS = 3
    else if(mat.type() == CV_8UC3)
    {
        // Copy input Mat
        const uchar *pSrc = (const uchar*)mat.data;
        // Create QImage with same dimensions as input Mat
        QImage image(pSrc, mat.cols, mat.rows, mat.step, QImage::Format_RGB888);
        return image.rgbSwapped();
    }
    else if(mat.type() == CV_8UC4)
    {
        qDebug() << "CV_8UC4";
        // Copy input Mat
        const uchar *pSrc = (const uchar*)mat.data;
        // Create QImage with same dimensions as input Mat
        QImage image(pSrc, mat.cols, mat.rows, mat.step, QImage::Format_ARGB32);
        return image.copy();
    }
    else
    {
        qDebug() << "ERROR: Mat could not be converted to QImage.";
        return QImage();
    }
}


bool calcSafeRect(const RotatedRect& roi_rect, const Mat src, Rect_<float>& safeBoundRect)
{
    Rect_<float> boudRect = roi_rect.boundingRect();

    // boudRect的左上的x和y有可能小于0
    float tl_x = boudRect.x > 0 ? boudRect.x : 0;
    float tl_y = boudRect.y > 0 ? boudRect.y : 0;
    // boudRect的右下的x和y有可能大于src的范围
    float br_x = boudRect.x + boudRect.width < src.cols ?
        boudRect.x + boudRect.width - 1 : src.cols - 1;
    float br_y = boudRect.y + boudRect.height < src.rows ?
        boudRect.y + boudRect.height - 1 : src.rows - 1;

    float roi_width = br_x - tl_x;
    float roi_height = br_y - tl_y;

    if (roi_width <= 0 || roi_height <= 0)
        return false;

    // 新建一个mat，确保地址不越界，以防mat定位roi时抛异常
    safeBoundRect = Rect_<float>(tl_x, tl_y, roi_width, roi_height);

    return true;
}
