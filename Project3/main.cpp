/*
 Author : Liqiang Yu
 UCLA ID : 904592975
 E-mail : yuliqiang92@gmail.com
 Based on OpenCV 2.4.9, PCL 1.6.0 and some other third-party libraries.
 */

#include <iostream>
#include <stdlib.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include <opencv2/nonfree/nonfree.hpp>
#include "opencv2/stitching/stitcher.hpp"
#include "pcl/common/common.h"
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/common/common_headers.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

using namespace std;
using namespace cv;
using namespace pcl;

double sizeRatio = 1;

void readImageParam(Mat& img1, Mat& img2, Mat& cameraMatrix, Mat& distortions){
        img1 = imread("./left_005.png");
        img2 = imread("./right_005.png");
        CvMat* cvcameraMatrix = (CvMat*)cvLoad("intrinsic_kitti.xml");
        CvMat* cvdistortions = (CvMat*)cvLoad("distortions_kitti.xml");
//    img1 = imread("./left.jpg");
//    img2 = imread("./right.jpg");
//    CvMat* cvcameraMatrix = (CvMat*)cvLoad("intrinsic.xml");
//    CvMat* cvdistortions = (CvMat*)cvLoad("distortions.xml");
    cameraMatrix = Mat(cvcameraMatrix,false);
    distortions = Mat(cvdistortions,false);
}

void resizeImage(Mat img1_ori, Mat& img1, Mat img2_ori, Mat& img2, Mat& cameraMatrix){
    resize(img1_ori, img1, Size(img1_ori.cols*sizeRatio, img1_ori.rows*sizeRatio));
    resize(img2_ori, img2, Size(img2_ori.cols*sizeRatio, img2_ori.rows*sizeRatio));
    cameraMatrix.at<double>(0,2) *= sizeRatio;
    cameraMatrix.at<double>(1,2) *= sizeRatio;
}

void featureMatch(Mat img1, Mat img2, vector<Point2f>& points1, vector<Point2f>& points2){
    SiftFeatureDetector detector(500, 3, 0.04, 12, 1.6);      //500, 3, 0.04, 12, 1.6 for kitti, 480 and other default for my image
    //SurfFeatureDetector detector(600);
    vector<KeyPoint> keypoints1, keypoints2;
    detector.detect(img1, keypoints1);
    detector.detect(img2, keypoints2);
    
    // computing descriptors
    SiftDescriptorExtractor extractor;
    //SurfDescriptorExtractor extractor;
    Mat descriptors1, descriptors2;
    extractor.compute(img1, keypoints1, descriptors1);
    extractor.compute(img2, keypoints2, descriptors2);
    
    // matching descriptors
    BFMatcher matcher(NORM_L2);
    vector<DMatch> matches;
    matcher.match(descriptors1, descriptors2, matches);
    
    //vector<Point2f> points1, points2;
    for(int i=0; i<matches.size(); i++){
        points1.push_back(keypoints1[matches[i].queryIdx].pt);
        points2.push_back(keypoints2[matches[i].trainIdx].pt);
    }
    Mat img_matches;
    drawMatches(img1, keypoints1, img2, keypoints2,matches, img_matches);
    imwrite("matches.jpg", img_matches);

}

void decomposeEssential(Mat essential, Mat& R1, Mat& R2, Mat& T){
    Mat D,U,Vt;
    SVD::compute(essential, D, U, Vt, SVD::FULL_UV);
    Matx33d W(0,-1,0,1,0,0,0,0,1);
    //Matx33d W(1,0,0,0,-1,0,0,0,1);
    R1 = U * Mat(W) * Vt;
    R2 = U * Mat(W.t()) * Vt;
    cout<<D<<endl;
    Mat_<double> T1 = U.col(2);
    Mat_<double> T2 = -U.col(2);
    if(T1.at<double>(0,0) > 0)  T = T1;
    else    T = T2;
    //cout<<T<<endl;

}

void drawLines(Mat image_to_draw, vector<cv::Vec3f> lines){
    int count = 0;
    for (cv::vector<cv::Vec3f>::const_iterator it = lines.begin(); it!=lines.end(); ++it)
    {
        cv::line(image_to_draw,
                 cv::Point(0,-(*it)[2]/(*it)[1]),
                 cv::Point(image_to_draw.cols,-((*it)[2] + (*it)[0]*image_to_draw.cols)/(*it)[1]),
                 cv::Scalar(0,0,255), 2);
        count++;
        if(count == 30) break;
    }
}

void drawInliers(Mat image_to_draw, vector<Point2f> pointsIn){
    vector<Point2f>::const_iterator itPts= pointsIn.begin();
    int  count = 0;
    while (itPts!=pointsIn.end()){
        cv::circle(image_to_draw,*itPts,4,cv::Scalar(255,0,0),2);
        count++;
        if(count == 30) break;
        itPts++;
    }
}

void extractInliers(vector<uchar> inliers, vector<Point2f> points1, vector<Point2f> points2,
                    vector<Point2f>& points1In, vector<Point2f>& points2In){
    vector<Point2f>::const_iterator itPts1= points1.begin();
    vector<Point2f>::const_iterator itPts2= points2.begin();
    vector<uchar>::const_iterator itIn= inliers.begin();
    while (itPts1!=points1.end()) {
        if (*itIn) {
            points1In.push_back(*itPts1);
            points2In.push_back(*itPts2);
        }
        ++itPts1;
        ++itPts2;
        ++itIn;
    }
}

void normalizePoints(vector<Point2f> points1,
                     vector<Point2f> points2, vector<Point2f>& npoints1, vector<Point2f>& npoints2, Mat N1, Mat N2){
//    Matx33d N1(1/stddev1.at<double>(0,0), 0, -mean1.at<double>(0,0)/stddev1.at<double>(0,0),
//               0, 1/stddev1.at<double>(1,0), -mean1.at<double>(1,0)/stddev1.at<double>(1,0),
//               0, 0, 1);
//    Matx33d N2(1/stddev2.at<double>(0,0), 0, -mean2.at<double>(0,0)/stddev2.at<double>(0,0),
//               0, 1/stddev2.at<double>(1,0), -mean2.at<double>(1,0)/stddev2.at<double>(1,0),
//               0, 0, 1);
    Mat point(3,1,CV_64F);
    Mat new_point;
    point.at<double>(2,0) = 1;
    for(int i=0; i<points1.size(); i++){
        point.at<double>(0,0) = points1[i].x;
        point.at<double>(1,0) = points1[i].y;
        new_point= N1 * point;
        Point2f npoint1;
        npoint1.x = new_point.at<double>(0,0);
        npoint1.y = new_point.at<double>(1,0);
        npoints1.push_back(npoint1);
        point.at<double>(0,0) = points2[i].x;
        point.at<double>(1,0) = points2[i].y;
        new_point= N2 * point;
        Point2f npoint2;
        npoint2.x = new_point.at<double>(0,0);
        npoint2.y = new_point.at<double>(1,0);
        npoints2.push_back(npoint2);
    }
    
}

double sampsonDistance(Mat fundamental, Point2f x1, Point2f x2){
    Mat X1(3,1,CV_64F), X2(3,1,CV_64F);
    X1.at<double>(2,0) = 1;
    X2.at<double>(2,0) = 1;
    X1.at<double>(0,0) = x1.x;
    X1.at<double>(1,0) = x1.y;
    X2.at<double>(0,0) = x2.x;
    X2.at<double>(1,0) = x2.y;
    Matx33d e3(0, -1, 0, 1, 0, 0, 0, 0, 0);
    Mat d1 = X2.t() * fundamental * X1;
    Mat d2 = (Mat)e3 * fundamental * X1;
    Mat d3 = X2.t() * fundamental * (Mat)e3;
    double sampson = pow(d1.at<double>(0,0),2) / (pow(norm(d2),2) + pow(norm(d3),2));
    return sampson;
}

Mat estimateFundamental(vector<Point2f>& src, vector<Point2f>& dst){
    srand((unsigned)time(NULL));
    int index[8];
    Mat A = Mat::zeros(8, 9, CV_64F);
    for(int i=0; i<8; i++){
        index[i] = rand()%src.size();
        double us = src.at(index[i]).x;
        double vs = src.at(index[i]).y;
        double ud = dst.at(index[i]).x;
        double vd = dst.at(index[i]).y;
        A.at<double>(i,0) = us * ud;
        A.at<double>(i,1) = us * vd;
        A.at<double>(i,2) = us;
        A.at<double>(i,3) = vs * ud;
        A.at<double>(i,4) = vs * vd;
        A.at<double>(i,5) = vs;
        A.at<double>(i,6) = ud;
        A.at<double>(i,7) = vd;
        A.at<double>(i,8) = 1;
    }
    Mat fundtmp,D,U,Vt;
    Mat fundamental(1,9,CV_64F);
    SVD::compute(A, D, U, Vt,SVD::FULL_UV);
    fundtmp = Vt.row(8);
    for (int i=0; i<fundtmp.cols; i++){
        fundamental.at<double>(0,i) = fundtmp.at<double>(0,i)/fundtmp.at<double>(0,8);
    }
    return fundamental.reshape(0,3);
    
    
}

Mat estimateFinalFundamental(vector<Point2f>& inlier_src, vector<Point2f>& inlier_dst){
    //srand((unsigned)time(NULL));
    //int index[4];
    int size = inlier_src.size();
    Mat A = Mat::zeros(size, 9, CV_64F);
    for(int i=0; i<size; i++){
        //index[i] = rand()%src.size();
        double us = inlier_src.at(i).x;
        double vs = inlier_src.at(i).y;
        double ud = inlier_dst.at(i).x;
        double vd = inlier_dst.at(i).y;
        A.at<double>(i,0) = us * ud;
        A.at<double>(i,1) = us * vd;
        A.at<double>(i,2) = us;
        A.at<double>(i,3) = vs * ud;
        A.at<double>(i,4) = vs * vd;
        A.at<double>(i,5) = vs;
        A.at<double>(i,6) = ud;
        A.at<double>(i,7) = vd;
        A.at<double>(i,8) = 1;
    }
    Mat fundtmp,D,U,Vt;
    Mat fundamental(1,9,CV_64F);
    SVD::compute(A, D, U, Vt,SVD::FULL_UV);
    fundtmp = Vt.row(8);
    for (int i=0; i<fundtmp.cols; i++){
        fundamental.at<double>(0,i) = fundtmp.at<double>(0,i)/fundtmp.at<double>(0,8);
    }
    //homography.at<double>(0,8) = 1;
    //cout<<homography<<endl;
    return fundamental.reshape(0,3);
    
    
}

Mat myFindFundamental(vector<Point2f>& src,vector<Point2f>& dst, vector<Point2f>& inlier_src, vector<Point2f>& inlier_dst,
                      vector<uchar>& inliers, int maxIteration = 20000, double threshold = 0.005)
{
    Mat fundBest, fundCurrent;
    int vote = 0, voteBest = 8;
    vector<Point2f> inlier_src_tmp, inlier_dst_tmp;
    
    Mat X(3,1,CV_64F),X_prime(3,1,CV_64F);
    X.at<double>(2,0) = 1;
    X_prime.at<double>(2,0) = 1;
    double sampson = 0;
    for(int i=0; i<maxIteration; i++){
        vector<uchar> inliers_tmp(src.size(), 0);
        fundCurrent = estimateFundamental(src, dst);
        //cout<<homoCurrent<<endl;
        for(int j=0; j<src.size(); j++){
            sampson = sampsonDistance(fundCurrent, src.at(j), dst.at(j));
            if (sampson <= threshold){
                vote++;
                inlier_src_tmp.push_back(src.at(j));
                inlier_dst_tmp.push_back(dst.at(j));
                inliers_tmp.at(j) = 1;
            }
        }
        if(vote >= voteBest){
            voteBest = vote;
            fundBest = fundCurrent;
            inlier_src.clear();
            inlier_dst.clear();
            inlier_src = inlier_src_tmp;
            inlier_dst = inlier_dst_tmp;
            inliers = inliers_tmp;
            
        }
        inlier_src_tmp.clear();
        inlier_dst_tmp.clear();
        vote = 0;
    }
    cout<<voteBest<<endl;
    return fundBest;
}

Mat calculateFundamental(vector<Point2f> points1, vector<Point2f> points2, vector<Point2f>& points1In,
                         vector<Point2f>& points2In, vector<uchar>& inliers){
    Mat mean1, stddev1, mean2, stddev2;
    meanStdDev(points1, mean1, stddev1);
    meanStdDev(points2, mean2, stddev2);
    Matx33d N1(1/stddev1.at<double>(0,0), 0, -mean1.at<double>(0,0)/stddev1.at<double>(0,0),
               0, 1/stddev1.at<double>(1,0), -mean1.at<double>(1,0)/stddev1.at<double>(1,0),
               0, 0, 1);
    Matx33d N2(1/stddev2.at<double>(0,0), 0, -mean2.at<double>(0,0)/stddev2.at<double>(0,0),
               0, 1/stddev2.at<double>(1,0), -mean2.at<double>(1,0)/stddev2.at<double>(1,0),
               0, 0, 1);
    vector<Point2f> npoints1, npoints2;
    normalizePoints(points1, points2, npoints1, npoints2, (Mat)N1, (Mat)N2);
//    Mat fundamental = findFundamentalMat((Mat)points1, (Mat)points2, inliers, FM_RANSAC, 0.0005);
//    return fundamental;
    //return (Mat)N2.t() * fundamental * (Mat)N1;
    vector<Point2f> inlier_src, inlier_dst;
    Mat f1 = myFindFundamental(npoints1, npoints2, inlier_src, inlier_dst, inliers);
    Mat fundamental = estimateFinalFundamental(inlier_src, inlier_dst);
    return (Mat)N2.t() * fundamental * (Mat)N1;
}

void calculateEpipoles(Mat fundamental, Mat& epipoleLeft, Mat& epipoleRight){
    Mat Dl,Ul,Vtl;
    Mat Dr,Ur,Vtr;
    SVD::compute(fundamental, Dl, Ul, Vtl, SVD::FULL_UV);
    epipoleLeft = Vtl.row(2);
    epipoleLeft = epipoleLeft/epipoleLeft.at<double>(0,2);
    SVD::compute(fundamental.t(), Dr, Ur, Vtr, SVD::FULL_UV);
    epipoleRight = Vtr.row(2);
    epipoleRight = epipoleRight/epipoleRight.at<double>(0,2);
}


Mat constructRect(Mat T){
    Matx13d e1_tmp(T.at<double>(0,0), T.at<double>(1,0), T.at<double>(2,0));
    Matx13d e2_tmp(-T.at<double>(1,0), T.at<double>(0,0), 0);
    double norm1 = sqrt(pow(T.at<double>(0,0), 2) + pow(T.at<double>(1,0), 2) + pow(T.at<double>(2,0), 2));
    double norm2 = sqrt(pow(T.at<double>(0,0), 2) + pow(T.at<double>(1,0), 2));
    Mat e2 = (Mat)e2_tmp;
    Mat e1 = (Mat)e1_tmp;
    e2 = e2 / norm2;
    e1 = e1 / norm1;
    Mat e3 = e1.cross(e2);
    Mat rect(3,3,CV_64F);
    for(int i=0; i<3; i++){
        rect.at<double>(0,i) = e1.at<double>(0,i);
        rect.at<double>(1,i) = e2.at<double>(0,i);
        rect.at<double>(2,i) = e3.at<double>(0,i);
    }
    return rect;
}

void initStereoMatch(StereoSGBM& sgbm){
    sgbm.preFilterCap = 7;
    sgbm.SADWindowSize = 9;
    sgbm.P1 = 8*3*sgbm.SADWindowSize*sgbm.SADWindowSize;
    sgbm.P2 = 32*3*sgbm.SADWindowSize*sgbm.SADWindowSize;
    sgbm.minDisparity = 0;
    sgbm.numberOfDisparities = 96;
    sgbm.uniquenessRatio = 15;
    sgbm.speckleWindowSize = 100;
    sgbm.speckleRange = 2;
    sgbm.disp12MaxDiff = 1;
    sgbm.fullDP =true;
}

Mat StereoMatch(Mat left, Mat right, StereoSGBM sgbm){
    Mat disp;
    sgbm(left, right, disp);
    imwrite("disparity.jpg", disp);
    cout<<"save the disparity map successfully"<<endl;
    return disp;
}

PointCloud<PointXYZRGB>::Ptr reprojectTo3D(Mat left, Mat disp, Mat Q){
    Mat reproject_img;
    reprojectImageTo3D(disp, reproject_img, Q);
    PointXYZRGB point;
    Point3f cvPoint;
    PointCloud<PointXYZRGB>::Ptr myPointCloud(new PointCloud<PointXYZRGB>);
    for(int j=0; j<reproject_img.rows; j++){
        for(int k=0; k<reproject_img.cols; k++){
            cvPoint = reproject_img.at<Point3f>(j,k);
            if(abs(cvPoint.z) < 1500 && abs(cvPoint.x) < 1200 && abs(cvPoint.y) < 1200){
                point.x = cvPoint.x;
                point.y = cvPoint.y;
                point.z = cvPoint.z;
                point.b = left.at<Vec3b>(j,k)[0];
                point.g = left.at<Vec3b>(j,k)[1];
                point.r = left.at<Vec3b>(j,k)[2];
                myPointCloud->points.push_back(point);
            }
        }
    }
    myPointCloud->width = (int) myPointCloud->points.size();
    myPointCloud->height = 1;
    return myPointCloud;
}

void showPointCloud(PointCloud<PointXYZRGB>::Ptr myPointCloud){
    boost::shared_ptr<visualization::PCLVisualizer> viewer(new visualization::PCLVisualizer ("Point Cloud"));
    viewer->setBackgroundColor (0, 0, 0);
    viewer->addPointCloud<pcl::PointXYZRGB> (myPointCloud, "sample cloud");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
    viewer->addCoordinateSystem (1000);
    viewer->initCameraParameters ();
    while(!viewer->wasStopped())
    {
        viewer->spinOnce (100);
    }
}

int main() {
    
    Mat img1_ori, img1, img2_ori, img2, cameraMatrix, distortions;
    readImageParam(img1_ori, img2_ori, cameraMatrix, distortions);
    resizeImage(img1_ori, img1, img2_ori, img2, cameraMatrix);
    
    vector<Point2f> points1, points2;
    featureMatch(img1, img2, points1, points2);
    cout<<points1.size()<<endl;
    
    
    vector<uchar> inliers(points1.size(), 0);
    vector<Point2f> points1In, points2In;
//    Mat fundamental = calculateFundamental(points1, points2, points1In, points2In, inliers);
//    fundamental = fundamental / fundamental.at<double>(2,2);
    Mat fundamental = findFundamentalMat((Mat)points1, (Mat)points2, inliers, FM_RANSAC);
    extractInliers(inliers, points1, points2, points1In, points2In);
    cout<<points1In.size()<<endl;
    
    
    Mat essential = cameraMatrix.t() * fundamental * cameraMatrix;
    vector<cv::Vec3f> epilines1, epilines2;
    computeCorrespondEpilines((Mat)points1In, 1, fundamental, epilines1);
    computeCorrespondEpilines((Mat)points2In, 2, fundamental, epilines2);
    cout<<fundamental<<endl;
    drawLines(img1, epilines1);
    drawInliers(img1, points1In);
    imwrite("epilines1.jpg", img1);
    drawLines(img2, epilines2);
    drawInliers(img2, points2In);
    imwrite("epilines2.jpg", img2);
    
    
    Mat_<double> R1, R2,T;
    decomposeEssential(essential, R1, R2,T);
    Mat_<double> epipoleLeft, epipoleRight;
    calculateEpipoles(fundamental, epipoleLeft, epipoleRight);
    
    Mat recImage1, recImage2;
    Mat H1, H2;
    stereoRectifyUncalibrated(points1In, points2In, fundamental, Size(img1.cols, img1.rows), H1, H2);
    warpPerspective(img1, recImage1, H1, Size(img1.cols, img1.rows), INTER_CUBIC);
    warpPerspective(img2, recImage2, H2, Size(img2.cols, img2.rows), INTER_CUBIC);
    cout<<H1 * epipoleLeft.t()<<endl;
    cout<<H2 * epipoleRight.t()<<endl;
    imwrite("left_rec.jpg", recImage1);
    imwrite("right_rec.jpg", recImage2);
    StereoSGBM sgbm;
    initStereoMatch(sgbm);
    Mat disp = StereoMatch(img1, img2, sgbm);
    Matx44d Q(1,0,0,-cameraMatrix.at<double>(0,2),
              0,1,0,-cameraMatrix.at<double>(1,2),
              0,0,0,cameraMatrix.at<double>(0,0),
              0,0,1.0/100,0);
    PointCloud<PointXYZRGB>::Ptr myPointCloud;
    myPointCloud = reprojectTo3D(img1, disp, (Mat)Q);
    //showPointCloud(myPointCloud);
    pcl::io::savePLYFileASCII ("myPointCloud.ply", *myPointCloud);
    return 0;
}
