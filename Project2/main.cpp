#include <iostream>
#include <stdlib.h>
#include <time.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include <opencv2/nonfree/nonfree.hpp>
#include "opencv2/stitching/stitcher.hpp"


using namespace std;
using namespace cv;

Mat estimateHomography(vector<Point2f>& src, vector<Point2f>& dst){
    srand((unsigned)time(NULL));
    int index[4];
    Mat A = Mat::zeros(8, 9, CV_64F);
    for(int i=0; i<4; i++){
        index[i] = rand()%src.size();
        double us = src.at(index[i]).x;
        double vs = src.at(index[i]).y;
        double ud = dst.at(index[i]).x;
        double vd = dst.at(index[i]).y;
        A.at<double>(2*i,0) = -us;
        A.at<double>(2*i,1) = -vs;
        A.at<double>(2*i,2) = -1;
        A.at<double>(2*i,6) = us * ud;
        A.at<double>(2*i,7) = vs * ud;
        A.at<double>(2*i,8) = ud;
        A.at<double>(2*i+1,3) = -us;
        A.at<double>(2*i+1,4) = -vs;
        A.at<double>(2*i+1,5) = -1;
        A.at<double>(2*i+1,6) = us * vd;
        A.at<double>(2*i+1,7) = vs * vd;
        A.at<double>(2*i+1,8) = vd;
    }
    Mat homotmp,D,U,Vt;
    Mat homography(1,9,CV_64F);
    SVD::compute(A, D, U, Vt);
    homotmp = Vt.row(7);
    for (int i=0; i<homotmp.cols; i++){
        homography.at<double>(0,i) = homotmp.at<double>(0,i)/homotmp.at<double>(0,8);
    }
    //homography.at<double>(0,8) = 1;
    //cout<<homography<<endl;
    return homography.reshape(0,3);
    
    
}
Mat myFindHomography(vector<Point2f>& src,vector<Point2f>& dst, int maxIteration = 2000, int threshold = 10)
{
    Mat homoBest, homoCurrent;
    int vote = 0, voteBest = 4;
    Mat X(3,1,CV_64F),X_tmp(3,1,CV_64F), X_prime(3,1,CV_64F);
    X_tmp.at<double>(2,0) = 1;
    X_prime.at<double>(2,0) = 1;
    for(int i=0; i<maxIteration; i++){
        homoCurrent = estimateHomography(src, dst);
        for(int j=0; j<src.size(); j++){
            X_tmp.at<double>(0,0) = src.at(j).x;
            X_tmp.at<double>(1,0) = src.at(j).y;
            X_prime.at<double>(0,0) = dst.at(j).x;
            X_prime.at<double>(1,0) = dst.at(j).y;
            X = homoCurrent * X_tmp;
            Point2f x(X.at<double>(0,0) / X.at<double>(2,0), X.at<double>(1,0) / X.at<double>(2,0));
            Point2f x_prime(X_prime.at<double>(0,0), X_prime.at<double>(1,0));
            if (norm(x-x_prime) <= threshold)   vote++;
        }
        if(vote >= voteBest){
            voteBest = vote;
            homoBest = homoCurrent;
            //cout<<homoBest<<endl;
        }
        vote = 0;
    }
    
    
    
    return homoBest;
}


int main1() {
    char imgName[25];
    vector<Mat> imgs;
    vector<Mat> readimgs;
    for(int i = 24; i<31;i++){
        sprintf(imgName, "IMG_34%02d.jpg", i);
        Mat img = imread(imgName);
        readimgs.push_back(img);
    }
    
    for(int i=0; i<readimgs.size(); i++){
        Mat newimg(Size(readimgs[i].cols*2, readimgs[i].rows*2), CV_8UC3);
        Mat roi(newimg, Rect(readimgs[i].cols/2, readimgs[i].rows/2,readimgs[i].cols, readimgs[i].rows));
        readimgs[i].copyTo(roi);
        imgs.push_back(newimg);
        
    
    }
    
    int imgToChoose1[] = {1,3,3};
    int imgToChoose2[] = {2,2,4};
    Mat homo[3];
    vector<Mat> warpImage;
    warpImage.push_back(imgs[1]);
    for(int item = 0; item < 3;item++){
    int img_choose1 = imgToChoose1[item];
    int img_choose2 = imgToChoose2[item];

    SurfFeatureDetector detector(2000);
    vector<KeyPoint> keypoints1, keypoints2;
    detector.detect(imgs[img_choose1], keypoints1);
    detector.detect(imgs[img_choose2], keypoints2);
    
    // computing descriptors
    SurfDescriptorExtractor extractor;
    Mat descriptors1, descriptors2;
    extractor.compute(imgs[img_choose1], keypoints1, descriptors1);
    extractor.compute(imgs[img_choose2], keypoints2, descriptors2);
    
    // matching descriptors
    BFMatcher matcher(NORM_L2);
    vector<DMatch> matches;
    matcher.match(descriptors1, descriptors2, matches);

    vector<Point2f> points1, points2;
    for(int i=0; i<matches.size(); i++){
        points1.push_back(keypoints1[matches[i].queryIdx].pt);
        points2.push_back(keypoints2[matches[i].trainIdx].pt);
    }
    //Mat h = myFindHomography(points2, points1);
    Mat h = findHomography(Mat(points2), Mat(points1), CV_RANSAC);
    if(item == 1){
        h = h.inv();
    }
        
    homo[item] = h;
    Mat totalh = h;
    Mat tmph;
    for(int k = item-1; k>=0; k--){
        tmph = totalh * homo[k];
        totalh = tmph;
    }
        
    Mat warpImage2;
    if(item == 1){
        warpPerspective(imgs[img_choose1], warpImage2, totalh, Size(imgs[img_choose2].cols*2, imgs[img_choose2].rows*2), INTER_CUBIC);
    }
    else    warpPerspective(imgs[img_choose2], warpImage2, totalh, Size(imgs[img_choose2].cols*2, imgs[img_choose2].rows*2), INTER_CUBIC);
        
    warpImage.push_back(warpImage2);
    }
    Mat final(Size(readimgs[0].cols*5, readimgs[0].rows*5), CV_8UC3);
    for(int item = 3; item >=3; item--){
        
        Mat roi1(final, Rect(0, 0,  warpImage[item].cols, warpImage[item].rows));
        
        Mat mask(Size(warpImage[item].cols, warpImage[item].rows), CV_8U, Scalar(255));
        Mat tmpgray;
        cvtColor(warpImage[item], tmpgray, CV_RGB2GRAY);
        bitwise_and(tmpgray, mask, mask);
        warpImage[item].copyTo(roi1, mask);
        
    }
    
    imwrite("final.jpg",final);
 
    return 0;
    }
