#include <opencv2/opencv.hpp>
#include <vector>
#include <stdlib.h>
#include <iomanip>

using namespace cv;
using std::vector;

/**
  * Threshold input image using adaptive thresholding
  */
static void _threshold(cv::InputArray _in, cv::OutputArray _out, int winSize, double constant) {

    CV_Assert(winSize >= 3);
    if(winSize % 2 == 0) winSize++; // win size must be odd
    adaptiveThreshold(_in, _out, 255, cv::ADAPTIVE_THRESH_MEAN_C,
                      cv::THRESH_BINARY_INV, winSize, constant);
}

static void _reorderCandidatesCorners(vector< vector< Point2f > > &candidates) {

    for(unsigned int i = 0; i < candidates.size(); i++) {
        double dx1 = candidates[i][1].x - candidates[i][0].x;
        double dy1 = candidates[i][1].y - candidates[i][0].y;
        double dx2 = candidates[i][2].x - candidates[i][0].x;
        double dy2 = candidates[i][2].y - candidates[i][0].y;
        double crossProduct = (dx1 * dy2) - (dy1 * dx2);

        if(crossProduct < 0.0) { // not clockwise direction
            swap(candidates[i][1], candidates[i][3]);
        }
    }
}

static Mat _extractBits(InputArray _image, InputArray _corners, int markerSize,
                        int markerBorderBits, int cellSize, double cellMarginRate,
                        double minStdDevOtsu) {

    CV_Assert(_image.getMat().channels() == 1);
    CV_Assert(_corners.total() == 4);
    CV_Assert(markerBorderBits > 0 && cellSize > 0 && cellMarginRate >= 0 && cellMarginRate <= 1);
    CV_Assert(minStdDevOtsu >= 0);

    // number of bits in the marker
    int markerSizeWithBorders = markerSize + 2 * markerBorderBits;
    int cellMarginPixels = int(cellMarginRate * cellSize);

    Mat resultImg; // marker image after removing perspective
    int resultImgSize = markerSizeWithBorders * cellSize;
    Mat resultImgCorners(4, 1, CV_32FC2);
    resultImgCorners.ptr< Point2f >(0)[0] = Point2f(0, 0);
    resultImgCorners.ptr< Point2f >(0)[1] = Point2f((float)resultImgSize - 1, 0);
    resultImgCorners.ptr< Point2f >(0)[2] =
            Point2f((float)resultImgSize - 1, (float)resultImgSize - 1);
    resultImgCorners.ptr< Point2f >(0)[3] = Point2f(0, (float)resultImgSize - 1);

    // remove perspective
    Mat transformation = getPerspectiveTransform(_corners, resultImgCorners);
    warpPerspective(_image, resultImg, transformation, Size(resultImgSize, resultImgSize),
                    INTER_NEAREST);

    // output image containing the bits
    Mat bits(markerSizeWithBorders, markerSizeWithBorders, CV_8UC1, Scalar::all(0));

    // check if standard deviation is enough to apply Otsu
    // if not enough, it probably means all bits are the same color (black or white)
    Mat mean, stddev;
    // Remove some border just to avoid border noise from perspective transformation
    Mat innerRegion = resultImg.colRange(cellSize / 2, resultImg.cols - cellSize / 2)
            .rowRange(cellSize / 2, resultImg.rows - cellSize / 2);
    meanStdDev(innerRegion, mean, stddev);
    if(stddev.ptr< double >(0)[0] < minStdDevOtsu) {
        // all black or all white, depending on mean value
        if(mean.ptr< double >(0)[0] > 127)
            bits.setTo(1);
        else
            bits.setTo(0);
        return bits;
    }

    cv::imwrite("../image/warpPerspective.png",resultImg);
    // now extract code, first threshold using Otsu
    threshold(resultImg, resultImg, 125, 255, THRESH_BINARY | THRESH_OTSU);

    cv::imwrite("../image/warpPerspective_threshold.png",resultImg);

    // for each cell
    for(int y = 0; y < markerSizeWithBorders; y++) {
        for(int x = 0; x < markerSizeWithBorders; x++) {
            int Xstart = x * (cellSize) + cellMarginPixels;
            int Ystart = y * (cellSize) + cellMarginPixels;
            Mat square = resultImg(Rect(Xstart, Ystart, cellSize - 2 * cellMarginPixels,
                                        cellSize - 2 * cellMarginPixels));
            // count white pixels on each cell to assign its value
            size_t nZ = (size_t) countNonZero(square);
            if(nZ > square.total() / 2) bits.at< unsigned char >(y, x) = 1;
        }
    }

    return bits;
}

int main()
{
    cv::Mat image;
    cv::Mat gray;
    image = cv::imread("/home/likun/CLionProjects/solvePnP/image/image.jpg");

    cv::Mat imagecut;
    cv::Rect rect(1000,30,500,400);
    imagecut = image(rect);

    cv::cvtColor(imagecut, gray, cv::COLOR_BGR2GRAY);

    double adaptiveThreshConstant = 7;
    int adaptiveThreshWinSize = 13;
    cv::Mat oneimage;
    _threshold(gray, oneimage, adaptiveThreshWinSize, adaptiveThreshConstant);

    cv::imwrite("../image/thresholding_image.png",imagecut);
    cv::imwrite("../image/thresholding_image_13.png",oneimage);


    // 寻找轮廓，返回contours
    std::vector< std::vector< cv::Point>> contours;
    cv::findContours(oneimage, contours, cv::RETR_LIST, cv::CHAIN_APPROX_NONE);

    Mat contourimg = Mat::zeros(imagecut.size(), CV_8UC3);
    if (!contours.empty()) {
        int idx = 0;
        for (; idx < contours.size(); idx ++)
        {
            Scalar color((rand() & 255), (rand() & 255), (rand() & 255));
            drawContours(contourimg, contours, idx, color, 1, 8);
        }
    }
    cv::imwrite("../image/contours_found_image.png",contourimg);

    // 轮廓近似
    double polygonalApproxAccuracyRate = 0.03;
    Mat approxcontourimg = Mat::zeros(imagecut.size(), CV_8UC3);
    std::vector< std::vector< cv::Point>> Points; //输出的近似的轮廓的角点
    if (!contours.empty()) {
        int idx = 0;
        for (; idx < contours.size(); idx ++)
        {
            std::vector< Point > approxCurve;
            approxPolyDP(contours[idx], approxCurve, double(contours[idx].size()) * polygonalApproxAccuracyRate, true);
            Points.push_back(approxCurve);

            Scalar color((rand() & 255), (rand() & 255), (rand() & 255));
            cv::polylines(approxcontourimg, approxCurve, 1, color);
        }
    }
    cv::imwrite("../image/contours_approx_image.png",approxcontourimg);

    // 过滤轮廓
    double minPerimeterRate = 0.03;
    double maxPerimeterRate = 4;
    double minCornerDistanceRate = 0.05;
    double minDistanceToBorder = 3;

    unsigned int minPerimeterPixels =
            (unsigned int)(minPerimeterRate * max(image.cols, image.rows));
    unsigned int maxPerimeterPixels =
            (unsigned int)(maxPerimeterRate * max(image.cols, image.rows));

    std::vector< std::vector< Point2f > > candidates;
    std::vector< std::vector< Point > > contoursOut;

    for(unsigned int i = 0; i < contours.size(); i++) {
        // check perimeter
        if(contours[i].size() < minPerimeterPixels || contours[i].size() > maxPerimeterPixels)
            continue;

        // check is square and is convex
        std::vector< Point > approxCurve;
        double accuracyRate = 0.03;
        approxPolyDP(contours[i], approxCurve, double(contours[i].size()) * accuracyRate, true);
        if(approxCurve.size() != 4 || !isContourConvex(approxCurve)) continue;

        // check min distance between corners
        double minDistSq =
                max(image.cols, image.rows) * max(image.cols, image.rows);
        for(int j = 0; j < 4; j++) {
            double d = (double)(approxCurve[j].x - approxCurve[(j + 1) % 4].x) *
                       (double)(approxCurve[j].x - approxCurve[(j + 1) % 4].x) +
                       (double)(approxCurve[j].y - approxCurve[(j + 1) % 4].y) *
                       (double)(approxCurve[j].y - approxCurve[(j + 1) % 4].y);
            minDistSq = min(minDistSq, d);
        }
        double minCornerDistancePixels = double(contours[i].size()) * minCornerDistanceRate;
        if(minDistSq < minCornerDistancePixels * minCornerDistancePixels) continue;

        // check if it is too near to the image border
        bool tooNearBorder = false;
        for(int j = 0; j < 4; j++) {
            if(approxCurve[j].x < minDistanceToBorder || approxCurve[j].y < minDistanceToBorder ||
               approxCurve[j].x > image.cols - 1 - minDistanceToBorder ||
               approxCurve[j].y > image.rows - 1 - minDistanceToBorder)
                tooNearBorder = true;
        }
        if(tooNearBorder) continue;

        // if it passes all the test, add to candidates vector
        std::vector< Point2f > currentCandidate;
        currentCandidate.resize(4);
        for(int j = 0; j < 4; j++) {
            currentCandidate[j] = Point2f((float)approxCurve[j].x, (float)approxCurve[j].y);
        }
        candidates.push_back(currentCandidate);
        contoursOut.push_back(contours[i]);
    }
    _reorderCandidatesCorners(candidates);


    Mat filter_contourimg = Mat::zeros(imagecut.size(), CV_8UC3);
    Mat filter_contour_raw_img = imagecut.clone();
    if (!contoursOut.empty()) {
        int idx = 0;
        for (; idx < contoursOut.size(); idx ++)
        {
            Scalar color((rand() & 255), (rand() & 255), (rand() & 255));
            drawContours(filter_contourimg, contoursOut, idx, color, 1, 8);
            drawContours(filter_contour_raw_img, contoursOut, idx, color, 1, 8);
        }
    }
    cv::imwrite("../image/filter_contourimg.png",filter_contourimg);
    cv::imwrite("../image/filter_contour_raw_img.png",filter_contour_raw_img);

    // 过滤平均距离太近的图像(这块代码移植太麻烦，假装是过滤)
    Mat filter_contour_onecontour_img = imagecut.clone();
    if (!contoursOut.empty()) {
        int idx = 1;
        for (; idx < 2; idx ++)
        {
            Scalar color((rand() & 255), (rand() & 255), (rand() & 255));
            drawContours(filter_contour_onecontour_img, contoursOut, idx, color, 1, 8);
        }
    }
    cv::imwrite("../image/filter_contour_onecontour_img.png",filter_contour_onecontour_img);

    // 取marker轮廓之内的图像
    cv::Mat pixel_in_contour;
    std::vector< Point > candi = contoursOut[1];
    cv::Rect rect2(200,120,110,110);
    pixel_in_contour = filter_contour_onecontour_img(rect2);
    cv::imwrite("../image/pixel_in_contour.png",pixel_in_contour);

    // 移除一个候选者的形变和重新二值化
    Mat candidateBits =_extractBits(gray,candidates[1],4,1,20,0.13,5);

    // 画网格
    Mat thimg = cv::imread("/home/likun/CLionProjects/solvePnP/image/warpPerspective_threshold.png");
    int thlenght = thimg.cols;
    int j = thlenght/6;
    for (int k = 0; k<=thlenght; k=k+j) {
        Scalar color(0, 0, 255);
        Point pt1 = Point(k,0);
        Point pt2 = Point(k,thlenght);
        cv::line(thimg, pt1, pt2, color,1,0);
        Point pt3 = Point(0,k);
        Point pt4 = Point(thlenght,k);
        cv::line(thimg, pt3, pt4, color,1,0);
    }
    cv::imwrite("../image/warpPerspective_threshold_grid.png",thimg);

    return 0;
}