#include <opencv2/highgui/highgui.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/core.hpp>

#include <vector>

int main() {
    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_250);
    cv::Ptr<cv::aruco::GridBoard> board = cv::aruco::GridBoard::create(1, 1, 0.176, 0.607, dictionary);
    board->ids = std::vector<int> {2,3};
    cv::Mat boardImage;
    board->draw( cv::Size(600, 500), boardImage, 0, 1 );

    cv::namedWindow( "board", CV_WINDOW_AUTOSIZE );
    cv::imshow("board", boardImage);
    cv::waitKey(0);
    return 0;
}