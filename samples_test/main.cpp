#include <opencv2/highgui.hpp>
#include <opencv2/aruco.hpp>
#include <iostream>
#include <vector>

using namespace std;
using namespace cv;

static bool readCameraParameters(string filename, Mat &camMatrix, Mat &distCoeffs) {
    FileStorage fs(filename, FileStorage::READ);
    if(!fs.isOpened())
        return false;
    fs["camera_matrix"] >> camMatrix;
    fs["distortion_coefficients"] >> distCoeffs;
    cout << "camera_matrix" << camMatrix << endl;
    cout << "distortion_coefficients" << distCoeffs << endl;
    return true;
}

int main() {

    Mat camM, distC;
    readCameraParameters("/home/likun/CLionProjects/samples_test/cmake-build-debug/camera_calibration.yaml", camM, distC);

//    FileStorage fs("test.yaml", FileStorage::WRITE);
//    vector<int> arr = { 1,2,3,4,5,6,7,8,9,10 };
//    fs <<"array_data";
//    fs << arr;
//    fs.release();

    FileStorage fsr("test.yaml", FileStorage::READ);
    vector<int> arrr;
    // cv::FileNode fea = fsr["array_data"];
    //cv::FileNodeIterator it = fea.begin();
    //fea >> arrr;
    int a;
    fsr["markerBorderBits"] >> a;
    fsr["array_data"] >> arrr;
    fsr.release();

    return 0;
}

// //@ read 2d vector
//#include <iostream>
//#include <opencv2/opencv.hpp>
//
//using namespace std;
//using namespace cv;
//
//// save on file
//void writeVectorOfVector(FileStorage &fs, string name, vector<vector<int>> &data)
//{
//    fs << name;
//    fs << "{";
//    for (int i = 0; i < data.size(); i++)
//    {
//        fs << name + "_" + to_string(i);
//        vector<int> tmp = data[i];
//        fs << tmp;
//    }
//    fs << "}";
//}
//
//// read from file
//void readVectorOfVector(FileStorage &fns, string name, vector<vector<int>> &data)
//{
//    data.clear();
//    FileNode fn = fns[name];
//    if (fn.empty()){
//        return;
//    }
//
//    FileNodeIterator current = fn.begin(), it_end = fn.end();
//    for (; current != it_end; ++current)
//    {
//        vector<int> tmp;
//        FileNode item = *current;
//        item >> tmp;
//        data.push_back(tmp);
//    }
//}
//
//
//int main()
//{
//    vector<vector<int> > test(4, std::vector<int>(3, 5));
//
//    FileStorage fs("test.yml", FileStorage::WRITE);;
//    writeVectorOfVector(fs, "values", test);
//    fs.release();
//
//    vector<vector<int> > tset;
//    FileStorage fs2("test.yml", FileStorage::READ);
//    readVectorOfVector(fs2, "values", tset);
//    fs2.release();
//
//    for(size_t i = 0; i < tset.size(); ++i)
//    {
//        for(size_t j = 0; j < tset[i].size(); ++j)
//            cout << tset[i][j] << " ";
//
//        cout << endl << endl;
//    }
//
//    return 0;
//}