//
// Created by likun on 19.04.20.
//

//#include "opencv2/opencv.hpp"
//#include <time.h>
//
//using namespace cv;
//
//int main(int, char** argv)
//{
//    FileStorage fs("test2.yml", FileStorage::WRITE);
//
//    fs << "StandardDeviationPoseEstimation" << "[";
//    for( int i = 0; i < 3; i++ )
//    {
//        int x = rand() % 640;
//        int y = rand() % 480;
//        uchar lbp = rand() % 256;
//
//        fs << "{:" << "x" << "[:";
//        for( int j = 0; j < 8; j++ )
//            fs << ((lbp >> j) & 1);
//        fs << "]" << "}";
//    }
//    fs << "]";
//    fs.release();
//    return 0;
//}



// *******************************************************************
/*
#include <iostream>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

// save on file
void writeVectorOfVector(FileStorage &fs, string name, vector<vector<int>> &data)
{
    fs << name;
    fs << "{";
    for (int i = 0; i < data.size(); i++)
    {
        fs << name + "_" + to_string(i);
        vector<int> tmp = data[i];
        fs << tmp;
    }
    fs << "}";
}

// read from file
void readVectorOfVector(FileStorage &fns, string name, vector<vector<int>> &data)
{
    data.clear();
    FileNode fn = fns[name];
    if (fn.empty()){
        return;
    }

    FileNodeIterator current = fn.begin(), it_end = fn.end();
    for (; current != it_end; ++current)
    {
        vector<int> tmp;
        FileNode item = *current;
        item >> tmp;
        data.push_back(tmp);
    }
}


int main()
{
    vector<vector<int> > test(4, std::vector<int>(3, 5));

    FileStorage fs("test.yml", FileStorage::WRITE);;
    writeVectorOfVector(fs, "values", test);
    fs.release();

    vector<vector<int> > tset;
    FileStorage fs2("test.yml", FileStorage::READ);
    readVectorOfVector(fs2, "values", tset);
    fs2.release();

    for(size_t i = 0; i < tset.size(); ++i)
    {
        for(size_t j = 0; j < tset[i].size(); ++j)
            cout << tset[i][j] << " ";

        cout << endl << endl;
    }

    return 0;
}
*/

// *****************************************************************************

#include <iostream>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

int main(){

    vector<vector<double> > tset;
    FileStorage fs2("dev.yml", FileStorage::READ);

    FileNode fn = fs2["StandardDeviationPoseEstimationSingleMarker"];
    if (fn.empty()){
        return 0;
    }

    FileNodeIterator current = fn.begin(), it_end = fn.end();
    for (; current != it_end; ++current)
    {
        vector<double> tmp;
        FileNode item = *current;
        item >> tmp;
        tset.push_back(tmp);
    }

    vector<vector<double> > tset2;
    FileNode fn2 = fs2["StandardDeviationPoseEstimationDoubleMarker"];
    if (fn.empty()){
        return 0;
    }

    FileNodeIterator current2 = fn2.begin(), it_end2 = fn2.end();
    for (; current2 != it_end2; ++current2)
    {
        vector<double> tmp;
        FileNode item = *current2;
        item >> tmp;
        tset2.push_back(tmp);
    }

    fs2.release();

    return 0;
}

