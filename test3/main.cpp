#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <iomanip>
#include <vector>

#include <opencv2/core.hpp>

bool Readcameracalib()
{
    std::string calibfile_path = "/home/likun/catkin_ws/src/localization_with_artrack_cv/config";
    std::string line;
    std::vector<double> camera_matrix;
    std::vector<double> distortion;
    std::stringstream ss;
    std::ifstream posefile(calibfile_path+"/camera_calibration.yaml");
    int flag = 1;

    if (posefile.is_open())
    {
         while (getline (posefile, line))
         {
             if(flag == 7)
             {
                int n1 = line.find('[');
                int n2 = line.find(']');
                int n3 = n2-n1-1;
                line = line.substr(n1+1,n3);
                ss.clear();
                ss.str("");
                ss.str(line);

                double mxvalue;

                while(ss >> mxvalue)
                {
                    if (ss.peek() == ',' || ss.peek() == ' ')
                        ss.ignore();

                    camera_matrix.push_back(mxvalue);
                }

                 for (int j = 0; j < camera_matrix.size(); ++j)
                 {
                     std::cout << std::setprecision(12) << camera_matrix[j] <<"  ";
                 }
                 std::cout << std::endl;
             }
             else if(flag == 12)
             {
                 int n1 = line.find('[');
                 int n2 = line.find(']');
                 int n3 = n2-n1-1;
                 line = line.substr(n1+1,n3);
                 ss.clear();
                 ss.str("");
                 ss.str(line);

                 double mxvalue;

                 while(ss >> mxvalue)
                 {
                     if (ss.peek() == ',' || ss.peek() == ' ')
                         ss.ignore();

                     distortion.push_back(mxvalue);
                 }
             }

             flag ++;

         }
         posefile.close();
    }
    else
    {
        std::cout << "Unable to open file";
        return false;
    }

    return true;
}

int main()
{
    Readcameracalib();
    return 0;
}