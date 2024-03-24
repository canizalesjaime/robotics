#include <iostream>
#include <cmath>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

/*****************************************************************************/
double convert_to_degree(double rad)
{
    return rad * 180/M_PI;
}

/*****************************************************************************/
double convert_to_radian(double degree)
{
    return degree*M_PI/180;
}

/*****************************************************************************/
void rot_matrix_to_euler_angles_zyx(Mat rot_mat)
{
    double r31=rot_mat.at<double>(2,0);
    double r21=rot_mat.at<double>(1,0); 
    double r11=rot_mat.at<double>(0,0);
    double r32=rot_mat.at<double>(2,1); 
    double r33=rot_mat.at<double>(2,2);

    double beta1 = -asin(r31);
    double beta2 = M_PI - beta1;

    double alpha1 = convert_to_degree(atan2(r21/cos(beta1), r11/cos(beta1)));
    double alpha2 = convert_to_degree(atan2(r21/cos(beta2), r11/cos(beta2)));
    
    double gamma1 = convert_to_degree(atan2(r32/cos(beta1), r33/cos(beta1)));
    double gamma2 = convert_to_degree(atan2(r32/cos(beta2), r33/cos(beta2)));
    beta1 = convert_to_degree(beta1);
    beta2 = convert_to_degree(beta2);
    cout.precision(3);
    cout<<"("<<alpha1<<","<<beta1<<","<<gamma1<<")"<<endl;
    cout<<"("<<alpha2<<","<<beta2<<","<<gamma2<<")"<<endl;
}

/*****************************************************************************/
Mat euler_angles_zyx_to_rot_matrix(double alpha, double beta, double gamma)
{ 
    cout.precision(3);
    alpha=convert_to_radian(alpha);
    double data_z[9] = { cos(alpha), -sin(alpha), 0,
                         sin(alpha), cos(alpha), 0,
                         0, 0, 1 };
    Mat rot_z = Mat(3, 3, CV_64F, data_z);
    //cout << "rotation z:"<<endl << rot_z << endl;

    beta = convert_to_radian(beta);
    double data_y[9] = { cos(beta), 0, sin(beta),
                         0, 1, 0,
                        -sin(beta), 0, cos(beta) };
    Mat rot_y = Mat(3, 3, CV_64F, data_y);
    //cout << "rotation y:" << endl << rot_y << endl;

    gamma = convert_to_radian(gamma);
    double data_x[9] = { 1, 0, 0,
                         0, cos(gamma), -sin(gamma),
                         0, sin(gamma), cos(gamma) };
    Mat rot_x = Mat(3, 3, CV_64F, data_x);
    //cout << "rotation x:" << endl <<rot_x << endl;

    Mat rotation_matrix = rot_z*rot_y*rot_x;

    Ptr<Formatter> formatMat=Formatter::get(Formatter::FMT_DEFAULT);
    formatMat->set64fPrecision(2);
    formatMat->set32fPrecision(2);
    cout<<"rotation matrix:"<<endl<<formatMat->format(rotation_matrix)<<endl;

    return rotation_matrix;
}

/*****************************************************************************/
void hardcode_matrix_mult()
{
    float data1[9] = {1.0, 0.0, 0.0,
                      0.0, 1.0, 0.0,
                      0.0, 0.0, 1.0};
    Mat M1 = Mat(3, 3, CV_32F, data1);
    float data2[9] = {3.0, 0.0, 7.0,
                      0.0, 1.0, 0.0,
                      0.0, 0.0, 1.0};
    Mat M2 = Mat(3, 3, CV_32F, data2);
    cout<< M1*M2<<endl;
}

/*****************************************************************************/
int main()
{
    float data[9] = {.15,   .18,  -.97,
                     .09,   -.98, -.16,
                      -.98, -.06, -.16};
    Mat rot_mat = Mat(3, 3, CV_32F, data);
    rot_matrix_to_euler_angles_zyx(rot_mat);
    cout<<endl;
    euler_angles_zyx_to_rot_matrix(-90,0,0);
    cout<<endl;
    euler_angles_zyx_to_rot_matrix(90,180,-180);
    return 0;
}