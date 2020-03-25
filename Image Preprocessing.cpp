/*
Author : Seunghwan Byun
Content : Read the .pcd file in data_road_velodyne that contains point cloud data for training and testing neural networks.
Information of the axis :
	=> Camera : x = right, y = down, z = forward
	=> Velodyne : x = forward, y = left, z = up
	=> GPS/IMU : x = forward, y = left, z = up
*/

// Basic Header
#include "pch.h"
#include <Windows.h>
#include <io.h>
#include <iostream>
#include <string>
#include <fstream>
#include <sstream>
#include <thread>
#include <vector>
#include <algorithm>

// Eigen Header
#include <Eigen/Dense>

// OpenCV Headers
#include "opencv342.h"

using namespace std;
using namespace cv;
using namespace Eigen;

# File Directories
#define Training_Velodyne_Path "C:\\data_road_velodyne\\training\\velodyne\\path"
#define Testing_Velodyne_Path "C:\\data_road_velodyne\\testing\\velodyne\\path"

#define Training_Image_Path "C:\\data_road\\training\\image_2\\path"
#define Testing_Image_Path "C:\\data_road\\testing\\image_2\\path"

#define Training_Calibrate_Path "C:\\data_road\\training\\calib\\path"
#define Testing_Calibrate_Path "C:\\data_road\\testing\\calib\\path"

#define Save_Training_Projection_Image_Path "C:\\data_road\\training\\projection\\path"
#define Save_Testing_Projection_Image_Path "C:\\data_road\\testing\\projection\\path"

vector<string> list_dir(string path);
vector<MatrixXf> Get_Homogeneous_Mat(string path, vector<string> files_calib);
vector<vector<vector<float>>> Get_Velodyne_Points(string path, vector<string> files_velodyne_points);
vector<Mat> Get_Result_Mat(vector<MatrixXf> homogeneous, vector<vector<vector<float>>> velodyne, string path, vector<string> files_image);
void Intensity_Normalization(vector<vector<vector<float>>> &velodyne, string path, vector<string> files_image);

int main() {
	// This is files in above directory.
	vector<string> files_training_calib;
	vector<string> files_testing_calib;
	vector<string> files_training_Image;
	vector<string> files_testing_Image;
	vector<string> files_training_velodyne_points;
	vector<string> files_testing_velodyne_points;

	// List in files
	files_training_calib = list_dir(Training_Calibrate_Path); // calibration parameters in training folder
	files_testing_calib = list_dir(Testing_Calibrate_Path); // calibration parameters in testing folder
	files_training_Image = list_dir(Training_Image_Path); // images in training folder
	files_testing_Image = list_dir(Testing_Image_Path); // images in testing folder
	files_training_velodyne_points = list_dir(Training_Velodyne_Path); // point clouds in training folder
	files_testing_velodyne_points = list_dir(Testing_Velodyne_Path); // point clouds in testing folder

	// Get Homogeneous Matrix
	vector<MatrixXf> Homogeneous_Matrixes = Get_Homogeneous_Mat(Training_Calibrate_Path, files_testing_calib);

	// Get Velodyne points
	vector<vector<vector<float>>> velodyne_points = Get_Velodyne_Points(Training_Velodyne_Path, files_testing_velodyne_points);

	// Intensity normalization
	Intensity_Normalization(velodyne_points, Training_Velodyne_Path, files_training_velodyne_points);

	// Get Projection image
	vector<Mat> resultMat = Get_Result_Mat(Homogeneous_Matrixes, velodyne_points, Training_Image_Path, files_testing_Image);

	for (int i = 0; i < resultMat.size(); i++) {		
		//string savepath = Save_Testing_Projection_Image_Path + string("pr_") + files_testing_Image[i];
		string savepath = string("projection/pr_") + files_testing_Image[i];

		cv::imwrite(savepath, resultMat[i]);
	}

	return 0;
}

vector<string> list_dir(string path) {
	vector<string> filepath;

	struct _finddata_t fd;
	intptr_t handle;
	if ((handle = _findfirst((path + "*.*").c_str(), &fd)) == -1L)
		cout << "No file in directory!" << endl;
	do {
		if (fd.name != "." && fd.name != "..")
			filepath.push_back(fd.name);
	} while (_findnext(handle, &fd) == 0);
	_findclose(handle);

	sort(filepath.begin(), filepath.end());
	filepath.erase(filepath.begin() + 0);
	filepath.erase(filepath.begin() + 0);

	return filepath;
}

vector<MatrixXf> Get_Homogeneous_Mat(string path, vector<string> files_calib) {
	// Return Variable
	vector<MatrixXf> Homogeneous_Matrixes;

	// Initialization Matrix
	MatrixXf P2_rect(3, 4);
	MatrixXf R0_rect(4, 4);
	MatrixXf Tr_velo_to_cam_init(3, 4);
	MatrixXf Tr_velo_to_cam(4, 4);
	VectorXf v(4);
	v << 0.0, 0.0, 0.0, 1.0;

	R0_rect.row(3) = v;
	R0_rect.col(3) = v;
	Tr_velo_to_cam.row(3) = v;

	string in_line;
	for (int index = 0; index < files_calib.size(); index++) {
		ifstream in(path + files_calib[index]);
		while (getline(in, in_line)) {
			int loc_colon = in_line.find(":");
			if (loc_colon != string::npos) {
				if (in_line.substr(0, loc_colon).find("P2") != string::npos) {
					string extra_string = in_line.substr(loc_colon + 1, string::npos);
					string num;
					stringstream ss(extra_string);
					vector<float> temp;
					while (ss >> num) {
						temp.push_back(atof(num.c_str()));
					}
					ss.clear();

					P2_rect(0, 0) = temp[0];
					P2_rect(0, 1) = temp[1];
					P2_rect(0, 2) = temp[2];
					P2_rect(0, 3) = temp[3];
					P2_rect(1, 0) = temp[4];
					P2_rect(1, 1) = temp[5];
					P2_rect(1, 2) = temp[6];
					P2_rect(1, 3) = temp[7];
					P2_rect(2, 0) = temp[8];
					P2_rect(2, 1) = temp[9];
					P2_rect(2, 2) = temp[10];
					P2_rect(2, 3) = temp[11];
				}
				if (in_line.substr(0, loc_colon).find("R0_rect") != string::npos) {
					string extra_string = in_line.substr(loc_colon + 1, string::npos);
					string num;
					stringstream ss(extra_string);
					vector<float> temp;
					while (ss >> num) {
						temp.push_back(atof(num.c_str()));
					}
					R0_rect(0, 0) = temp[0];
					R0_rect(0, 1) = temp[1];
					R0_rect(0, 2) = temp[2];
					R0_rect(0, 3) = 0.0;
					R0_rect(1, 0) = temp[3];
					R0_rect(1, 1) = temp[4];
					R0_rect(1, 2) = temp[5];
					R0_rect(1, 3) = 0.0;
					R0_rect(2, 0) = temp[6];
					R0_rect(2, 1) = temp[7];
					R0_rect(2, 2) = temp[8];
					R0_rect(2, 3) = 0.0;
					R0_rect(3, 0) = 0.0;
					R0_rect(3, 1) = 0.0;
					R0_rect(3, 2) = 0.0;
					R0_rect(3, 3) = 1.0;
				}
				if (in_line.substr(0, loc_colon).find("Tr_velo_to_cam") != string::npos) {
					string extra_string = in_line.substr(loc_colon + 1, string::npos);
					string num;
					stringstream ss(extra_string);
					vector<float> temp;
					while (ss >> num) {
						temp.push_back(atof(num.c_str()));
					}
					Tr_velo_to_cam(0, 0) = temp[0];
					Tr_velo_to_cam(0, 1) = temp[1];
					Tr_velo_to_cam(0, 2) = temp[2];
					Tr_velo_to_cam(0, 3) = temp[3];
					Tr_velo_to_cam(1, 0) = temp[4];
					Tr_velo_to_cam(1, 1) = temp[5];
					Tr_velo_to_cam(1, 2) = temp[6];
					Tr_velo_to_cam(1, 3) = temp[7];
					Tr_velo_to_cam(2, 0) = temp[8];
					Tr_velo_to_cam(2, 1) = temp[9];
					Tr_velo_to_cam(2, 2) = temp[10];
					Tr_velo_to_cam(2, 3) = temp[11];
				}
			}
		}

		// Homogeneous Matrix for projection, i.e. Projection Matrix
		MatrixXf Homogeneous_Matrix(3, 4);
		Homogeneous_Matrix = P2_rect * R0_rect * Tr_velo_to_cam;

		//cout << Homogeneous_Matrix << endl;

		Homogeneous_Matrixes.push_back(Homogeneous_Matrix);
	}
	
	return Homogeneous_Matrixes;
}

vector<vector<vector<float>>> Get_Velodyne_Points(string path, vector<string> files_velodyne_points) {
	vector<vector<vector<float>>> velodyne_points_list;

	for (int index = 0; index < files_velodyne_points.size(); index++) {
		ifstream read;

		vector<vector<float>> velodyne_points;
		//cout << "velodyne_points befor : " << velodyne_points.size() << endl;
		string axis;
		int line_count = 0;
		int index_count = 0;
		int count = -1;

		read.open(path + files_velodyne_points[index], ios::in | ios::binary);
		
		float temp;
		while (read.read(reinterpret_cast<char*>(&temp), sizeof(float))) {
			vector<float> tmpVec;
			if (count == -1 || index_count % 4 == 0) {
				velodyne_points.push_back(tmpVec);
				count++;
				line_count++;
			}
			velodyne_points[count].push_back(temp);

			index_count++;
		}
		velodyne_points_list.push_back(velodyne_points);

		//cout << "velodyne_points after : " << velodyne_points.size() << endl;
	}	

	return velodyne_points_list;
}

vector<Mat> Get_Result_Mat(vector<MatrixXf> homogeneous, vector<vector<vector<float>>> velodyne, string path, vector<string> files_image) {
	vector<Mat> result_Mats;
	for (int index = 0; index < files_image.size(); index++) {
		Mat src = imread(path + files_image[index]);
		Mat resultMat = cv::Mat::zeros(src.size(), CV_8UC3);
		//cv::imshow("src", src);
		
		uchar* rImage = resultMat.data;

		//cout << "velodyne_points.size() : " << velodyne[index].size() << endl;
		for (int i = 0; i < velodyne[index].size(); i++) {
			float Intensity = velodyne[index][i][3];

			VectorXf temp(4);
			temp << velodyne[index][i][0], velodyne[index][i][1], velodyne[index][i][2], 1.0;
			/*cout << temp << endl;
			cout << homogeneous[index] << endl;*/

			VectorXf resultVec(3);
			resultVec = homogeneous[index] * temp;
			// cout << int(result[0] / result[2] + 0.5) << " " << int(result[1] / result[2] + 0.5) << endl;
			int u = int(resultVec[0] / resultVec[2] + 0.5);
			int v = int(resultVec[1] / resultVec[2] + 0.5);

			if ((resultVec[0] >= 0) && (u >= 0) && (u < resultMat.cols) && (v >= 0) && (v < resultMat.rows) && (Intensity != 0)) {
				int offset = v * resultMat.step1() + u * 3;
				if (Intensity > 0.45) {
					rImage[offset + 0] = 255;
					rImage[offset + 1] = 255;
					rImage[offset + 2] = 255;
				}
				else {
					rImage[offset + 0] = 255;
					rImage[offset + 1] = 255;
					rImage[offset + 2] = 255;
				}
				
			}
		}
		//cv::imshow("Result", resultMat);
		//cv::waitKey(0);
		result_Mats.push_back(resultMat);
	}

	return result_Mats;
}

void Intensity_Normalization(vector<vector<vector<float>>> &velodyne, string path, vector<string> files_image) {
	for (int index = 0; index < files_image.size(); index++) {				
		float maximum, minimum;
		maximum = 0.0;
		minimum = 100000.0;
		float sum_intensity = 0.0;
		for (int i = 0; i < velodyne[index].size(); i++) {
			float Intensity = velodyne[index][i][3];
			sum_intensity += Intensity;
			if (Intensity > maximum) {
				maximum = Intensity;
			}
			if (Intensity < minimum) {
				minimum = Intensity;
			}
		}
		for (int i = 0; i < velodyne[index].size(); i++) {
			float Intensity = velodyne[index][i][3];
			velodyne[index][i][3] = (Intensity - minimum) / (maximum - minimum);
		}
	}
}
