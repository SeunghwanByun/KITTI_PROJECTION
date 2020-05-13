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

#define Training_Velodyne_Path "C:\\data_road_velodyne\\training\\velodyne\\path"
#define Testing_Velodyne_Path "C:\\data_road_velodyne\\testing\\velodyne\\path"

#define Training_Image_Path "C:\\data_road\\training\\image_2\\path"
#define Testing_Image_Path "C:\\data_road\\testing\\image_2\\path"

#define Training_Calibrate_Path "C:\\data_road\\training\\calib\\path"
#define Testing_Calibrate_Path "C:\\data_road\\testing\\calib\\path"

#define Save_Training_Projection_Image_Path "C:\\data_road\\training\\color map\\path"
#define Save_Testing_Projection_Image_Path "C:\\data_road\\testing\\color map\\path"

vector<string> list_dir(string path);
vector<MatrixXf> Get_Homogeneous_Mat(string path, vector<string> files_calib);
void Save_Color_Mapped_Points(string vel_path, string img_path, vector<string> files_velodyne_points, vector<string> files_images, vector<MatrixXf> homogeneous_matrix, string save_path);

int main()
{
	// List in files
	vector<string> files_training_calib = list_dir(Training_Calibrate_Path);
	vector<string> files_training_image = list_dir(Training_Image_Path);
	vector<string> files_training_velodyne = list_dir(Training_Velodyne_Path);

	// Get Homogeneous Matrix
	vector<MatrixXf> Training_Homogeneous_Matrixes = Get_Homogeneous_Mat(Training_Calibrate_Path, files_training_calib);

	// Save Color Mapping
	Save_Color_Mapped_Points(Training_Velodyne_Path, Training_Image_Path, files_training_velodyne, files_training_image, Training_Homogeneous_Matrixes, Save_Training_Projection_Image_Path);

	return 0;
}

vector<string> list_dir(string path) {
	vector<string> filepath;

	struct _finddata_t fd;
	intptr_t handle;
	if ((handle = _findfirst((path + "*.*").c_str(), &fd)) == -1L)
		cout << "No such file in directory!" << endl;
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

void Save_Color_Mapped_Points(string vel_path, string img_path, vector<string> files_velodyne_points, vector<string> files_images, vector<MatrixXf> homogeneous_matrix, string save_path) {
	for (int index = 0; index < files_velodyne_points.size(); index++) {
		// Image
		Mat src = imread(img_path + files_images[index]);
		Mat zero = cv::Mat::zeros(src.size(), CV_8UC3);

		uchar* sImage = src.data;
		uchar* zImage = zero.data;

		// Read point cloud data
		int32_t num = 1000000;
		float *data = (float*)malloc(num * sizeof(float));

		// point
		float *px = data + 0;
		float *py = data + 1;
		float *pz = data + 2;
		float *pr = data + 3; // reflection intensity

		FILE *stream;
		fopen_s(&stream, (vel_path + files_velodyne_points[index]).c_str(), "rb");
		num = fread(data, sizeof(float), num, stream) / 4; // Read point cloud data, about 100,000+ points
		std::fclose(stream);

		int count = 0;
		vector<vector<float>> velodyne_points;
		vector<vector<uint32_t>> color_points;
		for (int32_t i = 0; i < num; i++) {
			VectorXf temp(4);
			temp << *px, *py, *pz, 1.0;
			
			VectorXf resultVec(3);
			resultVec = homogeneous_matrix[index] * temp;
			int u = int(resultVec[0] / resultVec[2] + 0.5);
			int v = int(resultVec[1] / resultVec[2] + 0.5);
			
			if ((resultVec[0] >= 0) &&  (u >= 0) && (u < src.cols) && (v >= 0) && (v < src.rows)) {
				vector<float> tmpVec;
				vector<uint32_t> tmpVec2;
				
				int offset = v * src.step1() + u * 3;
        
                                // Check LiDAR Point Projection Location
				cv::circle(src, cv::Point(u, v), 1, cv::Scalar(0, 255, 0), 1, 8, 0);
				
                                // Visualize the pixels to be mapped to the lidar point
				zImage[offset] = sImage[offset + 0];
				zImage[offset + 1] = sImage[offset + 1];
				zImage[offset + 2] = sImage[offset + 2];
				
       				uint8_t b = sImage[offset + 0];
				uint8_t g = sImage[offset + 1];
				uint8_t r = sImage[offset + 2];
				uint32_t rgb1 = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);
				
				tmpVec.push_back(*px);
				tmpVec.push_back(*py);
				tmpVec.push_back(*pz);
				tmpVec2.push_back(rgb1);
        
				velodyne_points.push_back(tmpVec);
				color_points.push_back(tmpVec2);
			}
			px += 4; py += 4; pz += 4; pr += 4;
		}
		cv::imshow("src_after", src);
		cv::imshow("zero_after", zero);
		cv::waitKey(30);

		int extension = files_velodyne_points[index].find(".bin");
		string save_name = files_velodyne_points[index].substr(0, extension);
		cout << "Save Name " << save_name << endl;
		string pcdExtension = ".pcd";

		/* Convert to PCD */
		// Write a file statement, PCL header format
		num = velodyne_points.size();

		FILE *writePCDStream;
		fopen_s(&writePCDStream, (save_path + save_name + pcdExtension).c_str(), "wb");
		fprintf(writePCDStream, "VERSION 0.7\n"); // Release Description
		fprintf(writePCDStream, "FIELDS x y z rgb\n"); // Dimension Description
		fprintf(writePCDStream, "SIZE 4 4 4 4\n"); // Occupy Byte Description
		fprintf(writePCDStream, "TYPE F F F U\n"); // Specific data type defiition
		fprintf(writePCDStream, "COUNT 1 1 1 1\n");
		fprintf(writePCDStream, "WIDTH %d\n", num); // number of points
		fprintf(writePCDStream, "HEIGHT 1\n"); // Unordered point cloud  defaults to 1
		fprintf(writePCDStream, "VIEWPOINT 0 0 0 1 0 0 0\n");
		fprintf(writePCDStream, "POINTS %d\n", num); // Number of points
		fprintf(writePCDStream, "DATA ascii\n"); // Document uses the character type shuom

		for (int velo = 0; velo < velodyne_points.size(); velo++) {
			fprintf(writePCDStream, "%f %f %f %u\n", velodyne_points[velo][0], velodyne_points[velo][1], velodyne_points[velo][2], color_points[velo][0]);
		}

		std::free(data);
		std::fclose(writePCDStream);
	}
}
