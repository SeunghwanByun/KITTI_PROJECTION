#include "pch.h"
#include <Windows.h>
#include <iostream>
#include <fstream>
#include <string>
#include <vector>

using namespace std;

typedef std::wstring str_t;

vector<str_t> get_files_in_folder(str_t folder, str_t file_type = L"*.*");

int main()
{
	// File List in specific directory
	vector<str_t> list;
	vector<string> w2s_list;

	str_t path = L"path/to/the/data_road_velodyne/testing/velodyne/";
	string load_path = "path/to/the/data_road_velodyne/testing/velodyne/";
  
	list = get_files_in_folder(path);

	for (int i = 0; i < list.size(); i++) {
		string temp;

		w2s_list.push_back(temp.assign(list[i].begin(), list[i].end()));
	}

	//for (int i = 0; i < w2s_list.size(); i++) {
	//	cout << load_path + w2s_list[i] << endl;
	//}


	for (int index = 0; index < w2s_list.size(); index++) {
		// Read point cloud data
		int32_t num = 1000000;
		float *data = (float*)malloc(num * sizeof(float));

		// point
		float *px = data + 0;
		float *py = data + 1;
		float *pz = data + 2;
		float *pr = data + 3; // reflection intensity

		FILE *stream;
		fopen_s(&stream, (load_path + w2s_list[index]).c_str(), "rb");
		num = fread(data, sizeof(float), num, stream) / 4; // Read point cloud data, about 100,000+ points
		fclose(stream);

		string save_path = "path\to\the\save\path";
		int extension = w2s_list[index].find(".bin");
		string save_name = w2s_list[index].substr(0, extension);
		string pcdExtension = ".pcd";

		/* converted to PCD */
		// Write a file statement
		FILE *writePCDStream;
		fopen_s(&writePCDStream, (save_path + save_name + pcdExtension).c_str(), "wb");
		fprintf(writePCDStream, "VERSION 0.7\n"); // Release Description
		fprintf(writePCDStream, "FIELDS x y z\n"); // Dimension Description
		fprintf(writePCDStream, "SIZE 4 4 4\n"); // Occupy Byte Description
		fprintf(writePCDStream, "TYPE F F F\n"); // Specific data type definition
		fprintf(writePCDStream, "WIDTH %d\n", num); // number of points
		fprintf(writePCDStream, "HEIGHT 1\n"); // Unordered point cloud defaults to 1
		fprintf(writePCDStream, "POINTS %d\n", num); // Number of points
		fprintf(writePCDStream, "DATA ascii\n"); // Document uses the character type shuom

		// Write point cloud data
		for (int32_t i = 0; i < num; i++) {
			fprintf(writePCDStream, "%f %f %f\n", *px, *py, *pz);
			px += 4; py += 4; pz += 4; pr += 4;
		}


		free(data);

		fclose(writePCDStream);
	}	

	return 0;
}

vector<str_t> get_files_in_folder(str_t folder, str_t file_type) {
	vector<str_t> names;
	wchar_t search_path[200];
	wsprintf(search_path, L"%s/%s", folder.c_str(), file_type.c_str());
	WIN32_FIND_DATA fd;
	HANDLE hFind = ::FindFirstFile(search_path, &fd);
	if (hFind != INVALID_HANDLE_VALUE) {
		do {
			// read all (real) files in current folder
			// , delete '!' read other 2 default folder. and ..
			if (!(fd.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY)) {
				names.push_back(fd.cFileName);
			}
		} while (::FindNextFile(hFind, &fd));
		::FindClose(hFind);
	}

	return names;
}
