도로 검출을 위해 3D LiDAR point cloud에서 2D 이미지를 projection하는 작업을 설명한다. 사용 된 KITTI dataset은 [여기](http://www.cvlibs.net/datasets/kitti/eval_road.php)에서 얻을 수 있다. KITTI dataset는 다중 센서 교정을 위한 파라미터를 제공하는데 여기선 LiDAR 센서와 카메라 센서 만 사용하므로 데이터 세트에 적용하는 데에는 행렬 p2_rect, r0_rect 및 cam_to_vel 만 사용한다. 이 세 매개 변수를 행렬로 만들고 직렬로 곱하면 이미지에 투영 된 좌표를 얻을 수 있다. 자세한 내용은 [논문](http://www.cvlibs.net/publications/Geiger2013IJRR.pdf)에서 가져왔다.

#### 각 파일에 대한 설명

Image Preprocessing.cpp
-> dataset 경로에 가서 calibration 파일을 parsing하여 homogeneous matrix를 만들어주고 data_road_velodyne 경로의 velodyne data를 가져와 모든 point에 homogeneous를 곱해 투영된 이미지 픽셀 값을 얻어 투영을 완료한다.

Convert_bin2pcd
-> kitti dataset에서 제공하는 velodyne data는 bin 파일로 되어있다. bin 파일에 직접 접근하여 사용할 수도 있지만 pcd 형식으로 파일을 옮겨줬을 때 사용하기 편할 때가 있기 때문에 그럴 상황에 필요한 pcd 변환 코드

PointCloudColorMapping.cpp
-> 카메라 이미지를 LiDAR 포인트에 역투영하는 코드인데, 역투영 변환 행렬이 존재하는 것은 아니지만, 기존 투영 변환 행렬을 통해 LiDAR가 이미지 평면에 투영될 때 그에 매칭되는 픽셀값을 LiDAR에 가져오는 코드

# KITTI_PROJECTION

Describes the task of projecting a 2D image from a 3D LiDAR point cloud for road detection. The KITTI dataset used was provided [here](http://www.cvlibs.net/datasets/kitti/eval_road.php). The kitti data set provides parameters for multi senser calibration. I use only LiDAR sensor and camera sensor, so the parameters used to apply to the data set are the metrics p2_rect, r0_rect, and cam_to_vel. By making these three parameters into a matrix and multiplying them in series, you can get the projected coordinates on the image. Details are excerpted from the [paper](http://www.cvlibs.net/publications/Geiger2013IJRR.pdf).
