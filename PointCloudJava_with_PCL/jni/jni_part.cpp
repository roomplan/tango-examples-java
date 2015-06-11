#include <jni.h>
//#include <opencv2/core/core.hpp>
//#include <opencv2/imgproc/imgproc.hpp>
//#include <opencv2/features2d/features2d.hpp>
#include <vector>
#include <string>
#include <android/log.h>
#include <iostream>
#include <pcl/common/time.h>

#include <pcl/point_types.h>

#include <pcl/point_cloud.h>

#include <pcl/io/pcd_io.h>

#include <pcl/sample_consensus/method_types.h>

#include <pcl/sample_consensus/model_types.h>

#include <pcl/segmentation/sac_segmentation.h>

#include <pcl/features/normal_3d.h>

#include <pcl/segmentation/region_growing.h>

#include <pcl/common/transforms.h>

#include <pcl/filters/statistical_outlier_removal.h>

#include "../jni/header.h"

//#include <boost/shared_ptr.hpp>
//#include <pcl/kdtree/kdtree_flann.h>

using namespace std;
using namespace pcl;
namespace pc = pcl::console;
//

void
filterPointCloud(pcl::PointCloud<PointT>::Ptr in, pcl::PointCloud<PointT>::Ptr out)
{
	// Create the filtering object
	pcl::StatisticalOutlierRemoval<PointT> sor;
	sor.setInputCloud(in);
	sor.setMeanK (50);
	sor.setStddevMulThresh (1.0);
	sor.filter(*out);
}

int findClusters(pcl::PointCloud<PointT>::Ptr cloud, std::vector <vector<int> > &clusters)
{
	pcl::search::Search<PointT>::Ptr tree = boost::shared_ptr<pcl::search::Search<PointT> > (new pcl::search::KdTree<PointT>);
	pcl::PointCloud <pcl::Normal>::Ptr normals (new pcl::PointCloud <pcl::Normal>);
	pcl::NormalEstimation<PointT, pcl::Normal> normal_estimator;
	normal_estimator.setSearchMethod (tree);
	normal_estimator.setInputCloud (cloud);
	normal_estimator.setKSearch (30); //50
	//normal_estimator.setRadiusSearch(0.1);
	normal_estimator.compute (*normals);


	pcl::RegionGrowing<PointT> reg;

	reg.setNumberOfNeighbours(20);
	reg.setCloud (cloud);
	reg.setNeighbourSearchMethod(tree);
	//reg.setIndices (indices);
	reg.setNormals(normals);
	reg.setSmoothnessThreshold (2.0 / 180.0 * M_PI);
	reg.setCurvatureThreshold  (0.5);

	reg.segmentPoints();
	std::vector<vector<int> > tempClusters;
	tempClusters=reg.getSegments();

	int tenPercentOfCloud=cloud->points.size()/10;
	int minClusterSize=min(500, tenPercentOfCloud);
	int n=0;
	for(n=0; n<tempClusters.size();n++)
	{
		if(tempClusters[n].size()>=minClusterSize)
			clusters.push_back(tempClusters[n]);
	}

	return clusters.size();

}

extern "C" {

JNIEXPORT jint JNICALL Java_com_tangoproject_experiments_javapointcloud_PointCloudActivity_greetingsFromPCL(
		JNIEnv* env, jobject, jobject pcBuffer);
JNIEXPORT jint JNICALL Java_com_tangoproject_experiments_javapointcloud_PointCloudActivity_saveRotatedPointCloud(
		JNIEnv* env, jobject, jdoubleArray pose, jstring filename);

JNIEXPORT jint JNICALL Java_com_tangoproject_experiments_javapointcloud_PointCloudActivity_getCountPlanesByPCL(
		JNIEnv* env,jobject, jstring filename);

void GetJStringContent(JNIEnv *AEnv, jstring AStr, std::string &ARes) {
	if (!AStr) {
		ARes.clear();
		return;
	}

	const char *s = AEnv->GetStringUTFChars(AStr, NULL);
	ARes = s;
	AEnv->ReleaseStringUTFChars(AStr, s);
}

JNIEXPORT jint JNICALL Java_com_tangoproject_experiments_javapointcloud_PointCloudActivity_getCountPlanesByPCL(JNIEnv* env,jobject, jstring filename){


	//Read the cloud from file
	std::string str;
	GetJStringContent(env, filename, str);
	pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>());


	pcl::PCDReader reader;
	reader.read(str, *cloud);

	//filter the cloud
	filterPointCloud(cloud,cloud);

	//extract planar planar clusters
	vector<vector<int> >clusters;
	findClusters(cloud,clusters);

	return clusters.size();

}

JNIEXPORT jint JNICALL Java_com_tangoproject_experiments_javapointcloud_PointCloudActivity_saveRotatedPointCloud(JNIEnv* env,jobject, jdoubleArray jpose, jstring filename){


	//get the pose
	double pose[7];
	jdouble *bodyd = env->GetDoubleArrayElements(jpose, NULL);
	for (int i = 0; i < 7; i++) {
		pose[i] = bodyd[i];
	}

	//TODO: Make these values variable for different devices
	Eigen::Quaternion<float> imu2Device(0.703, -0.08, -0.08, 0.703);
	imu2Device.normalize();
	Eigen::Quaternion<float> imu2Depth(0.002, -0.707, -0.707, -0.002);
	imu2Depth.normalize();


	Eigen::Vector3f shiftIMU2Depth= Eigen::Vector3f(-0.004, 0.062, -0.004);
	Eigen::Vector3f shiftDev2IMU= Eigen::Vector3f(0.0, 0.0, 0.0);

	float x_tr = (float)pose[0];

	float y_tr = (float)pose[1];

	float z_tr = (float)pose[2];

	Eigen::Vector3f translation = Eigen::Vector3f(x_tr,y_tr, z_tr);

	float x = (float)pose[3];
	float y = (float)pose[4];
	float z = (float)pose[5];
	float w = (float)pose[6];

	Eigen::Quaternion<float> rotationQuatSS2Dev=  Eigen::Quaternion<float>(w, x, y, z);;


	Eigen::Matrix3f rotationMatDev2Depth, rotationMatIMU2Depth, rotationMatIMU2Dev, rotationMatSS2Dev;

	//fill translation and rotation data
	Eigen::Quaternion<float> rotationQuatDev2Depth = rotationQuatSS2Dev * (imu2Device.inverse() * imu2Depth);
	//convert to matrix
	rotationMatDev2Depth = rotationQuatDev2Depth.matrix();
	rotationMatIMU2Dev = imu2Device.matrix();
	rotationMatSS2Dev = rotationQuatSS2Dev.matrix();

	//rotate shift imu2Depth
	rotationMatIMU2Depth=imu2Depth.matrix();
	Eigen::Vector3f rotatedShiftIMU2Depth = (rotationMatIMU2Dev.inverse() *  shiftIMU2Depth);

	//combine to one transformation
	Eigen::Affine3f transformDepth = Eigen::Affine3f::Identity();

	//final steps
	transformDepth.translate(translation + rotationMatSS2Dev* shiftDev2IMU + rotationMatSS2Dev * rotationMatIMU2Dev.inverse() * shiftIMU2Depth);
	transformDepth.rotate(rotationMatDev2Depth);


	//Read the cloud from file
	std::string str;
	GetJStringContent(env, filename, str);
	pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>());


	pcl::PCDReader reader;
	reader.read(str, *cloud);

	//filter the cloud
	filterPointCloud(cloud,cloud);

	//transform into SS coordinate frame
	pcl::transformPointCloud(*cloud, *cloud, transformDepth.matrix());

	//save the cloud to file
	string cloudName=str;
	cloudName.replace(cloudName.end()-4,str.end(),"rot.pcd");
	pcl::PCDWriter writer;
	writer.write<PointT> (cloudName,*cloud, true);

	//extract planar planar clusters
	//vector<vector<int> >clusters;
	//findClusters(cloud,clusters);

	return 0; //clusters.size();

}
JNIEXPORT jint JNICALL Java_com_tangoproject_experiments_javapointcloud_PointCloudActivity_greetingsFromPCL(
		JNIEnv* env, jobject, jobject pcBuffer) {
	jbyte *dBuf = (signed char*) env->GetDirectBufferAddress(pcBuffer);
	signed char e = dBuf[0];

	pcl::PointCloud<pcl::PointXYZRGB> myPointCloud;

	myPointCloud.width = 100;
	pcl::PointXYZRGB tempPoint;

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(
			new pcl::PointCloud<pcl::PointXYZ>);
	// Fill in the cloud data
	cloud->width = 160;
	cloud->height = 1;
	cloud->points.resize(cloud->width * cloud->height);

	for (size_t i = 0; i < cloud->points.size(); ++i) {
		cloud->points[i].x = dBuf[i];
		cloud->points[i].y = 1024 * rand() / (RAND_MAX + 1.0f);
		cloud->points[i].z = 1024 * rand() / (RAND_MAX + 1.0f);
	}
	return 10;
}
}
