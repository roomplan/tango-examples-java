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

JNIEXPORT jint JNICALL Java_com_projecttango_experiments_javapointcloud_PointCloudActivity_greetingsFromPCL(
		JNIEnv* env, jobject pcBuffer);
JNIEXPORT jint JNICALL Java_com_projecttango_experiments_javapointcloud_PointCloudActivity_getCountPlanesByPCL(
		JNIEnv* env, jstring filename);

void GetJStringContent(JNIEnv *AEnv, jstring AStr, std::string &ARes) {
	if (!AStr) {
		ARes.clear();
		return;
	}

	const char *s = AEnv->GetStringUTFChars(AStr, NULL);
	ARes = s;
	AEnv->ReleaseStringUTFChars(AStr, s);
}

JNIEXPORT jint JNICALL Java_com_projecttango_experiments_javapointcloud_PointCloudActivity_getCountPlanesByPCL(JNIEnv* env, jstring filename){


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
JNIEXPORT jint JNICALL Java_com_projecttango_experiments_javapointcloud_PointCloudActivity_greetingsFromPCL(
		JNIEnv* env, jobject pcBuffer) {
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
