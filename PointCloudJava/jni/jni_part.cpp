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
//using namespace cv;
using namespace pcl;
namespace pc = pcl::console;
//
extern "C" {

JNIEXPORT jint JNICALL Java_com_projecttango_experiments_javapointcloud_PointCloudActivity_greetingsFromPCL(
		JNIEnv* env, jobject pcBuffer);
JNIEXPORT jint JNICALL Java_com_projecttango_experiments_javapointcloud_PointCloudActivity_getCountPlanesByPCL(jstring filename);


JNIEXPORT jint JNICALL Java_com_projecttango_experiments_javapointcloud_PointCloudActivity_getCountPlanesByPCL(jstring filename){

 return 0;

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
