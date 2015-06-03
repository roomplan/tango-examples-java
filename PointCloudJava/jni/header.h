#include <iostream>
//#include <boost/filesystem.hpp>
#include <pcl/common/time.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <string>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

#include <sensor_msgs/PointCloud2.h>
#include <pcl/io/pcd_io.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/console/time.h>

#include <android/log.h>

using namespace std;
using namespace pcl;

typedef pcl::PointXYZRGB PointT;



enum wallType
{
	Corners=0,
	InfiniteWall=1,
	LimitedWall=2,
	FloorPlane=3,
	CeilingPlane=4,
	StairBottom=5,
	StairTop=6
};


int
sampleSegmentAndSave(pcl::PointCloud<PointT>::Ptr cloud, Eigen::Affine3f transformDepth, float cornerPoints[], float hnfs[], int flag);

int
pclTest(string fileName);

int Test(pcl::PointCloud<PointT>::Ptr cloud);

float
angleBetweenNormalVecs(Eigen::Vector3f vec1, Eigen::Vector3f vec2);

void
filterPointCloud(pcl::PointCloud<PointT>::Ptr in, pcl::PointCloud<PointT>::Ptr out);

float
fitModelToCloud(pcl::PointCloud<PointT>::Ptr cloud, pcl::SacModel myModel, int method, float inlierThreshold, pcl::PointIndices::Ptr inliers, pcl::ModelCoefficients::Ptr coefficients);


int
compute (pcl::PointCloud<PointXYZ>::Ptr xyz, pcl::PointCloud<PointXYZ>::Ptr &output, int max_iterations, double threshold, bool negative);


void
findSegmentBorders(pcl::PointCloud<PointT>::Ptr cloud, vector<vector<int> >clusters,	std::vector <float> &minXOfClusters,std::vector <float>  &maxXOfClusters, std::vector <int> &indexMinXOfClusters, std::vector <int> &indexMaxXOfClusters, std::vector<PointT> &maxXPoints,std::vector<PointT> &minXPoints);

std::vector<int> sortClustersForCoord(std::vector <float> minCoordOfClusters);

Eigen::Vector3f
getNormalFromCoefficients(pcl::ModelCoefficients plane);

vector<pcl::ModelCoefficients>
addPlanesBetween(vector<pcl::ModelCoefficients> vecPlaneCoefficientsTransformed, vector<PointT> maxXPoints,vector<PointT> minXPoints,std::vector <int> orderOfClusters,Eigen::Affine3f transformDepth);

int
segmentAndSave2(pcl::PointCloud<PointT>::Ptr cloud, Eigen::Affine3f transformDepth, vector<pcl::ModelCoefficients> &allPlaneCoefficients, wallType recordedType, float cornerPoints[], float hnfs[]);

void
calcWallSegmentBorderPoints(vector<pcl::ModelCoefficients> &allPlaneCoefficients,vector<PointT> &wallSegmentEndPoints, wallType recordedType, float widthFirstSegment, float widthOfLastSegment, float roomHeight, float floorLevel, PointT initPoint);

int
findClusters(pcl::PointCloud<PointT>::Ptr cloud, std::vector<std::vector<int> > &clusters);

int
findHNFForClusters(pcl::PointCloud<PointT>::Ptr cloud, Eigen::Affine3f transformDepth, vector<vector<int> >clusters, std::vector <int> orderOfClusters, vector<pcl::ModelCoefficients> &vecSegmentPlaneCoefficientsTransformed, vector<float> &qualityOfPlaneEstimation);

int
addHiddenPlanes(Eigen::Affine3f transformDepth, vector<vector<int> >clusters, vector<pcl::ModelCoefficients> vecSegmentPlaneCoefficientsTransformed, vector<pcl::ModelCoefficients> &allPlaneCoefficients, wallType recordedType,vector<PointT> maxXPoints,vector<PointT> minXPoints,std::vector <int> orderOfClusters, vector <float>& qualityOfPlaneEstimation, float hnfs[]);

void calcPlaneIntersection(pcl::ModelCoefficients plane1, pcl::ModelCoefficients plane2, Eigen::Vector3f &dirEdgeVec, Eigen::Vector3f &supportVec);

void setReturnValues(wallType recordedType, vector<PointT> wallCorners, vector<pcl::ModelCoefficients> allPlaneCoefficients, int numPlaneSegments, float cornerPoints[], float hnfs[]);

void getTypeOfPointCloud(vector<pcl::ModelCoefficients> vecSegmentPlaneCoefficientsTransformed,vector<vector<int> >clusters, pcl::PointCloud<PointT>::Ptr cloud, Eigen::Affine3f transformDepth, std::vector <int> orderOfClusters, wallType &recordedType, float &floorLevel, float &roomHeight);

Eigen::Vector4f calcHNFForEdgePoints(float cornerPoints[], int startIndex);
