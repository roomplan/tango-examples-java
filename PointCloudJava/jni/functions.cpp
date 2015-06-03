#include "../jni/header.h"

using namespace std;
using namespace pcl;
using namespace Eigen;
////
//

int sampleSegmentAndSave(pcl::PointCloud<PointT>::Ptr cloud, Eigen::Affine3f transformDepth, float cornerPoints[], float hnfs[], int flag)
{
	vector<pcl::ModelCoefficients> allPlaneCoefficients;
	wallType recordedType;
	switch (flag)
	{
		case 0:
			recordedType=Corners;
			break;
		case 1:
			recordedType=InfiniteWall;
			break;
		case 2:
			recordedType=LimitedWall;
			break;
		case 3:
			recordedType=FloorPlane;
			break;
		case 4:
			recordedType=CeilingPlane;
			break;
		default:
			recordedType=Corners;

	}

	int numPlanes=segmentAndSave2(cloud, transformDepth, allPlaneCoefficients, recordedType, cornerPoints, hnfs);

	return numPlanes;
}

Eigen::Vector3f
getNormalFromCoefficients(pcl::ModelCoefficients plane)
{
	Eigen::Vector3f normal;

	normal(0)=plane.values[0];
	normal(1)=plane.values[1];
	normal(2)=plane.values[2];

	return normal;

}

float
angleBetweenNormalVecs(Eigen::Vector3f vec1, Eigen::Vector3f vec2)
{
	float angle=0;
	float dotProd=vec1.dot(vec2);
	angle=acos(dotProd);

	return angle;
}

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

float
fitModelToCloud(pcl::PointCloud<PointT>::Ptr cloud, pcl::SacModel myModel, int method, float inlierThreshold, pcl::PointIndices::Ptr inliers, pcl::ModelCoefficients::Ptr coefficients)
{
	// Create the segmentation object

	 pcl::SampleConsensusModelPlane<PointT>::Ptr model_p (new pcl::SampleConsensusModelPlane<PointT> (cloud));
	 pcl::RandomSampleConsensus<PointT> ransac (model_p,0.01);
	 ransac.setProbability(0.98);
	 ransac.setDistanceThreshold (.01);
	 ransac.computeModel();

	 Eigen::VectorXf model_coefficients;
	 ransac.getModelCoefficients(model_coefficients);
	 coefficients->values.resize(4);
	 coefficients->values[0]=model_coefficients(0);
	 coefficients->values[1]=model_coefficients(1);
	 coefficients->values[2]=model_coefficients(2);
	 coefficients->values[3]=model_coefficients(3);

	 vector<int> myInliers;
	 float numPoints, numInliers;

	 ransac.getInliers(myInliers);

	 numPoints=cloud->points.size();
	 numInliers=myInliers.size();

	 return numInliers/numPoints;


}
//

pcl::ModelCoefficients transformModelCoefficients(pcl::ModelCoefficients input,Eigen::Affine3f transformDepth)
{
	cout << "\tGet Normals\n";
	Eigen::Vector3f normalPlane=getNormalFromCoefficients(input);
	float faktor=1.0;
	if(normalPlane(2)<0.0)
	{
		faktor=-1.0;
		normalPlane=normalPlane*faktor;
	}

	cout << "\tRotate\n";
	//rotiere den normalenvektor
	Eigen::Vector3f rotNormalPlane=transformDepth.rotation()*normalPlane;
	Eigen::Vector3f supportPoint=-faktor* input.values[3]*normalPlane;

	PointT supportPointPCL, supportPointTransfPCL;
	supportPointPCL.x=supportPoint(0);
	supportPointPCL.y=supportPoint(1);
	supportPointPCL.z=supportPoint(2);
	supportPointTransfPCL = pcl::transformPoint(supportPointPCL,transformDepth);

	cout << "\tCalc SupportPoint\n";

	//berechne d für neue ebene
	float dTransf;
	Eigen::Vector3f pTransf;
	pTransf(0)=supportPointTransfPCL.x;
	pTransf(1)=supportPointTransfPCL.y;
	pTransf(2)=supportPointTransfPCL.z;

	dTransf= -faktor* pTransf.dot(rotNormalPlane);

	cout << "\tSave result\n";
	pcl::ModelCoefficients output;
	output.values.push_back(pTransf(0));
	output.values.push_back(pTransf(1));
	output.values.push_back(pTransf(2));
	output.values.push_back(dTransf);

	cout << "\tDone\n";

	return output;
}

void
findSegmentBorders(pcl::PointCloud<PointT>::Ptr cloud, vector<vector<int> >clusters,	std::vector <float> &minXOfClusters,std::vector <float>  &maxXOfClusters, std::vector <int> &indexMinXOfClusters, std::vector <int> &indexMaxXOfClusters, std::vector<PointT> &maxXPoints,std::vector<PointT> &minXPoints, std::vector <float> &minYOfClusters,std::vector <float>  &maxYOfClusters, std::vector <int> &indexMinYOfClusters, std::vector <int> &indexMaxYOfClusters, std::vector<PointT> &maxYPoints,std::vector<PointT> &minYPoints)
{
	//finde grenzen in X-Richtung der Punktwolke
		for(int n=0;n<clusters.size();n++)
		{
			float minX=100000.0f;
			float minY=100000.0f;

			float maxX=-100000.0f;
			float maxY=-100000.0f;

			int indexMaxX=-1;
			int indexMinX=-1;

			int indexMaxY=-1;
			int indexMinY=-1;

			for(int counter=0; counter < clusters[n].size (); counter++)
			{
				//finde minMaxX des aktuellen Segments
				if(cloud->points[clusters[n][counter]].x < minX)
				{
					minX=cloud->points[clusters[n][counter]].x;
					indexMinX=clusters[n][counter];
				}

				if(cloud->points[clusters[n][counter]].x > maxX)
				{
					maxX=cloud->points[clusters[n][counter]].x;
					indexMaxX=clusters[n][counter];
				}

				//finde minMaxY des aktuellen Segments
				if(cloud->points[clusters[n][counter]].y < minY)
				{
					minY=cloud->points[clusters[n][counter]].y;
					indexMinY=clusters[n][counter];
				}

				if(cloud->points[clusters[n][counter]].y > maxY)
				{
					maxY=cloud->points[clusters[n][counter]].y;
					indexMaxY=clusters[n][counter];
				}

			}

			//speichere werte
			minXOfClusters.push_back(minX);
			maxXOfClusters.push_back(maxX);
			indexMinXOfClusters.push_back(indexMinX);
			indexMaxXOfClusters.push_back(indexMaxX);
			maxXPoints.push_back(cloud->points[indexMaxX]);
			minXPoints.push_back(cloud->points[indexMinX]);

			minYOfClusters.push_back(minY);
			maxYOfClusters.push_back(maxY);
			indexMinYOfClusters.push_back(indexMinY);
			indexMaxYOfClusters.push_back(indexMaxY);
			maxYPoints.push_back(cloud->points[indexMaxY]);
			minYPoints.push_back(cloud->points[indexMinY]);


		}

}

std::vector<int>
sortClustersForCoord(std::vector <float> minCoordOfClusters)
{
	//sortiere Cluster nach koordinate
	std::vector <int> orderOfClusters(minCoordOfClusters.size(),0);
	std::cout << "order: ";
	int counter;
	for(int n=0;n<minCoordOfClusters.size();n++)
	{
		counter=0;
		for(int m=0;m<minCoordOfClusters.size();m++)
		{
			if(minCoordOfClusters.at(n)>minCoordOfClusters.at(m))
			{
				counter++;
			}
		}
		orderOfClusters.at(counter)=n;
		std::cout << counter << " ";
		counter=0;
	}
	return orderOfClusters;
}

vector<pcl::ModelCoefficients>
addPlanesBetween(vector<pcl::ModelCoefficients> vecPlaneCoefficientsTransformed, vector<PointT> maxXPoints,vector<PointT> minXPoints,std::vector <int> orderOfClusters,Eigen::Affine3f transformDepth)
{
	vector<pcl::ModelCoefficients> vecAllPlaneCoefficients;
	int n=0;
	if(vecPlaneCoefficientsTransformed.size()>1)
	{
		//winkel zwischen zwei aufeinanderfolgenden ebenen berechnen

		int nPlanes=1;
		for(n=1;n<vecPlaneCoefficientsTransformed.size();n++)
		{
			Eigen::Vector3f normalN, normalM, rotAxis, newNormalVec;
			normalM=getNormalFromCoefficients(vecPlaneCoefficientsTransformed.at(orderOfClusters[n-1]));
			normalM.normalize();
			normalN=getNormalFromCoefficients(vecPlaneCoefficientsTransformed.at(orderOfClusters[n]));
			normalN.normalize();

			Eigen::Quaternion<float> myQuat;
			myQuat.setFromTwoVectors(normalN,normalM);

			float angle=(2.0*acos(myQuat.w()))/M_PI*180.0;
			std::cout << "ANGLE: " << angle << " --- ";
			angle=180.0-angleBetweenNormalVecs(normalN,normalM)/M_PI*180.0;
			std::cout << angle << "\n";


			vecAllPlaneCoefficients.push_back(vecPlaneCoefficientsTransformed.at(n-1));


			//bei zu stumpfen winkeln füge eine verbindungsebene ein.
			//kann so ausgebaut werden, dass wenn berechneter schnittpunkt der zwei flachen ebenen weit vom ende des ebenenenabschnites wegliegt nur dann neue ebene eingefügt wird
			if(angle>150 || angle < 30)
			{
				nPlanes++;

				PCL_WARN("ADDING PLANE\n");
				//setze diese im 90° winkel zu erster ebene. (normalenvektor rotiert um y-Achse)
				//drehachse ist der vektor, der senkrecht auf normalM und normalM steht.
				rotAxis=normalM.cross(normalN);
				rotAxis.normalize();


				Eigen::Matrix3f rotation = (Eigen::Matrix3f)Eigen::AngleAxisf (M_PI/2.0f,rotAxis);

				//std::cout<< "Rotation: " << rotation << std::endl;
				newNormalVec=rotation*normalN;
				newNormalVec.normalize();



				// und an das rechte ende (maxX) der ersten ebene
				PointT pointOnLeftPlane=maxXPoints.at(orderOfClusters[n-1]);
				PointT pointOnRightPlane=minXPoints.at(orderOfClusters[n]);
				PointT pointOnPlane;

				if(pointOnLeftPlane.z<pointOnRightPlane.z)
				{
					pointOnPlane=pointOnLeftPlane;
				}
				else
				{
					pointOnPlane=pointOnRightPlane;
				}

				std::cout << "Using Point: " << pointOnPlane;
				PointT pointOnPlaneTransformed=pointOnPlane;
				pointOnPlaneTransformed = pcl::transformPoint(pointOnPlane,transformDepth);
				std::cout << "--> " << pointOnPlaneTransformed << endl;


				//todo: für zuweisung entscheidung ob ebene n-1 oder n ausgewählt werden muss. dies kann anhand der zwerte entschieden werden
				//TODO: richtigen aufpunkt bestimmen.


				//berechne d für neue ebene
				float d;
				Eigen::Vector3f p;
				p(0)=pointOnPlaneTransformed.x;
				p(1)=pointOnPlaneTransformed.y;
				p(2)=pointOnPlaneTransformed.z;

				d= - p.dot(newNormalVec);



				pcl::ModelCoefficients tempCoefficients;

				tempCoefficients.values.push_back(newNormalVec(0));
				tempCoefficients.values.push_back(newNormalVec(1));
				tempCoefficients.values.push_back(newNormalVec(2));
				tempCoefficients.values.push_back(d);

				vecAllPlaneCoefficients.push_back(tempCoefficients);

			}


			nPlanes++;
		}

	}
	vecAllPlaneCoefficients.push_back(vecPlaneCoefficientsTransformed.at(n-1));
	return vecAllPlaneCoefficients;
}

int Test(pcl::PointCloud<PointT>::Ptr cloud)
{
	pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
	tree->setInputCloud (cloud);

	// Extract largest cluster minus the plane
	vector<PointIndices> cluster_indices;
	PointIndices::Ptr everything_but_the_plane (new PointIndices);

	EuclideanClusterExtraction<PointT> ec;
	ec.setClusterTolerance (0.02); // 2cm
	ec.setMinClusterSize (100);
	ec.setMaxClusterSize(100000);
	ec.setSearchMethod(tree);
	ec.setInputCloud (cloud);
	ec.extract(cluster_indices);

	return cluster_indices.size();
}

int segmentAndSave2(pcl::PointCloud<PointT>::Ptr cloud,Eigen::Affine3f transformDepth, vector<pcl::ModelCoefficients> &allPlaneCoefficients, wallType recordedType, float cornerPoints[], float hnfs[])
{
	static float roomHeight=2.20;
	static float floorLevel=0.0;

	if(hnfs[99]>0.5)
	{
		char temp[100];
		sprintf(temp, "%f, %f, %f, %f", hnfs[0],hnfs[1],hnfs[2],hnfs[3]);
		__android_log_write(ANDROID_LOG_INFO, "HNF In: ", temp);
	}


	PointT initPoint;

	filterPointCloud(cloud,cloud);

	vector<vector<int> >clusters;
	findClusters(cloud,clusters);
	std::cout << "\n*******\nNumber of clusters is equal to " << clusters.size () << std::endl;


	int counter = 0;

	char temp[100];
	sprintf(temp, "Got %f Points", cornerPoints[99]);
	__android_log_write(ANDROID_LOG_INFO, "Got Points: ", temp);


	pcl::PointCloud<PointT>::Ptr cloudSegment (new pcl::PointCloud<PointT>);
	pcl::PointIndices::Ptr segmentPlaneInliers (new pcl::PointIndices);

	vector<float> qualityOfPlaneEstimation;

	pcl::ModelCoefficients::Ptr segmentPlaneCoefficients (new pcl::ModelCoefficients);
	vector<pcl::ModelCoefficients> vecSegmentPlaneCoefficients;
	pcl::ModelCoefficients::Ptr segmentPlaneCoefficientsTransformed (new pcl::ModelCoefficients);
	vector<pcl::ModelCoefficients> vecSegmentPlaneCoefficientsTransformed;

	vector<PointT> maxXPoints, minXPoints;
	std::vector <float> minXOfClusters, maxXOfClusters;
	std::vector <int> indexMinXOfClusters, indexMaxXOfClusters;
	std::vector <int> indexMinXOfClustersInOrder, indexMaxXOfClustersInOrder;

	vector<PointT> maxYPoints, minYPoints;
	std::vector <float> minYOfClusters, maxYOfClusters;
	std::vector <int> indexMinYOfClusters, indexMaxYOfClusters;
	std::vector <int> indexMinYOfClustersInOrder, indexMaxYOfClustersInOrder;

	//finde grenzen in X-Richtung der Punktwolke
	findSegmentBorders(cloud,clusters,minXOfClusters,maxXOfClusters,indexMinXOfClusters,indexMaxXOfClusters,maxXPoints, minXPoints, minYOfClusters, maxYOfClusters, indexMinYOfClusters, indexMaxYOfClusters, maxYPoints, minYPoints);
	//TODO: Cluster auf überschneidung prüfen und nur größeren behalten wenn einer den anderen in +-x überragt, d.h. umfasst.

	__android_log_write(ANDROID_LOG_INFO, "Sort", "X");
	//sortiere Cluster nach minX
	std::vector <int> orderOfClusters=sortClustersForCoord(minXOfClusters);
	for(int n=0;n<orderOfClusters.size();n++)
	{
		indexMinXOfClustersInOrder.push_back(indexMinXOfClusters[orderOfClusters[n]]);
		indexMaxXOfClustersInOrder.push_back(indexMaxXOfClusters[orderOfClusters[n]]);
	}
	__android_log_write(ANDROID_LOG_INFO, "Measure", "Width");
	//Messe länge des ersten und letzen ebenensegments.
	float widthFirstSegment;
	float widthLastSegment;

	//TODO: zur längenberechnung der segmente x und z werte des punktes verwenden.
	float dx, dy, dz;
	dx=maxXPoints[orderOfClusters[0]].x - minXPoints[orderOfClusters[0]].x;
	dz=maxXPoints[orderOfClusters[0]].z - minXPoints[orderOfClusters[0]].z;
	widthFirstSegment=sqrt(dx*dx+dz*dz); //maxXPoints[orderOfClusters[0]].x-minXPoints[orderOfClusters[0]].x;

	dx=maxXPoints[orderOfClusters[clusters.size()-1]].x - minXPoints[orderOfClusters[clusters.size()-1]].x;
	dz=maxXPoints[orderOfClusters[clusters.size()-1]].z - minXPoints[orderOfClusters[clusters.size()-1]].z;
	widthLastSegment=sqrt(dx*dx+dz*dz);//maxXPoints[orderOfClusters[clusters.size()-1]].x-minXPoints[orderOfClusters[clusters.size()-1]].x;

	__android_log_write(ANDROID_LOG_INFO, "Sort", "Y");
	//sortiere Cluster nach minY
	std::vector <int> orderOfClustersY=sortClustersForCoord(minYOfClusters);
	for(int n=0;n<orderOfClustersY.size();n++)
	{
		indexMinYOfClustersInOrder.push_back(indexMinYOfClusters[orderOfClustersY[n]]);
		indexMaxYOfClustersInOrder.push_back(indexMaxYOfClusters[orderOfClustersY[n]]);
	}
	//Messe höhe des ersten und letzen ebenensegments.
	__android_log_write(ANDROID_LOG_INFO, "Measure", "Height");
	float heightFirstSegment;
	float heightLastSegment;
	//TODO: zur höhenberechnung der segmente y und z werte des punktes verwenden.

	dy=maxYPoints[orderOfClustersY[0]].y-minYPoints[orderOfClustersY[0]].y;
	dz=maxYPoints[orderOfClustersY[0]].z-minYPoints[orderOfClustersY[0]].z;
	heightFirstSegment=sqrt(dy*dy+dz*dz);

	dy=maxYPoints[orderOfClustersY[clusters.size()-1]].y-minYPoints[orderOfClustersY[clusters.size()-1]].y;
	dz=maxYPoints[orderOfClustersY[clusters.size()-1]].z-minYPoints[orderOfClustersY[clusters.size()-1]].z;
	heightLastSegment=sqrt(dy*dy+dz*dz);



	__android_log_write(ANDROID_LOG_INFO, "FindHNF", "Now");
	//Finde Ebenengleichungen für die einzelsegmente.
	//Speichere diese in der richtigen reihenfolge
	int res=findHNFForClusters(cloud, transformDepth, clusters, orderOfClusters, vecSegmentPlaneCoefficientsTransformed, qualityOfPlaneEstimation);


	//Überprüfe ob spezieller typ automatisch erkannt werden kann.
	//ändert den wert nur, wenn es eine treppe, oder den boden bzw. decke erkennt.
	getTypeOfPointCloud(vecSegmentPlaneCoefficientsTransformed,clusters,cloud, transformDepth, orderOfClusters, recordedType, floorLevel, roomHeight);

	if(recordedType==StairBottom)
	{

		vecSegmentPlaneCoefficientsTransformed.clear();
		int res=findHNFForClusters(cloud, transformDepth, clusters, orderOfClustersY, vecSegmentPlaneCoefficientsTransformed, qualityOfPlaneEstimation);

		initPoint.x=minXPoints[orderOfClustersY[1]].x;
		initPoint.y=minYPoints[orderOfClustersY[1]].y;
		initPoint.z=minYPoints[orderOfClustersY[1]].z;

		initPoint=pcl::transformPoint(initPoint,transformDepth);
	}

	int numPlaneSegments=0;
	vector<PointT> wallCorners;

	if((recordedType!=CeilingPlane)&&(recordedType!=FloorPlane))
	{
		//Add hidden Planes
		if(recordedType==StairBottom)
		{
			numPlaneSegments = addHiddenPlanes(transformDepth, clusters, vecSegmentPlaneCoefficientsTransformed, allPlaneCoefficients, recordedType, maxYPoints, minYPoints, orderOfClustersY, qualityOfPlaneEstimation, hnfs);
		}
		else
		{
			numPlaneSegments = addHiddenPlanes(transformDepth, clusters, vecSegmentPlaneCoefficientsTransformed, allPlaneCoefficients, recordedType, maxXPoints, minXPoints, orderOfClusters, qualityOfPlaneEstimation, hnfs);
		}
		std::cout << "done." << std::endl;

		vecSegmentPlaneCoefficients.clear();
		vecSegmentPlaneCoefficientsTransformed.clear();

		//wenn nur eine ebene gefunden wurde und letze mit übergeben wurde, dann berechne länge des letzten teilstücks so, dass alles bis zur aktuellen position angezeigt wird.
		if( (hnfs[99]>0.5) && (clusters.size()==1) )
		{
			//TODO: diesen recorded type integrieren und behandlung unten als extra fall einführen
			//recordedType=InfiniteWall; --> wird aktuell in corners behandelt

			Eigen::Vector3f normalN, normalM;
			normalN=getNormalFromCoefficients(allPlaneCoefficients[0]);
			normalN.normalize();
			normalM=getNormalFromCoefficients(allPlaneCoefficients[1]);
			normalM.normalize();

			Eigen::Vector3f supportVecXYZ;
			Eigen::Vector3f dirEdgeVec;

			//berechne schnittgerade
			calcPlaneIntersection(allPlaneCoefficients[0],allPlaneCoefficients[1], dirEdgeVec, supportVecXYZ);

			//Berechne länge des letzen segments neu
			PointT maxXPointTransf = pcl::transformPoint(maxXPoints[orderOfClusters[clusters.size()-1]], transformDepth);
			float dx, dy;
			dx=maxXPointTransf.x - (-supportVecXYZ(0));
			dy=maxXPointTransf.y - (-supportVecXYZ(1));

			widthLastSegment = sqrt(dx*dx+dy*dy);
			char temp1[100];
			sprintf(temp1, "WidthLastSegment of Wall: %f", widthLastSegment);
			__android_log_write(ANDROID_LOG_INFO, "Measurement", temp1);
		}


		//Berechne EbenenEckpunkte
		__android_log_write(ANDROID_LOG_INFO, "Berechne", "Ebenenpunkte");
		calcWallSegmentBorderPoints(allPlaneCoefficients,wallCorners, recordedType, widthFirstSegment, widthLastSegment, roomHeight, floorLevel, initPoint);
		__android_log_write(ANDROID_LOG_INFO, "Done", "...");
	}

	//TODO: if(recordedType==FloorPlane) übergebe bodenebene
	if(recordedType==StairBottom) numPlaneSegments=8;

	setReturnValues(recordedType, wallCorners, allPlaneCoefficients, numPlaneSegments, cornerPoints, hnfs);

	numPlaneSegments=cornerPoints[99];

	char temp1[100];
	sprintf(temp1, "Type: %d", recordedType);
	__android_log_write(ANDROID_LOG_INFO, "Type", temp1);
	return (numPlaneSegments);
}

void calcWallSegmentBorderPoints(vector<pcl::ModelCoefficients> &allPlaneCoefficients,vector<PointT> &wallSegmentEndPoints, wallType recordedType, float widthFirstSegment, float widthLastSegment, float roomHeight, float floorLevel, PointT initPoint)
{
	Eigen::Vector3f normalN, normalM;
	if(recordedType!=StairBottom)
	{
		for(int n=1; n<allPlaneCoefficients.size();n++)
		{
			cout << "Plane: " << n << endl;
			//hole normalenvektoren
			normalN=getNormalFromCoefficients(allPlaneCoefficients.at(n-1));
			normalN.normalize();
			normalM=getNormalFromCoefficients(allPlaneCoefficients.at(n));
			normalM.normalize();
			Eigen::Vector3f supportVecXYZ, vA,vB,vC,vD;
			Eigen::Vector3f dirEdgeVec, dirVecInPlane;

			//berechne schnittgerade
			calcPlaneIntersection(allPlaneCoefficients.at(n-1),allPlaneCoefficients.at(n), dirEdgeVec, supportVecXYZ);



			//berechne vektor parallel zu boden in ebene.
			dirVecInPlane=dirEdgeVec.cross(normalM);
			dirVecInPlane.normalize();


			//Berechne Punkte
			/*				 .
			A			B  / |
			.____________./  |
			|			 |   .
			|			 |  /
			|			 | /
			.____________./
			C			D
			*/

			PointT pTemp, pTemp2;

			//Setze die Wände auf die bodenebene
			supportVecXYZ(2)=floorLevel;

			//...und zeichne sie bis zur decke
			vD=supportVecXYZ;
			vB=vD+dirEdgeVec*roomHeight;


			char temp[100];
			sprintf(temp, "Using height: %f EdgeVec: %f, %f, %f ", roomHeight, dirEdgeVec(0), dirEdgeVec(1), dirEdgeVec(2));
			__android_log_write(ANDROID_LOG_INFO, "roomheight: ", temp);


			//bei erster Ecke berechne Punkte der ersten ebene
			if(n==1)
			{
				dirVecInPlane=dirEdgeVec.cross(normalN);

				//zeichne sichtbaren bereich des ersten segments
				vC=vD-dirVecInPlane*widthFirstSegment;
				pTemp.x=vC(0);
				pTemp.y=vC(1);
				pTemp.z=vC(2);
				//Punkt speichern
				wallSegmentEndPoints.push_back(pTemp);

				vA=vC+dirEdgeVec*roomHeight;
				pTemp.x=vA(0);
				pTemp.y=vA(1);
				pTemp.z=vA(2);
				//Punkt speichern
				wallSegmentEndPoints.push_back(pTemp);

				pTemp.x=vB(0);
				pTemp.y=vB(1);
				pTemp.z=vB(2);
				//Punkt speichern
				wallSegmentEndPoints.push_back(pTemp);

				pTemp.x=vD(0);
				pTemp.y=vD(1);
				pTemp.z=vD(2);
				//Punkt speichern
				wallSegmentEndPoints.push_back(pTemp);


			}
			//bei ecke in der mitte berechne 2 punkte und übernehme andere zwei von vorher
			if (n>1 && n<allPlaneCoefficients.size())
			{

				//übernehme zuvor berechnete punkte
				pTemp=wallSegmentEndPoints.at(wallSegmentEndPoints.size()-1);
				pTemp2=wallSegmentEndPoints.at(wallSegmentEndPoints.size()-2);

				wallSegmentEndPoints.push_back(pTemp);
				wallSegmentEndPoints.push_back(pTemp2);


				//Berechne neue punkte
				pTemp.x=vB(0);
				pTemp.y=vB(1);
				pTemp.z=vB(2);
				//Punkt speichern
				wallSegmentEndPoints.push_back(pTemp);


				pTemp.x=vD(0);
				pTemp.y=vD(1);
				pTemp.z=vD(2);
				//Punkt speichern
				wallSegmentEndPoints.push_back(pTemp);

			}
			//bei letzer ecke berechne punkte der letzten ebene
			if(n==allPlaneCoefficients.size()-1)
			{

				//übernehme zuvor berechnete punkte
				//übernehme zuvor berechnete punkte
				pTemp=wallSegmentEndPoints.at(wallSegmentEndPoints.size()-1);
				pTemp2=wallSegmentEndPoints.at(wallSegmentEndPoints.size()-2);


				wallSegmentEndPoints.push_back(pTemp);
				wallSegmentEndPoints.push_back(pTemp2);


				//Berechne neue punkte
				dirVecInPlane=dirEdgeVec.cross(normalM);

				//Zeichen sichtbaren bereich in neue Wandrichtung.
				vC=vD+dirVecInPlane*widthLastSegment;
				vA=vC+dirEdgeVec*roomHeight;

				pTemp.x=vA(0);
				pTemp.y=vA(1);
				pTemp.z=vA(2);
				//Punkt speichern
				wallSegmentEndPoints.push_back(pTemp);

				pTemp.x=vC(0);
				pTemp.y=vC(1);
				pTemp.z=vC(2);
				//Punkt speichern
				wallSegmentEndPoints.push_back(pTemp);

			}
		}
	}

	if(recordedType==StairBottom)
	{
		/*      _____________
		 *	   /____________/
		 *    |____________|
		 *   /____________/
		 *D |____________| C
		 *  A			B
		 */
		PointT pTemp;
		Eigen::Vector3f supportVecXYZ, vA,vB,vC,vD;
		Eigen::Vector3f dirStepUp, dirStepSide, dirStepIn;


		float stepHeight=0.2f;

		stepHeight=(allPlaneCoefficients.at(allPlaneCoefficients.size()-1).values[3]-allPlaneCoefficients.at(0).values[3]);
		stepHeight=abs(stepHeight);

		char temp1[200];
		sprintf(temp1, "HNF1: %f, %f, %f, %f",allPlaneCoefficients.at(0).values[0],allPlaneCoefficients.at(0).values[1],allPlaneCoefficients.at(0).values[2],allPlaneCoefficients.at(0).values[3] );
		__android_log_write(ANDROID_LOG_INFO, "HNF1", temp1);
		char temp2[200];
		sprintf(temp2, "HNF3: %f, %f, %f, %f",allPlaneCoefficients.at(2).values[0],allPlaneCoefficients.at(2).values[1],allPlaneCoefficients.at(2).values[2],allPlaneCoefficients.at(2).values[3] );
		__android_log_write(ANDROID_LOG_INFO, "HNF3", temp2);

		//berechne schnittgerade
		calcPlaneIntersection(allPlaneCoefficients.at(0),allPlaneCoefficients.at(1), dirStepSide, supportVecXYZ);

		dirStepSide.normalize();
		dirStepSide*= -1.0;

		dirStepUp(0)=0.0; dirStepUp(1)=0.0; dirStepUp(2)=stepHeight;
		//dirStepSide(0)=1.0; dirStepSide(1)=0.0; dirStepSide(2)=0.0;
		//dirStepIn(0)=0.0; dirStepIn(1)=0.2 ; dirStepIn(2)=0.0;
		dirStepIn=-dirStepSide.cross(dirStepUp);

		//Setzte StartPunkt
		vA(0)=-initPoint.x;
		vA(1)=-initPoint.y;
		vA(2)=initPoint.z-stepHeight;


		char temp[200];
		sprintf(temp, "Up: %f, %f, %f Side: %f, %f, %f, In: %f, %f, %f, Z[0] und Z[1]: %f, %f, StepHeight: %f ", dirStepUp(0),dirStepUp(1),dirStepUp(2),dirStepSide(0),dirStepSide(1),dirStepSide(2),dirStepIn(0),dirStepIn(1),dirStepIn(2),allPlaneCoefficients.at(2).values[3], allPlaneCoefficients.at(0).values[3], stepHeight);
		__android_log_write(ANDROID_LOG_INFO, "Type", temp);

		//TODO: hier nur die sichtbare anzahl zeichnen und den rest einfügen, wenn treppenende aufgenommen wurde.
		for(int n=0;n<4;n++)
		{
			//Senkrechte Fläche
			vB=vA+dirStepSide;
			vC=vB+dirStepUp;
			vD=vA+dirStepUp;

			pTemp.x=vA(0);
			pTemp.y=vA(1);
			pTemp.z=vA(2);
			//Punkt speichern
			wallSegmentEndPoints.push_back(pTemp);

			pTemp.x=vB(0);
			pTemp.y=vB(1);
			pTemp.z=vB(2);
			//Punkt speichern
			wallSegmentEndPoints.push_back(pTemp);

			pTemp.x=vC(0);
			pTemp.y=vC(1);
			pTemp.z=vC(2);
			//Punkt speichern
			wallSegmentEndPoints.push_back(pTemp);

			pTemp.x=vD(0);
			pTemp.y=vD(1);
			pTemp.z=vD(2);
			wallSegmentEndPoints.push_back(pTemp);

			//Trittfläche
			vA=vD;

			vB=vA+dirStepSide;
			vC=vB+dirStepIn;
			vD=vA+dirStepIn;

			pTemp.x=vA(0);
			pTemp.y=vA(1);
			pTemp.z=vA(2);
			//Punkt speichern
			wallSegmentEndPoints.push_back(pTemp);

			pTemp.x=vB(0);
			pTemp.y=vB(1);
			pTemp.z=vB(2);
			//Punkt speichern
			wallSegmentEndPoints.push_back(pTemp);

			pTemp.x=vC(0);
			pTemp.y=vC(1);
			pTemp.z=vC(2);
			//Punkt speichern
			wallSegmentEndPoints.push_back(pTemp);

			pTemp.x=vD(0);
			pTemp.y=vD(1);
			pTemp.z=vD(2);
			wallSegmentEndPoints.push_back(pTemp);

			vA=vD;

		}
	}
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
	reg.setCurvatureThreshold  (0.5); //0.1 funktioniert auch sehr gut. Was bedeutet dieser wert genau?

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

int findHNFForClusters(pcl::PointCloud<PointT>::Ptr cloud, Eigen::Affine3f transformDepth, vector<vector<int> >clusters, std::vector <int> orderOfClusters, vector<pcl::ModelCoefficients> &vecSegmentPlaneCoefficientsTransformed, vector<float> &qualityOfPlaneEstimation)
{

	pcl::PointCloud<PointT>::Ptr cloudSegment (new pcl::PointCloud<PointT>);
	pcl::PointIndices::Ptr segmentPlaneInliers (new pcl::PointIndices);

	pcl::ModelCoefficients::Ptr segmentPlaneCoefficients (new pcl::ModelCoefficients);

	pcl::ModelCoefficients::Ptr segmentPlaneCoefficientsTransformed (new pcl::ModelCoefficients);

	for(int n=0;n<orderOfClusters.size();n++)
	{
		cloudSegment->points.clear();
		cloudSegment->clear();

		std::cout << "\nClusterIndex: " << orderOfClusters[n] << std::endl;
		for(int counter=0; counter < clusters[orderOfClusters[n]].size (); counter++)
		{
			//speichere punktwolke des aktuellen segments
			cloudSegment->points.push_back(cloud->points[clusters[orderOfClusters[n]][counter]]);

		}

		//finde Ebenenmodell für aktuelles Segment
		float quality=fitModelToCloud(cloudSegment,pcl::SACMODEL_PLANE,pcl::SAC_RANSAC,0.02,segmentPlaneInliers,segmentPlaneCoefficients);
		qualityOfPlaneEstimation.push_back(quality);

		cout << "Found coefficients: " <<" "<< segmentPlaneCoefficients->values[0] <<" "<< segmentPlaneCoefficients->values[1] <<" "<< segmentPlaneCoefficients->values[2] <<" "<< segmentPlaneCoefficients->values[3] << endl;

		//Stelle sicher, das Normalenvektor richtung kamera zeigt (oder alle wegzeigen)
		float faktor=1;
		Eigen::Vector3f normalPlane=getNormalFromCoefficients(*segmentPlaneCoefficients);
		if(normalPlane(2)<0.0)
		{
			faktor=-1.0;
		}
		segmentPlaneCoefficients->values[0]*=faktor;
		segmentPlaneCoefficients->values[1]*=faktor;
		segmentPlaneCoefficients->values[2]*=faktor;
		segmentPlaneCoefficients->values[3]*=faktor;
		normalPlane*=faktor;

		cout << "got normals...\n";

		//rotiere den normalenvektor
		Eigen::Vector3f rotNormalPlane=transformDepth.rotation()*normalPlane;
		rotNormalPlane.normalize();

		cout << "Transformed n: " << rotNormalPlane << endl;

		//berechne stützpunkt
		Eigen::Vector3f supportPoint=-segmentPlaneCoefficients->values[3]*normalPlane;


		//transformiere stützpunkt
		PointT supportPointPCL, supportPointTransfPCL;
		supportPointPCL.x=supportPoint(0);
		supportPointPCL.y=supportPoint(1);
		supportPointPCL.z=supportPoint(2);
		supportPointTransfPCL = pcl::transformPoint(supportPointPCL,transformDepth);


		cout << "got supportpoint...\n";


		//berechne d f??eue ebene
		float dTransf;
		Eigen::Vector3f pTransf;
		pTransf(0)=supportPointTransfPCL.x;
		pTransf(1)=supportPointTransfPCL.y;
		pTransf(2)=supportPointTransfPCL.z;

		dTransf= -pTransf.dot(rotNormalPlane);

		cout << "got d...\n";
		//fitModelToCloud(cloudSegmentTransformed,pcl::SACMODEL_PLANE,pcl::SAC_RANSAC,0.02,segmentPlaneInliers,segmentPlaneCoefficientsTransformed);


		//verwende die rotierten normalenvektoren, nicht die normalenvektoren der rotierten ebenen.

		segmentPlaneCoefficientsTransformed->values.push_back(rotNormalPlane(0));
		segmentPlaneCoefficientsTransformed->values.push_back(rotNormalPlane(1));
		segmentPlaneCoefficientsTransformed->values.push_back(rotNormalPlane(2));
		segmentPlaneCoefficientsTransformed->values.push_back(dTransf);

		cout << "got transformed coeffs...\n";

		//speichere das ebenen modell
		vecSegmentPlaneCoefficientsTransformed.push_back(*segmentPlaneCoefficientsTransformed);

//			fprintf(fileInfo, "%f, %f, %f, %f, ",
//				segmentPlaneCoefficientsTransformed->values[0],
//				segmentPlaneCoefficientsTransformed->values[1],
//				segmentPlaneCoefficientsTransformed->values[2],
//				segmentPlaneCoefficientsTransformed->values[3]);


		segmentPlaneCoefficientsTransformed->values.clear();
	}
	return vecSegmentPlaneCoefficientsTransformed.size();
}

int addHiddenPlanes(Eigen::Affine3f transformDepth, vector<vector<int> >clusters, vector<pcl::ModelCoefficients> vecSegmentPlaneCoefficientsTransformed, vector<pcl::ModelCoefficients> &allPlaneCoefficients, wallType recordedType,vector<PointT> maxXPoints,vector<PointT> minXPoints,std::vector <int> orderOfClusters,vector<float> &qualityOfPlaneEstimation, float hnfs[])
{
	//Fuege verdeckte ebenen hinzu

	if(recordedType==Corners) cout << "EXPECTING A CORNER\n";
	if(recordedType==InfiniteWall) cout << "EXPECTING A WALL\n";
	if(recordedType==FloorPlane) cout << "EXPECTING THE FLOOR\n";
	if(recordedType==CeilingPlane) cout << "EXPECTING THE CEILING\n";

	//Wenn mehr als eine Ebene gefunden wurde
	if((clusters.size()>1) &&(recordedType!=StairBottom) )
	{
		//winkel zwischen zwei aufeinanderfolgenden ebenen berechnen
		int n;
		int nPlanes=0;
		cout << "VecSegPlaneCoeffsTransf.Size : " << vecSegmentPlaneCoefficientsTransformed.size() << endl;
		for(n=1;n<vecSegmentPlaneCoefficientsTransformed.size();n++)
		{
			cout << "run: " << n << endl;
			Eigen::Vector3f normalN, normalM, rotAxis, newNormalVec;
			normalM=getNormalFromCoefficients(vecSegmentPlaneCoefficientsTransformed.at(n-1));
			normalM.normalize();
			normalN=getNormalFromCoefficients(vecSegmentPlaneCoefficientsTransformed.at(n));
			normalN.normalize();

			Eigen::Quaternion<float> myQuat;
			myQuat.setFromTwoVectors(normalN,normalM);

			float angle=(2.0*acos(myQuat.w()))/M_PI*180.0;
			angle=180.0-angleBetweenNormalVecs(normalN,normalM)/M_PI*180.0;

			cout << "Angle between planes: " << angle << endl;


			allPlaneCoefficients.push_back(vecSegmentPlaneCoefficientsTransformed.at(n-1));


			//bei zu stumpfen winkeln f??eine verbindungsebene ein.
			//kann so ausgebaut werden, dass wenn berechneter schnittpunkt der zwei flachen ebenen weit vom ende des ebenenenabschnites wegliegt nur dann neue ebene eingef??wird
			if(angle>170 || angle < 10)
			{
				nPlanes++;

				PCL_WARN("ADDING PLANE\n");
				//setze diese im 90Рwinkel zu erster ebene. (normalenvektor rotiert um y-Achse)
				//drehachse ist der vektor, der senkrecht auf normalM und normalM steht.
				rotAxis=normalM.cross(normalN);

				rotAxis.normalize();
				rotAxis(0)=0.0f;
				rotAxis(1)=0.0f;
				rotAxis(2)=-1.0f;

				Eigen::Matrix3f rotation = (Eigen::Matrix3f)Eigen::AngleAxisf (M_PI/2.0f,rotAxis);

				// und an das rechte ende (maxX) der ersten ebene
				PointT pointOnLeftPlane=maxXPoints.at(orderOfClusters[n-1]);
				PointT pointOnRightPlane=minXPoints.at(orderOfClusters[n]);
				PointT pointOnPlane;
				float quality;


				if(pointOnLeftPlane.z<pointOnRightPlane.z)
				{
					pointOnPlane=pointOnLeftPlane;
					Eigen::Matrix3f rotation = (Eigen::Matrix3f)Eigen::AngleAxisf (-M_PI/2.0f,rotAxis);

					newNormalVec=rotation*normalN;
					newNormalVec.normalize();

					quality=qualityOfPlaneEstimation.at(orderOfClusters[n-1]);

				}
				else
				{
					pointOnPlane=pointOnRightPlane;

					Eigen::Matrix3f rotation = (Eigen::Matrix3f)Eigen::AngleAxisf (M_PI/2.0f,rotAxis);

					newNormalVec=rotation*normalN;
					newNormalVec.normalize();

					quality=qualityOfPlaneEstimation.at(orderOfClusters[n]);

				}

				//füge  neue qualitätsinformationen ein
				std::vector<float>::iterator it;
				it = qualityOfPlaneEstimation.begin() + n;
				qualityOfPlaneEstimation.insert(it,quality);


				PointT pointOnPlaneTransformed;
				pointOnPlaneTransformed = pcl::transformPoint(pointOnPlane,transformDepth);

				//berechne d f??eue ebene
				float d;
				Eigen::Vector3f p;
				p(0)=pointOnPlaneTransformed.x;
				p(1)=pointOnPlaneTransformed.y;
				p(2)=pointOnPlaneTransformed.z;

				d=  -p.dot(newNormalVec);

				//speichere in datei:
				//fprintf(fileAddedPlanesInfo, "%d, %f, %f, %f, %f, <-\n", nPlanes, newNormalVec(0),newNormalVec(1),newNormalVec(2), d);
				pcl::ModelCoefficients tempCoefficients;

				tempCoefficients.values.push_back(newNormalVec(0));
				tempCoefficients.values.push_back(newNormalVec(1));
				tempCoefficients.values.push_back(newNormalVec(2));
				tempCoefficients.values.push_back(d);

				allPlaneCoefficients.push_back(tempCoefficients);

				//break;

			}


			nPlanes++;
		}


		allPlaneCoefficients.push_back(vecSegmentPlaneCoefficientsTransformed.at(n-1));
	}

	if(recordedType==StairBottom)
	{
		for(int n=0;n<vecSegmentPlaneCoefficientsTransformed.size();n++)
		{
			allPlaneCoefficients.push_back(vecSegmentPlaneCoefficientsTransformed.at(n));
		}
	}

	//Wenn nur eine Ebene gefunden wurde, d.h. eine einzelne wand, dann füge die letzte ebene vor diese wand in die ebenen ein
	if(clusters.size()==1 && (recordedType!=FloorPlane || recordedType!=CeilingPlane))
	{

		//füge übergebene ebene an.
		pcl::ModelCoefficients tempCoefficients;


		Eigen::Vector3f normalN;
		normalN=getNormalFromCoefficients(vecSegmentPlaneCoefficientsTransformed.at(0));
		normalN.normalize();

		//if possible:....
		//add the previous plane. we need it to calc intersection with current plane.
		if(hnfs[99]>=0.5)
		{
			//füge übergebene daten hinten an...
			tempCoefficients.values.push_back(hnfs[0]);
			tempCoefficients.values.push_back(hnfs[1]);
			tempCoefficients.values.push_back(hnfs[2]);
			tempCoefficients.values.push_back(hnfs[3]);

			allPlaneCoefficients.push_back(tempCoefficients);

			char temp[100];
			sprintf(temp, "%f, %f, %f, %f", hnfs[0],hnfs[1],hnfs[2],hnfs[3]);
			__android_log_write(ANDROID_LOG_INFO, "HNF In: ", temp);

		}


		//add the new plane
		allPlaneCoefficients.push_back(vecSegmentPlaneCoefficientsTransformed.at(0));

	}

	if(clusters.size()==1 && (recordedType==FloorPlane || recordedType==CeilingPlane))
	{
		Eigen::Vector3f normalN;
		normalN=getNormalFromCoefficients(vecSegmentPlaneCoefficientsTransformed.at(0));
		normalN.normalize();

		pcl::ModelCoefficients tempCoefficients;
		//add the previous plane. matlab skript needs it.
		//tempCoefficients=allPlaneCoefficients.at(allPlaneCoefficients.size()-1);
		if(hnfs[99]>=0.5)
		{
			//füge übergebene daten hinten an...
			tempCoefficients.values.push_back(hnfs[0]);
			tempCoefficients.values.push_back(hnfs[1]);
			tempCoefficients.values.push_back(hnfs[2]);
			tempCoefficients.values.push_back(hnfs[3]);

			allPlaneCoefficients.push_back(tempCoefficients);

		}
		allPlaneCoefficients.push_back(tempCoefficients);

	}

	return allPlaneCoefficients.size();
}

void calcPlaneIntersection(pcl::ModelCoefficients plane1, pcl::ModelCoefficients plane2, Eigen::Vector3f &dirEdgeVec, Eigen::Vector3f &supportVec)
{
	//hole normalenvektoren
	Eigen::Vector3f normalN, normalM;
	normalN=getNormalFromCoefficients(plane1);
	normalN.normalize();
	normalM=getNormalFromCoefficients(plane2);
	normalM.normalize();

	float dN=plane1.values[3];
	float dM=plane2.values[3];

	//berechne richtung der schnittgerade und lasse sie nach oben zeigen
	dirEdgeVec=normalN.cross(normalM);


	if(dirEdgeVec(2)<-0.6)
	{
		dirEdgeVec*=-1.0;
		dirEdgeVec.normalize();
	}

//	//bei nahezu senkrechten, senkrecht setzen.
	if(dirEdgeVec(2)>0.9) dirEdgeVec= Eigen::Vector3f(0.0,0.0,1.0);
//	//überprüfen ob waagerecht geschnitten wird, dann auf wert parallel zur bodenebene setzen.
	if(dirEdgeVec(2)<0.1)
	{
		dirEdgeVec(2)=0.0f;
		dirEdgeVec.normalize();
	}
	cout << "EdgeVec: " << dirEdgeVec << endl;



	dirEdgeVec.normalize();
	//berechne stützpunkt der schnittgerade mit z=0:

	Eigen::Matrix2f A;
	A(0,0)=normalN(0);
	A(0,1)=normalN(1);
	A(1,0)=normalM(0);
	A(1,1)=normalM(1);
	cout << "A:\n" << A << endl;
	Eigen::Vector2f supportVecXY;
	Eigen::Vector2f temp;
	temp(0)=dN;
	temp(1)=dM;

	supportVecXY=A.inverse()*temp;

	supportVec(0)=supportVecXY(0);
	supportVec(1)=supportVecXY(1);
	supportVec(2)=0;
}

void setReturnValues(wallType recordedType, vector<PointT> wallCorners, vector<pcl::ModelCoefficients> allPlaneCoefficients, int numPlaneSegments, float cornerPoints[], float hnfs[])
{

	for(int n=0;n<4;n++)
	{
		char temp2[200];
		sprintf(temp2, "Point %d XYZ: %f, %f, %f",n, cornerPoints[n*3+0],cornerPoints[n*3+1],cornerPoints[n*3+2]);
		__android_log_write(ANDROID_LOG_INFO, "PointsIn", temp2);
	}

	if(recordedType==LimitedWall)
		{
			//verbinde anfang des startsegments mit übergebenen eckpunkten. spare keine ecke aus.

			//1. Erzeuge am anfang künstliche ebene, von letzen Endpunkten bis zu neuen startpunkten

			//-> schreibe an erste stelle die endpunkte des letzten segments der vorigen aufnahme
			int startIndex=0;

			//Da die punkte verkehrt rum kommen: vertausche Sie.
			float tempA, tempB, tempC;


			cornerPoints[0]=cornerPoints[6];
			cornerPoints[1]=cornerPoints[7];
			cornerPoints[2]=cornerPoints[8];

			cornerPoints[3]=cornerPoints[9];
			cornerPoints[4]=cornerPoints[10];
			cornerPoints[5]=cornerPoints[11];


			startIndex=2;
			//-> füge anfang des ersten segments der aktuellen aufnahme an.
			//dies ist die verbindende Ebene zwischen aktueller und letzer aufnahme.
			for(int k=startIndex;k< (startIndex+2);k++)
			{
				cornerPoints[k*3+0]= wallCorners[k-startIndex].x;
				cornerPoints[k*3+1]= wallCorners[k-startIndex].y;
				cornerPoints[k*3+2]= wallCorners[k-startIndex].z;
				__android_log_write(ANDROID_LOG_INFO, "Tag", "2");
			}

			//jetzt die neuen messungen einbringen.
			startIndex=4;

			//3. füge die jetzt erkannten eckpunkte an
			for(int k=startIndex;k<(wallCorners.size()+startIndex);k++)
			{
				cornerPoints[k*3+0]= wallCorners[k-startIndex].x;
				cornerPoints[k*3+1]= wallCorners[k-startIndex].y;
				cornerPoints[k*3+2]= wallCorners[k-startIndex].z;
				__android_log_write(ANDROID_LOG_INFO, "Tag", "3");
			}


			numPlaneSegments++;
			cornerPoints[99]=numPlaneSegments;


			//TODO: HNFs zurückgeben.
			Eigen::Vector4f hnfAddedWall=calcHNFForEdgePoints(cornerPoints,0);
			hnfs[0]=hnfAddedWall(0); hnfs[1]=hnfAddedWall(1); hnfs[2]=hnfAddedWall(2); hnfs[3]=hnfAddedWall(3);

			//Befuelle array mit HNFs
			for(int k=0;k<numPlaneSegments-1;k++)
			{
				hnfs[(k+1)*4+0]=allPlaneCoefficients[k].values[0];
				hnfs[(k+1)*4+1]=allPlaneCoefficients[k].values[1];
				hnfs[(k+1)*4+2]=allPlaneCoefficients[k].values[2];
				hnfs[(k+1)*4+3]=allPlaneCoefficients[k].values[3];
				char temp[100];
				sprintf(temp, "%f, %f, %f, %f", hnfs[k*4+0],hnfs[k*4+1],hnfs[k*4+2],hnfs[k*4+3]);
				__android_log_write(ANDROID_LOG_INFO, "HNF Out: ", temp);
			}

		}

	if(recordedType == Corners)
		{
			//Verbinde letzte ecke mit erster ecke in aktueller aufnahme.

			//Wenn Eckpunkte von voriger Ecke geliefert werden dann verbinde die neue ecke damit.
			int startIndex=0;
			if(cornerPoints[99]>=2)
			{
				//behalte übergebene zwei eckpunkte...
				//...
				//und füge neu gefundene eckpunkte dahinter an
				startIndex=2;

//				cornerPoints[0]=cornerPoints[9];
//				cornerPoints[1]=cornerPoints[10];
//				cornerPoints[2]=cornerPoints[11];
//				cornerPoints[3]=cornerPoints[6];
//				cornerPoints[4]=cornerPoints[7];
//				cornerPoints[5]=cornerPoints[8];

				__android_log_write(ANDROID_LOG_INFO, "Tag", "Corner[99]=2");//Or ANDROID_LOG_ERROR, ...
			}

			//Befuelle array mit eckpunkten
			for(int k=startIndex;k<wallCorners.size();k++)
			{
				cout << wallCorners[k].x << " " << wallCorners[k].y << " " << wallCorners[k].z << "\n";
				cornerPoints[k*3+0]=wallCorners[k].x;
				cornerPoints[k*3+1]=wallCorners[k].y;
				cornerPoints[k*3+2]=wallCorners[k].z;
				__android_log_write(ANDROID_LOG_INFO, "Tag", "Speichere Eckpunkt");//Or ANDROID_LOG_ERROR, ...

			}


			cornerPoints[99]=numPlaneSegments;

			//Befuelle array mit HNFs
			for(int k=0;k<numPlaneSegments;k++)
			{
				hnfs[k*4+0]=allPlaneCoefficients[k].values[0];
				hnfs[k*4+1]=allPlaneCoefficients[k].values[1];
				hnfs[k*4+2]=allPlaneCoefficients[k].values[2];
				hnfs[k*4+3]=allPlaneCoefficients[k].values[3];
				char temp[100];
				sprintf(temp, "%f, %f, %f, %f", hnfs[k*4+0],hnfs[k*4+1],hnfs[k*4+2],hnfs[k*4+3]);
				__android_log_write(ANDROID_LOG_INFO, "HNF Out: ", temp);

			}
			hnfs[99]=numPlaneSegments;
		}

	if(recordedType== StairBottom)
	{
		//Befuelle array mit eckpunkten
		for(int k=0;k<wallCorners.size();k++)
		{
			cornerPoints[k*3+0]=wallCorners[k].x;
			cornerPoints[k*3+1]=wallCorners[k].y;
			cornerPoints[k*3+2]=wallCorners[k].z;
		}
		cornerPoints[99]=numPlaneSegments;

		//TODO: HNFs setzen.
	}



}

void getTypeOfPointCloud(vector<pcl::ModelCoefficients> vecSegmentPlaneCoefficientsTransformed,vector<vector<int> >clusters, pcl::PointCloud<PointT>::Ptr cloud, Eigen::Affine3f transformDepth, std::vector <int> orderOfClusters, wallType &recordedType, float &floorLevel, float &roomHeight)
{
	//nur, wenn boden/decke oder Treppe erfasst wurde, wird recordedType verändert.

	__android_log_write(ANDROID_LOG_INFO, "Test", "Floor/Ceiling");
		//Überprüfe ob boden oder decke erfasst wurde
		if(clusters.size()==1)
		{
			if(vecSegmentPlaneCoefficientsTransformed.at(0).values[2]<-0.9)
			{
				recordedType=FloorPlane;
				floorLevel=vecSegmentPlaneCoefficientsTransformed.at(0).values[3];

				char temp2[200];
				sprintf(temp2, "FloorLevel: %f",floorLevel);
				__android_log_write(ANDROID_LOG_INFO, "Floor", temp2);
			}
			if(vecSegmentPlaneCoefficientsTransformed.at(0).values[2]>0.9)
			{
				recordedType=CeilingPlane;
				//berechne abstand zu voriger ebene (dies war der Boden)
				roomHeight= -(vecSegmentPlaneCoefficientsTransformed.at(0).values[3]+floorLevel);
				__android_log_write(ANDROID_LOG_INFO, "Type", "Ceiling");
				char temp[200];
				sprintf(temp, "Ceiling: Level %f  Roomheight: %f",vecSegmentPlaneCoefficientsTransformed.at(0).values[3], roomHeight);
				__android_log_write(ANDROID_LOG_INFO, "Ceiling:", temp);
			}
		}

		__android_log_write(ANDROID_LOG_INFO, "Test", "Stair");
		//Überprüfe ob Treppe erfasst wurde
		if(clusters.size()>=2)
		{
			int numPlanesParallelToFloor=0;
			for(int n=0;n<vecSegmentPlaneCoefficientsTransformed.size();n++)
			{
				if(vecSegmentPlaneCoefficientsTransformed.at(n).values[2]<-0.9)
				{
					numPlanesParallelToFloor++;
				}
			}
			if(numPlanesParallelToFloor>=2)
			{
				recordedType=StairBottom;
				//Sortiere neu nach Y...
				//TODO: nicht neu die ebenen suchen, sondern die numerierung ändern
				vector<float> qualityOfPlaneEstimation;
				int res=findHNFForClusters(cloud, transformDepth, clusters, orderOfClusters, vecSegmentPlaneCoefficientsTransformed, qualityOfPlaneEstimation);
				floorLevel=vecSegmentPlaneCoefficientsTransformed.at(0).values[3];
				roomHeight= -(vecSegmentPlaneCoefficientsTransformed.at(2).values[3]+floorLevel);
				__android_log_write(ANDROID_LOG_INFO, "Type", "StairBottom");

			}
		}

}

Eigen::Vector4f calcHNFForEdgePoints(float cornerPoints[], int startIndex)
{
	Eigen::Vector4f myHNF;
	//die zwei richtungsvektoren
	Eigen::Vector3f a, b, n;
	int index=startIndex;

	//Setze richtungsvektor 1
	a(0)=(cornerPoints[startIndex]-cornerPoints[startIndex +3]);
	startIndex++;
	a(1)=(cornerPoints[startIndex]-cornerPoints[startIndex +3]);
	startIndex++;
	a(2)=cornerPoints[startIndex]-cornerPoints[startIndex +3];

	//Setze richtungsvektor 2
	startIndex=index;
	b(0)=(cornerPoints[startIndex]-cornerPoints[startIndex +6]);
	startIndex++;
	b(1)=(cornerPoints[startIndex]-cornerPoints[startIndex +6]);
	startIndex++;
	b(2)=cornerPoints[startIndex]-cornerPoints[startIndex +6];

	//Berechne n:
	a.normalize();
	b.normalize();

	n=a.cross(b);
	n.normalize();


	startIndex=index;
	//Berechne d: n*x-d=0;
	float d;
	startIndex=index;
	d=(n(0)*cornerPoints[startIndex+0] + n(1)*cornerPoints[startIndex+1] + n(2)*cornerPoints[startIndex+2]);

	//setze rückgabewert
	myHNF(0)=n(0);
	myHNF(1)=n(1);
	myHNF(2)=n(2);
	myHNF(3)=d;

	return myHNF;
}

