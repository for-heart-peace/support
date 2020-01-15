#pragma once
#include <iostream>
#include <map>
#include "bean.h"
#include "base_model.h"
#include <string>
#include <time.h>

class support_point
{
public:
	double dis_x;
	double dis_y;
	double min_z;
	double min_radius;
	double max_radius;
	double top;
	double support_angle;
	double to_mesh_angle;
	double cone_length;
	double bottom_radius;
	double bottom_height;
	double bottom_down;//底座下降的距离
	double min_x, min_y, max_x, max_y;
	string file_name;
	vector<Point3d>support_point_list;

	//读off文件,把点云存在point，面集存在face。off文件点集从0开始
	bool readoff(string filename);
	bool print_off(string filename,vector<Point3d>point1,vector<Point3i>face1);
	Point3d normal(Point3i & now_face);
	void GroupMarkedFaces();        //合并相邻三角面
	void GetSupportPoint();         //得到所有需要加支撑的点
	bool GetInterSecPointRegion(Point3d samplePoint,Point3d &resultPoint,Point3i FaceList);//投影到模型上的交点
	bool SameSide(Point3d A, Point3d B, Point3d C, Point3d P);          
	bool IsPointinTriangle1(Point3d A, Point3d B, Point3d C, Point3d P);//点是否在三角形内
	void GenerateSupport();                                             //找到所有的线
	void GenerateSupportModel();										//生成.off文件
	Point3d calculateTheIntersectionOfTwoCone(const Point3d &A,const Point3d & B,double alpha);
	void bottom_base();
private :
	vector<Point3d>support_off_point;
	vector<Point3i>support_off_face;
	vector<support_line_node>support_line;
	vector<Point3d>point;
	vector<Point3i>face;
	vector<vector<int>>edge;					//边，判断点之间的连接
	vector<int>visit;							//是否访问过该点
	vector<int>MakeFaceList;					//大角度面的列表
	map<pair<int, int>,pair<int, int>>edge_face;	//边和面的对应关系
	vector<vector<int>>face_face;					//面和面之间的连接关系
	vector<int>visit_face;							//每个面合并到那个堆
	vector<vector<int>>cluster_face;						//提取出每个堆的面
	base_model model;
	vector<Point2d>point_2d;
};

