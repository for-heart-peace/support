#pragma once
#include "bean.h"
class base_model
{
public:
	 base_model(void);
	~base_model(void);
	void creatCylinder(int edge,double radiusTop,double radiusBottom,Point3d topPoint,Point3d bottomPoint);
	void creatCone(int edge,double radius,Point3d topPoint,Point3d bottomPoint);
	void creatReverseCone(int edge, double radius, Point3d topPoint, Point3d bottomPoint);
	void creatBase(double minx, double maxx, double miny, double maxy, double minz, double height);
	void creatBase(vector<Point2d>&a, double minz, double height);
	vector<Point3d>cylinderPoint;
	vector<Point3i>cylinderFace;
	vector<Point3d>conePoint;
	vector<Point3i>coneFace;
	vector<Point3d>bottomPoint;
	vector<Point3i>bottomFace;
};

