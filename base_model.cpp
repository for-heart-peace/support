#include "StdAfx.h"
#include "base_model.h"

base_model::base_model(void)
{
}


base_model::~base_model(void)
{
}

void  base_model::creatCylinder(int edge,double radiusTop,double radiusBottom,Point3d topPoint,Point3d bottomPoint){
	cylinderPoint.clear();
	cylinderFace.clear();
	//顶面
	for(int i=0;i<edge;i++)
	{
		double x=radiusTop*cos(PI*2/edge*i)+topPoint.x;
		double y=radiusTop*sin(PI*2/edge*i)+topPoint.y;
		double z=topPoint.z;
		cylinderPoint.push_back(Point3d(x,y,z));
		cylinderFace.push_back(Point3i(i,(i+1)%edge,2*edge));
	}
	//底面
	for(int i=0;i<edge;i++)
	{
		double x=radiusBottom*cos(PI*2/edge*i)+bottomPoint.x;
		double y=radiusBottom*sin(PI*2/edge*i)+bottomPoint.y;
		double z=bottomPoint.z;
		cylinderPoint.push_back(Point3d(x,y,z));
		cylinderFace.push_back(Point3i(i+edge,(i-1+edge)%edge+edge,2*edge+1));
	}
	cylinderPoint.push_back(topPoint);
	cylinderPoint.push_back(bottomPoint);
	//侧面
	for(int i=0;i<edge;i++)
	{
		cylinderFace.push_back(Point3i((i+1)%edge,i,edge+i));
		cylinderFace.push_back(Point3i(i+edge,(i+1)%edge+edge,(i+1)%edge));
	}

}
void base_model::creatCone(int edge,double radius,Point3d topPoint,Point3d bottomPoint)
{
	conePoint.clear();
	coneFace.clear();
	//底面
	for(int i=0;i<edge;i++)
	{
		double x=radius*cos(PI*2/edge*i)+bottomPoint.x;
		double y=radius*sin(PI*2/edge*i)+bottomPoint.y;
		double z=bottomPoint.z;
		conePoint.push_back(Point3d(x,y,z));
		coneFace.push_back(Point3i(i,(i-1+edge)%edge,edge+1));
		coneFace.push_back(Point3i(i,(i+1)%edge,edge));
	}
	conePoint.push_back(topPoint);
	conePoint.push_back(bottomPoint);

}
void base_model::creatReverseCone(int edge, double radius, Point3d topPoint, Point3d bottomPoint)
{
	conePoint.clear();
	coneFace.clear();
	//底面
	for (int i = 0; i<edge; i++)
	{
		double x = radius*cos(PI * 2 / edge*i) + bottomPoint.x;
		double y = radius*sin(PI * 2 / edge*i) + bottomPoint.y;
		double z = bottomPoint.z;
		conePoint.push_back(Point3d(x, y, z));
		coneFace.push_back(Point3i(i, (i + 1 + edge) % edge, edge + 1));
		coneFace.push_back(Point3i((i + 1) % edge, i, edge));
	}
	conePoint.push_back(topPoint);
	conePoint.push_back(bottomPoint);
}

void base_model::creatBase(double minx, double maxx, double miny, double maxy, double minz, double height)
{
	vector<Point2d>a;
	a.push_back(Point2d(minx, miny));
	a.push_back(Point2d(maxx, miny));
	a.push_back(Point2d(maxx, maxy));
	a.push_back(Point2d(minx, maxy));
	int size = a.size();
	bottomPoint.clear();
	bottomFace.clear();
	Point3d middle = Point3d(minx + 0.5 + maxx*0.5, miny + 0.5 + maxy*0.5, minz);
	Point3d bottomMiddle = Point3d(minx + 0.5 + maxx*0.5, miny + 0.5 + maxy*0.5, minz - height);
	//顶面
	for (int i = 0; i<size; i++)
	{
		bottomPoint.push_back(Point3d(a[i].x, a[i].y, minz));
		bottomFace.push_back(Point3i(i, (i + 1) % size, 2 * size));
	}
	//底面
	for (int i = 0; i<size; i++)
	{
		bottomPoint.push_back(Point3d(a[i].x, a[i].y, minz - height));
		bottomFace.push_back(Point3i(i + size, (i - 1 + size) % size + size, 2 * size + 1));
	}
	bottomPoint.push_back(middle);
	bottomPoint.push_back(bottomMiddle);
	//侧面
	for (int i = 0; i<size; i++)
	{
		bottomFace.push_back(Point3i((i + 1) % size, i, size + i));
		bottomFace.push_back(Point3i(i + size, (i + 1) % size + size, (i + 1) % size));
	}

}
void base_model::creatBase(vector<Point2d>&a, double minz, double height)
{
	int size = a.size();
	bottomPoint.clear();
	bottomFace.clear();
	Point3d middle = Point3d(a[0].x*0.5 + a[size / 2].x*0.5, a[0].y*0.5 + a[size / 2].y*0.5, minz);
	Point3d bottomMiddle = Point3d(a[0].x*0.5 + a[size / 2].x*0.5, a[0].y*0.5 + a[size / 2].y*0.5, minz - height);
	//顶面
	for (int i = 0; i<size; i++)
	{
		bottomPoint.push_back(Point3d(a[i].x, a[i].y, minz));
		bottomFace.push_back(Point3i(i, (i + 1) % size, 2 * size));
	}
	//底面
	for (int i = 0; i<size; i++)
	{
		bottomPoint.push_back(Point3d(a[i].x, a[i].y, minz - height));
		bottomFace.push_back(Point3i(i + size, (i - 1 + size) % size + size, 2 * size + 1));
	}
	bottomPoint.push_back(middle);
	bottomPoint.push_back(bottomMiddle);
	//侧面
	for (int i = 0; i<size; i++)
	{
		bottomFace.push_back(Point3i((i + 1) % size, i, size + i));
		bottomFace.push_back(Point3i(i + size, (i + 1) % size + size, (i + 1) % size));
	}
}
