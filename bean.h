#pragma once
#include <iostream>
#include <fstream>
#include <vector>
using namespace std;
//各种基础结构体或者类
#define PI 3.141592653579
struct Point3d
{
	
	double x, y, z;
	Point3d(double x,double y,double z)
	{this->x=x;
	this->y=y;
	this->z=z;
	}
	Point3d(){}
	Point3d &operator-(Point3d &v1)
	{
	 x-=v1.x;
	 y-=v1.y;
	 z-=v1.z;
	 return *this;
	}

	bool operator<(const Point3d &v1)const
	{
		return this->z<v1.z;
	}
	
};
struct support_line_node
{
   Point3d a,b;
   int flaga,flagb;
};
//是否和mesh相连
struct Point3d_mesh
{
	Point3d point;
	int flag;
};
//存面
struct Point3i
{
	int x, y, z;
	Point3i()
	{}
	Point3i(int x,int y,int z)
	{
	this->x=x;
	this->y=y;
	this->z=z;
	}
};
struct Point2d
{
	double x, y;
	Point2d() {}
	Point2d(double x, double y)
	{
		this->x = x;
		this->y = y;
	}
	Point2d friend operator -(Point2d a, Point2d b)
	{
		return Point2d(a.x - b.x, a.y - b.y);
	}
};
