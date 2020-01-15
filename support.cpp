#include "support.h"
#include <iostream>
#include <algorithm>
#include <queue>
#include <set>
#include <fstream>
#include <list>
#include <CGAL/Surface_mesh.h>
#include <CGAL/Simple_cartesian.h>
#include <CGAL/AABB_tree.h>
#include <CGAL/AABB_traits.h>
#include <CGAL/Polyhedron_3.h>
#include <CGAL/AABB_face_graph_triangle_primitive.h>
typedef CGAL::Simple_cartesian<double> K;
typedef K::Point_3 Point;
typedef K::Plane_3 Plane;
typedef K::Vector_3 Vector;
typedef K::Segment_3 Segment;
typedef K::Ray_3 Ray;
typedef CGAL::Surface_mesh<Point> Mesh;
typedef CGAL::Polyhedron_3<K> Polyhedron;
typedef CGAL::AABB_face_graph_triangle_primitive<Polyhedron> Primitive;
typedef CGAL::AABB_traits<K, Primitive> Traits;
typedef CGAL::AABB_tree<Traits> Tree;
typedef boost::optional< Tree::Intersection_and_primitive_id<Segment>::Type > Segment_intersection;
typedef boost::optional< Tree::Intersection_and_primitive_id<Ray>::Type > Ray_intersection;
typedef boost::optional< Tree::Intersection_and_primitive_id<Plane>::Type > Plane_intersection;
typedef Tree::Primitive_id Primitive_id;

bool support_point::readoff(string filename)
{
	FILE* FI;
	FI = fopen(filename.c_str(), "r");
	if (FI == NULL)
		return false;
	string s;
	int n, m, k;
	fscanf(FI, "%s", s);
	//printf("%s",s);
	fscanf(FI, "%d%d%d", &n, &m, &k);
	Point3d temp;
	visit.clear();
	edge.clear();
	int flag = 0;
	if (min_z>10000.0)
		flag = 1;
	//vector<int> edge_temp;
	edge.reserve(n);
	vector<int> edge_temp;
	for (int i = 0; i < n; i++)
	{
		visit.push_back(0);//该点没访问过
		edge.push_back(edge_temp);
		fscanf(FI, "%lf%lf%lf", &temp.x, &temp.y, &temp.z);
		max_x = max(max_x, temp.x);
		max_y = max(max_y, temp.y);
		min_x = min(min_x, temp.x);
		min_y = min(min_y, temp.y);
		if (flag == 1)
			min_z = min(min_z, temp.z);
		//fscanf(FI, "%lf%lf%lf", &temp[0], &temp[1], &temp[2]);
		point.push_back(temp);
	}

	Point3i temp1;
	int i1;
	edge_face.clear();
	face_face.clear();
	face_face.reserve(m);
	for (int i = 0; i < m; i++)
	{
		face_face.push_back(edge_temp);
		visit_face.push_back(0);
		fscanf(FI, "%d", &i1);
		fscanf(FI, "%d%d%d", &temp1.x, &temp1.y, &temp1.z);
		edge[temp1.x].push_back(temp1.y);
		edge[temp1.x].push_back(temp1.z);
		edge[temp1.y].push_back(temp1.x);
		edge[temp1.y].push_back(temp1.z);
		edge[temp1.z].push_back(temp1.y);
		edge[temp1.z].push_back(temp1.x);
		face.push_back(temp1);
		int a[3];
		a[0] = temp1.x;
		a[1] = temp1.y;
		a[2] = temp1.z;
		sort(a, a + 3);
		for (int j = 0; j <= 2; j++)
		{
			pair<int, int>pair1;
			if (j != 2)
			{
				pair1.first = a[j];
				pair1.second = a[j + 1];
			}
			else
			{
				pair1.first = a[0];
				pair1.second = a[2];
			}
			if (edge_face.count(pair1) == 0)
			{
				edge_face[pair1] = make_pair(i, -1);
			}
			else
			{
				pair<int, int>pair2;
				pair2 = edge_face[pair1];
				pair2.second = i;
				edge_face[pair1] = pair2;
				face_face[pair2.first].push_back(pair2.second);
				face_face[pair2.second].push_back(pair2.first);
			}
		}
	}

	return true;
}

bool support_point::print_off(string filename, vector<Point3d>point1, vector<Point3i>face1)
{
	ofstream f1(filename);
	f1 << "OFF" << endl;
	f1 << point1.size() << " " << face1.size() << " 0" << endl;
	for (int i = 0; i < point1.size(); i++)
	{
		f1 << point1[i].x << " " << point1[i].y << " " << point1[i].z << endl;
	}
	for (int i = 0; i < face1.size(); i++)
	{
		f1 << 3 << " ";
		f1 << face1[i].x << " " << face1[i].y << " " << face1[i].z << endl;
		f1 << endl;
	}
	f1.close();
	return true;
}


Point3d support_point::normal(Point3i & now_face)
{
	//平面方程: na * (x – n1) + nb * (y – n2) + nc * (z – n3) = 0 ;
	Point3d v1, v2, v3;
	v1 = point[now_face.x];
	v2 = point[now_face.y];
	v3 = point[now_face.z];
	double na = (v2.y - v1.y)*(v3.z - v1.z) - (v2.z - v1.z)*(v3.y - v1.y);
	double nb = (v2.z - v1.z)*(v3.x - v1.x) - (v2.x - v1.x)*(v3.z - v1.z);
	double nc = (v2.x - v1.x)*(v3.y - v1.y) - (v2.y - v1.y)*(v3.x - v1.x);
	double k = sqrt(na*na + nb*nb + nc*nc);
	na /= k;
	nb /= k;
	nc /= k;
	return Point3d(na, nb, nc);

}


void support_point::GroupMarkedFaces()
{
	if (point.size() == 0)return;
	MakeFaceList.clear();
	Point3d  Zaxis = Point3d(0, 0, 1);
	//获取到所有的大角度面
	for (int i = 0; i < face.size(); i++)
	{
		Point3d face_normal = normal(face[i]);
		double angle = face_normal.x*Zaxis.x + face_normal.y*Zaxis.y + face_normal.z*Zaxis.z;
		angle = acos(angle);
		//cout<<fabs(angle)<<" "<<(180.0-support_angle)*PI/180.0<<endl;
		if (fabs(angle)>(180.0 - support_angle)*PI / 180.0)//下面的面,且不符合要求
		{
			MakeFaceList.push_back(i);
			visit_face[i] = 1;
		}
	}
	//把相邻面聚类到cluster_face
	int cnt = 2;
	vector<int>cluster_temp;
	int sum = 0;
	for (int i = 0; i<MakeFaceList.size(); i++)
	{
		if (visit_face[MakeFaceList[i]] != 1)continue;
		cluster_temp.clear();
		queue<int>q;
		q.push(MakeFaceList[i]);
		visit_face[MakeFaceList[i]] = cnt;
		while (!q.empty())
		{
			int p = q.front();
			cluster_temp.push_back(p);
			q.pop();
			for (int j = 0; j<face_face[p].size(); j++)
			{
				if (visit_face[face_face[p][j]] == 1)
				{
					visit_face[face_face[p][j]] = cnt;
					q.push(face_face[p][j]);
				}
			}
		}
		cluster_face.push_back(cluster_temp);
		sum += cluster_temp.size();
		cnt++;
	}
	cout << "初试大角度面的个数：" << sum << endl;
	cout << "初试大角度面的个数：" << MakeFaceList.size() << endl;
	cout << "合并后面的个数：" << cluster_face.size() << endl;
	//print_off("mesh.off",point,MakeFaceList);

}


bool face_point_cmp(const Point3d &a, const Point3d &b)
{
	if (a.x == b.x)
	{
		if (a.y == b.y)
			return a.z<b.z;
		else return a.y<b.y;
	}
	else return a.x<b.x;
}



void support_point::GetSupportPoint()
{
	support_point_list.clear();
	//最低点获取
	for (int i = 0; i < point.size(); i++)
	{
		if (visit[i])continue;
		visit[i] = 1;
		int flag = -1;//判断是否是局部极小值
		double now_minz = point[i].z;
		for (int j = 0; j < edge[i].size(); j++)
		{
			visit[edge[i][j]] = 1;
			if (now_minz > point[edge[i][j]].z)
			{
				flag = edge[i][j];
				now_minz = point[edge[i][j]].z;
			}
		}
		if (flag != -1)visit[flag] = 0;

		if (flag == -1)
		{
			support_point_list.push_back(point[i]);
		}
	}
	cout << support_point_list.size() << endl;
	vector<Point3i>face1;
	face1.clear();
	//print_off("onlypoint.off",support_point_list,face1);
	//system("pause");
	vector<Point3d>face_point_list;
	vector<Point3d>face2d_point;
	//找出大角度的面
	GroupMarkedFaces();
	for (int i = 0; i<cluster_face.size(); i++)
	{
		face_point_list.clear();
		face2d_point.clear();
		double minx = 999999.0, miny = 999999.0, maxx = -999999.0, maxy = -999999.0;
		//每个面的每个点的x,y值,这里每个点都会重复计算几次，增加了复杂度，影响不大
		for (int j = 0; j<cluster_face[i].size(); j++)
		{
			minx = min(minx, point[face[cluster_face[i][j]].x].x);
			miny = min(miny, point[face[cluster_face[i][j]].x].y);
			maxx = max(maxx, point[face[cluster_face[i][j]].x].x);
			maxy = max(maxy, point[face[cluster_face[i][j]].x].y);
			minx = min(minx, point[face[cluster_face[i][j]].y].x);
			miny = min(miny, point[face[cluster_face[i][j]].y].y);
			maxx = max(maxx, point[face[cluster_face[i][j]].y].x);
			maxy = max(maxy, point[face[cluster_face[i][j]].y].y);
			minx = min(minx, point[face[cluster_face[i][j]].z].x);
			miny = min(miny, point[face[cluster_face[i][j]].z].y);
			maxx = max(maxx, point[face[cluster_face[i][j]].z].x);
			maxy = max(maxy, point[face[cluster_face[i][j]].z].y);
		}
		int x_point = floor((maxx - minx) / dis_x);//x轴方向的点个数
		int y_point = floor((maxy - miny) / dis_y);//y轴方向的点个数

		if (x_point == 0 || y_point == 0)continue;

		/*if(x_point==0)
		{
		double now_x=(maxx+minx)/2;
		Point3d temp;
		temp.x=now_x;
		temp.z=0;
		double nowy=((maxy-miny)-(y_point-1)*dis_y)/2+miny;
		for(int j=0;j<y_point;j++)
		{
		temp.y=nowy+j*dis_y;
		face2d_point.push_back(temp);
		}
		}
		else if(y_point==0)
		{
		double now_y=(maxy+miny)/2;
		Point3d temp;
		temp.y=now_y;
		double nowx=((maxx-minx)-(x_point-1)*dis_x)/2+minx;
		for(int j=0;j<x_point;j++)
		{
		temp.x=nowx+j*dis_x;
		face2d_point.push_back(temp);
		}
		}*/
		else
		{
			Point3d temp;
			temp.z = 0;
			double nowx = ((maxx - minx) - (x_point - 1)*dis_x) / 2 + minx;
			double nowy = ((maxy - miny) - (y_point - 1)*dis_y) / 2 + miny;
			for (int j = 0; j<x_point; j++)
				for (int k = 0; k<y_point; k++)
				{
					temp.x = nowx + j*dis_x;
					temp.y = nowy + k*dis_y;
					face2d_point.push_back(temp);
				}
		}

		//把待支撑点映射到模型
		for (int j = 0; j<cluster_face[i].size(); j++)
		{
			for (int k = 0; k<face2d_point.size(); k++)
			{
				Point3d result_point;
				if (GetInterSecPointRegion(face2d_point[k], result_point, face[cluster_face[i][j]]) == true)
				{
					face_point_list.push_back(result_point);
				}
			}
		}
		if (face_point_list.size() == 0)continue;
		//去掉重复的点，支撑点在两个面的交界处情况,这里可以排序把复杂度降低为nlgn
		sort(face_point_list.begin(), face_point_list.end(), face_point_cmp);
		support_point_list.push_back(face_point_list[0]);
		for (int j = 1; j<face_point_list.size(); j++)
		{
			if (fabs(face_point_list[j].x - face_point_list[j - 1].x)<1e-3&&fabs(face_point_list[j].y - face_point_list[j - 1].y)<1e-3
				&&fabs(face_point_list[j].z - face_point_list[j - 1].z)<1e-3)
				continue;
			else
				support_point_list.push_back(face_point_list[j]);
		}
	}
	cout << "最终的需要加支撑的点" << support_point_list.size() << endl;

	//print_off("point.off",support_point_list,face1);
	GenerateSupport();
	//找出悬吊边
}


Point3d crossProduct(Point3d a, Point3d b)
{
	return Point3d(a.y*b.z - a.z*b.y, a.z*b.x - a.x*b.z, a.x*b.y - a.y*b.x);
}


double dotProduct(Point3d a, Point3d b)
{
	return a.x*b.x + a.y*b.y + a.z*b.z;
}


bool support_point::SameSide(Point3d A, Point3d B, Point3d C, Point3d P)
{
	Point3d AB = B - A;
	Point3d AC = C - A;
	Point3d AP = P - A;

	Point3d v1 = crossProduct(AB, AC);
	Point3d v2 = crossProduct(AB, AP);

	// v1 and v2 should point to the same direction
	return dotProduct(v1, v2) >= 0;
}



// Same side method
// Determine whether point P in triangle ABC
bool support_point::IsPointinTriangle1(Point3d A, Point3d B, Point3d C, Point3d P)
{
	return SameSide(A, B, C, P) &&
		SameSide(B, C, A, P) &&
		SameSide(C, A, B, P);
}



bool support_point::GetInterSecPointRegion(Point3d samplePoint, Point3d &resultPoint, Point3i FaceList)
{

	vector<Point3d> lowest_point;
	//now get the equation of the triangle 
	//将平面方程写成点法式方程形式，即有：
	//vp1 * (x – n1) + vp2 * (y – n2) + vp3 * (z – n3) = 0 
	//Point3d samplePoint=newReg->samplePointList[ii];//the through point of the line 
	Point3d lineNormal = Point3d(0, 0, 1);
	//get the equation of the line
	//将直线方程写成参数方程形式，即有：
	//	x = m1+ v1 * t
	//	y = m2+ v2 * t                                                    (1)
	//	z = m3+ v3 * t
	double m1 = samplePoint.x;
	double m2 = samplePoint.y;
	double m3 = samplePoint.z;
	double v1 = lineNormal.x;
	double v2 = lineNormal.y;
	double v3 = lineNormal.z;


	double n1 = point[FaceList.x].x;
	double n2 = point[FaceList.x].y;
	double n3 = point[FaceList.x].z;
	Point3d faceNormal;
	faceNormal = normal(FaceList);
	double vp1 = faceNormal.x;
	double vp2 = faceNormal.y;
	double vp3 = faceNormal.z;

	//联立直线方程和平面方程得到t
	double t;
	if (fabs(vp1* v1 + vp2* v2 + vp3* v3)<1e-4)//说明直线和平面水平
	{
		//no intersection
		//do nothing
	}
	else
	{
		t = ((n1 - m1)*vp1 + (n2 - m2)*vp2 + (n3 - m3)*vp3) / (vp1* v1 + vp2* v2 + vp3* v3);
		//get the section
		double x = m1 + v1 * t;
		double y = m2 + v2 * t;
		double z = m3 + v3 * t;
		resultPoint = Point3d(x, y, z);
		//judge the intersection in triangle
		Point3d A = point[FaceList.x];
		Point3d B = point[FaceList.y];
		Point3d C = point[FaceList.z];
		Point3d P = resultPoint;
		bool isInTri = IsPointinTriangle1(A, B, C, P);
		if (isInTri == true)
		{
			//supportPointList.push_back(interSection);
			return true;
			//可能出现多余采样点的情况，就是在一个面里面包含多个采样点
		}
	}
	return false;
}


bool Z_cmp(const Point3d_mesh& a, const Point3d_mesh& b)
{
	return a.point.z>b.point.z;
}

//两点间的距离
double dis_pp(Point3d a, Point3d b)
{
	return sqrt(pow(a.x - b.x, 2) + pow(a.y - b.y, 2) + pow(a.z - b.z, 2));
}
void Point2Point3d(Point3d& a, Point b)
{
	a.x = b.x();
	a.y = b.y();
	a.z = b.z();
}

//加入和mesh的交点时候要注意改flag
void support_point::GenerateSupport()
{
	const char* filename = file_name.c_str();
	std::ifstream input(filename);
	Polyhedron polyhedron;
	input >> polyhedron;
	Tree tree(faces(polyhedron).begin(), faces(polyhedron).end(), polyhedron);
	if (support_point_list.size() == 0)
		return;
	//set<Point3d_mesh,Z_cmp>support_point_sort;
	vector<Point3d_mesh>support_point_sort;
	for (int i = 0; i<support_point_list.size(); i++)
	{
		Point3d_mesh temp;
		temp.point = support_point_list[i];
		temp.flag = 1;
		support_point_sort.push_back(temp);
	}
	sort(support_point_sort.begin(), support_point_sort.end(), Z_cmp);
	vector<Point3d_mesh> temp_sort;
	for (int i = 0; i<support_point_sort.size(); i++)
	{
		Point3d a = support_point_sort[i].point, b = support_point_sort[i].point;
		b.z -= cone_length;
		int flag = 0;
		for (int j = 0; j < 9; j++)
		{
			Segment segment_query(Point(a.x, a.y, a.z-0.01), Point(a.x+cone_length * cos(PI * 2.0 / 9.0*j),
				a.y+cone_length*sin(PI * 2.0 / 9.0*j), a.z-cone_length*tan(support_angle / 180.0*PI)));
			if (tree.do_intersect(segment_query))
			{
				flag = 1; 
				cout << i << endl;
				break;
			}
		}
	/*	Segment segment_query(Point(a.x, a.y, a.z - 0.01), Point(a.x, a.y, a.z-2*cone_length));
		if (tree.do_intersect(segment_query))
		{
			flag = 1;
			cout << i << endl;
		}*/
		if (flag == 1)continue;
		Point3d_mesh B;
		B.point = b;
		B.flag = 2;
		temp_sort.push_back(B);
		support_line_node node;
		node.a = a;
		node.b = b;
		node.flaga = 1;
		node.flagb = 2;
		support_line.push_back(node);
	}
	support_point_sort = temp_sort;
	while (support_point_sort.size()>1)
	{
		if (support_point_sort[0].point.z <= min_z)break;
		int flag = 0;
		temp_sort = support_point_sort;
		double dis = 100000.0;
		int tmp = -1;
		Point3d insection_temp;
		for (int i = 1; i<support_point_sort.size(); i++)
		{
			if ((pow(support_point_sort[0].point.z - support_point_sort[i].point.z, 2)/ tan(support_angle / 180.0*PI) -
				pow(support_point_sort[0].point.x - support_point_sort[i].point.x, 2) -
				pow(support_point_sort[0].point.y - support_point_sort[i].point.y, 2))>-0.04)continue;
			Point3d insection = calculateTheIntersectionOfTwoCone(support_point_sort[0].point, support_point_sort[i].point, support_angle / 180.0*PI);
			double dis1 = dis_pp(support_point_sort[0].point, insection);
			int flag = 0;
			Point a(support_point_sort[0].point.x, support_point_sort[0].point.y, support_point_sort[0].point.z - 0.001);
			Point b(insection.x, insection.y, insection.z);
			for (int j = 0; j < 9; j++)
			{

				Segment segment_query(Point(a.x()+ (support_point_sort[0].flag*0.1-0.1+min_radius) * cos(PI * 2.0 / 9.0*j),
					a.y()+ (support_point_sort[0].flag*0.1-0.1 + min_radius)*sin(PI * 2.0 / 9.0*j), a.z() ),
					Point(insection.x + (support_point_sort[0].flag*0.1-0.1 + min_radius) * cos(PI * 2.0 / 9.0*j),
					insection.y + (support_point_sort[0].flag*0.1 - 0.1 + min_radius)*sin(PI * 2.0 / 9.0*j), insection.z));
				if (tree.do_intersect(segment_query))
				{
					flag = 1;
					break;
				}
				Segment segment_query1(Point(support_point_sort[i].point.x + (support_point_sort[0].flag*0.1 - 0.1 + min_radius) * cos(PI * 2.0 / 9.0*j),
					support_point_sort[i].point.y + (support_point_sort[0].flag*0.1 - 0.1 + min_radius)*sin(PI * 2.0 / 9.0*j), support_point_sort[i].point.z),
					Point(insection.x + (support_point_sort[0].flag*0.1 - 0.1 + min_radius) * cos(PI * 2.0 / 9.0*j),
						insection.y + (support_point_sort[0].flag*0.1 - 0.1 + min_radius)*sin(PI * 2.0 / 9.0*j), insection.z));
				if (tree.do_intersect(segment_query1))
				{
					flag = 1;
					break;
				}
			}
			if (flag == 1)continue;

			if (dis>dis1)
			{
				dis = dis1;
				tmp = i;
				insection_temp = insection;
			}
		}
		Point a;
		Point3d c3d;
		a = Point(support_point_sort[0].point.x, support_point_sort[0].point.y, support_point_sort[0].point.z - 0.02);
		if (dis > 0)
		{
			int edge = 13;
			for (int i = 0; i < edge; i++)
			{
				for (int j = 1; j <= 5; j++)
				{
					Vector b = Vector(1.0 / j*cos(PI * 2.0 / edge*i), 1.0 / j*sin(PI * 2.0 / edge*i), -1.0*tan(to_mesh_angle / 180.0*PI));

					Ray segment_query(a, b);
					Ray_intersection intersection =
						tree.first_intersection(segment_query);
					if (intersection)
					{
						const Point* p = boost::get<Point>(&(intersection->first));
						Point2Point3d(c3d, *p);
						//cout << *p << endl;
						double dis1 = dis_pp(support_point_sort[0].point, c3d);
						//cout << support_point_sort.size() << "  " << dis << " " << dis1 << endl;
						if (dis > dis1)
						{
							//cout << support_point_sort.size() << " " << dis1 << endl;
							dis = dis1;
							tmp = -1;
							flag = 1;
							insection_temp = c3d;
						}
					}
				}
			}

			//垂直的线判断一下
			Vector b = Vector(0, 0, -1.0);
			Ray segment_query(a, b);
			Ray_intersection intersection =
				tree.first_intersection(segment_query);
			if (intersection)
			{
				const Point* p = boost::get<Point>(&(intersection->first));
				Point2Point3d(c3d, *p);
				double dis1 = support_point_sort[0].point.z - c3d.z;
				if (dis > dis1)
				{
					dis = dis1;
					tmp = -1;
					flag = 1;
					insection_temp = c3d;
				}
			}
		}
		support_point_sort.clear();
		//cout << flag << endl;

		//不如直接垂直到最低点
		if (dis>temp_sort[0].point.z - min_z)
		{
			support_line_node node;
			node.a = temp_sort[0].point;
			node.b = temp_sort[0].point;
			node.b.z = min_z;
			node.flaga = temp_sort[0].flag;
			node.flagb = (max_radius-min_radius)/0.1+1;
			support_line.push_back(node);
			for (int i = 1; i<temp_sort.size(); i++)
				support_point_sort.push_back(temp_sort[i]);
		}
		else
		{
			//两圆锥相交
			if (flag == 0)
			{
				for (int i = 1; i < temp_sort.size(); i++)
					if (i != tmp)
						support_point_sort.push_back(temp_sort[i]);
				Point3d_mesh insection_mesh;
				//insection_mesh.flag = 0;
				insection_mesh.point = insection_temp;
				support_point_sort.push_back(insection_mesh);
				support_line_node node;
				node.a = temp_sort[0].point;
				node.b = insection_mesh.point;
				node.flaga = temp_sort[0].flag;
				node.flagb = min(int((max_radius - min_radius) / 0.1) + 1,max(temp_sort[tmp].flag, temp_sort[0].flag)+1);
				support_line.push_back(node);
				node.a = temp_sort[tmp].point;
				node.flaga = temp_sort[tmp].flag;
				support_line.push_back(node);

				sort(support_point_sort.begin(), support_point_sort.end(), Z_cmp);
			}
			//交到面上
			else
			{
				support_line_node node;
				node.a = temp_sort[0].point;
				node.b = insection_temp;
				node.flaga = temp_sort[0].flag;
				node.flagb = 1;
				support_line.push_back(node);
				for (int i = 1; i<temp_sort.size(); i++)
					support_point_sort.push_back(temp_sort[i]);
			}
		}
	}
	//把剩下的延申到底面，如果a.z也等于min_z，生成的时候会处理，b.z会下降
	for (int i = 0; i<support_point_sort.size(); i++)
	{
		support_line_node node;
		node.a = support_point_sort[i].point;
		node.flaga = support_point_sort[i].flag;
		node.flagb = int((max_radius - min_radius) / 0.1) + 1;
		node.b = Point3d(support_point_sort[i].point.x, support_point_sort[i].point.y, min_z);
		support_line.push_back(node);
	}
	GenerateSupportModel();
}

void merge_off(vector<Point3d>&sum_point, vector<Point3i>&sum_face, vector<Point3d>&leaf_point, vector<Point3i>&leaf_face)
{
	int sum = sum_point.size();
	for (int i = 0; i<leaf_point.size(); i++)
		sum_point.push_back(leaf_point[i]);
	for (int i = 0; i<leaf_face.size(); i++)
		sum_face.push_back(Point3i(leaf_face[i].x + sum, leaf_face[i].y + sum, leaf_face[i].z + sum));


}

//加底面
//2d两点距离
double dis_2d(Point2d a, Point2d b)
{
	Point2d c = a - b;
	return sqrt(c.x*c.x + c.y*c.y);
}
//叉乘
double X_2d(Point2d a, Point2d b)
{
	return a.x*b.y - a.y*b.x;
}
Point2d temp_2d;//point_2d[0]左下角的点
bool bottom_cmp(const  Point2d& a, const Point2d& b)
{
	double x = X_2d(a - temp_2d, b - temp_2d);
	if (x>0) return 1;
	if (x == 0 && dis_2d(a, temp_2d)<dis_2d(b, temp_2d)) return 1;
	return 0;
}
double multi(Point2d p1, Point2d p2, Point2d p3)
{
	return X_2d(p2 - p1, p3 - p1);
}
void support_point::bottom_base()
{
	vector<Point2d>bottom_point;
	point_2d.clear();
	for (int i = 0; i<point.size(); i++)
	{
		point_2d.push_back(Point2d(point[i].x, point[i].y));
	}

	int k = 1;
	for (int i = 1; i<point_2d.size(); i++)
		if (point_2d[i].y<point_2d[k].y || (point_2d[i].y == point_2d[k].y&&point_2d[i].x<point_2d[k].x))k = i;
	swap(point_2d[0], point_2d[k]);
	temp_2d = point_2d[0];
	sort(point_2d.begin() + 1, point_2d.end(), bottom_cmp);
	bottom_point.push_back(point_2d[0]);
	bottom_point.push_back(point_2d[1]);
	for (int i = 2; i<point_2d.size(); i++)
	{
		//while(t>=2&&multi(s[t-1],s[t],p[i])<=0) t--;
		while (1)
		{
			int t = bottom_point.size() - 1;
			if (t >= 2 && multi(bottom_point[t - 1], bottom_point[t], point_2d[i]) <= 0)
				bottom_point.pop_back();
			else
				break;
		}
		bottom_point.push_back(point_2d[i]);
	}
	model.creatBase(bottom_point, min_z - bottom_down+0.1, bottom_height);
	merge_off(support_off_point, support_off_face, model.bottomPoint, model.bottomFace);
}


bool cmp_line(support_line_node& a, support_line_node& b)
{
	return a.a.z>b.a.z;
}

void support_point::GenerateSupportModel()
{
	support_off_point.clear();
	support_off_face.clear();
	map<Point3d, double>point_radius;
	sort(support_line.begin(), support_line.end(), cmp_line);
	/*model.creatBase(min_x,max_x,min_y,max_y, min_z-bottom_down, bottom_height);
	merge_off(support_off_point, support_off_face, model.bottomPoint, model.bottomFace);*/
	bottom_base();
	for (int i = 0; i<support_line.size(); i++)
	{
		support_line_node temp;
		temp = support_line[i];
		//支撑向下bottom_down
		if (temp.b.z == min_z)
		{
			temp.b.z -= bottom_down;
			point_radius[temp.b] = max_radius;
		}
		if (temp.flaga == 1)
		{
			if (temp.flagb == 1)//两点都是在面上
			{
				if (dis_pp(temp.a, temp.b) <= 2 * cone_length)//距离过近，只生成两个锥
				{
					Point3d C(temp.a.x*0.5 + temp.b.x*0.5, temp.a.y*0.5 + temp.b.y*0.5, temp.a.z*0.5 + temp.b.z*0.5);
					double dis = dis_pp(temp.a, temp.b);
					temp.a.x += top*(temp.a.x - temp.b.x) / dis;
					temp.a.y += top*(temp.a.y - temp.b.y) / dis;
					temp.a.z += top*(temp.a.z - temp.b.z) / dis;
					model.creatCone(12, min_radius, temp.a, C);
					merge_off(support_off_point, support_off_face, model.conePoint, model.coneFace);
					dis = dis_pp(temp.a, temp.b);
					temp.b.x += top*(temp.b.x - temp.a.x) / dis;
					temp.b.y += top*(temp.b.y - temp.a.y) / dis;
					temp.b.z += top*(temp.b.z - temp.a.z) / dis;
					model.creatReverseCone(12, min_radius, temp.b, C);
					merge_off(support_off_point, support_off_face, model.conePoint, model.coneFace);
				}
				else//有一定距离，生成两个锥一个柱
				{
					double disAB = dis_pp(temp.a, temp.b);
					Point3d C(temp.a.x + cone_length*(temp.b.x - temp.a.x) / disAB, temp.a.y + cone_length*(temp.b.y - temp.a.y) / disAB,
						temp.a.z + cone_length*(temp.b.z - temp.a.z) / disAB);
					Point3d D(temp.b.x + cone_length*(temp.a.x - temp.b.x) / disAB, temp.b.y + cone_length*(temp.a.y - temp.b.y) / disAB,
						temp.b.z + cone_length*(temp.a.z - temp.b.z) / disAB);
					//第一个锥
					double dis = dis_pp(temp.a, temp.b);
					temp.a.x += top*(temp.a.x - temp.b.x) / dis;
					temp.a.y += top*(temp.a.y - temp.b.y) / dis;
					temp.a.z += top*(temp.a.z - temp.b.z) / dis;
					model.creatCone(12, min_radius, temp.a, C);
					merge_off(support_off_point, support_off_face, model.conePoint, model.coneFace);
					//柱
					model.creatCylinder(12, min_radius, min_radius, C, D);
					merge_off(support_off_point, support_off_face, model.cylinderPoint, model.cylinderFace);
					//第二个锥
					dis = dis_pp(temp.a, temp.b);
					temp.b.x += top*(temp.b.x - temp.a.x) / dis;
					temp.b.y += top*(temp.b.y - temp.a.y) / dis;
					temp.b.z += top*(temp.b.z - temp.a.z) / dis;
					model.creatReverseCone(12, min_radius, temp.b, D);
					merge_off(support_off_point, support_off_face, model.conePoint, model.coneFace);
				}
			}

			else //从面上延申出来一个锥
			{
				double dis = dis_pp(temp.a, temp.b);
				temp.a.x += top*(temp.a.x - temp.b.x) / dis;
				temp.a.y += top*(temp.a.y - temp.b.y) / dis;
				temp.a.z += top*(temp.a.z - temp.b.z) / dis;
				if (dis>cone_length)//生成一个柱，一个锥
				{
					double disAB = dis_pp(temp.a, temp.b);
					Point3d C(temp.a.x + cone_length*(temp.b.x - temp.a.x) / disAB, temp.a.y + cone_length*(temp.b.y - temp.a.y) / disAB,
						temp.a.z + cone_length*(temp.b.z - temp.a.z) / disAB);

					model.creatCone(12, min_radius, temp.a, C);
					merge_off(support_off_point, support_off_face, model.conePoint, model.coneFace);
					if (point_radius.count(temp.b) == 0)
						point_radius[temp.b] = min_radius + 0.1;
					else
					{
						point_radius[temp.b] = max(min_radius + 0.1, point_radius[temp.b]);
					}
					//柱
					model.creatCylinder(12, min_radius, min_radius + 0.1, C, temp.b);
					merge_off(support_off_point, support_off_face, model.cylinderPoint, model.cylinderFace);
				}
				else
				{
					if (point_radius.count(temp.b) == 0) point_radius[temp.b] = min_radius;
					else 	point_radius[temp.b] = max(min_radius, point_radius[temp.b]);

					model.creatCone(12, min_radius, temp.a, temp.b);
					merge_off(support_off_point, support_off_face, model.conePoint, model.coneFace);
				}
			}
		}
		else
		{
			if (temp.flagb == 1)//生成到面上的锥
			{
				double dis = dis_pp(temp.a, temp.b);
				//cout << top << endl;
				temp.b.x += top*(temp.b.x - temp.a.x) / dis;
				temp.b.y += top*(temp.b.y - temp.a.y) / dis;
				temp.b.z += top*(temp.b.z - temp.a.z) / dis;
				temp.b.z = max(temp.b.z, min_z);
				//cout << dis << " "<<cone_length << endl;
				
				if (dis <= cone_length)
				{
					model.creatReverseCone(12, point_radius[temp.a], temp.b, temp.a);
					merge_off(support_off_point, support_off_face, model.conePoint, model.coneFace);
				}
				else
				{
					double disAB = dis_pp(temp.a, temp.b);
					Point3d C(temp.b.x - cone_length*(temp.b.x - temp.a.x) / disAB, temp.b.y - cone_length*(temp.b.y - temp.a.y) / disAB,
						temp.b.z - cone_length*(temp.b.z - temp.a.z) / disAB);
					//生成锥
					model.creatReverseCone(12, min_radius, temp.b, C);
					merge_off(support_off_point, support_off_face, model.conePoint, model.coneFace);
					//生成柱
					model.creatCylinder(12, point_radius[temp.a], min_radius, temp.a, C);
					merge_off(support_off_point, support_off_face, model.cylinderPoint, model.cylinderFace);
				}

			}
			else   //只生成圆柱
			{
				if (point_radius.count(temp.b) == 0)
				{
					point_radius[temp.b] = min(point_radius[temp.a] + 0.1, max_radius);
				}
				else
				{
					point_radius[temp.b] = max(point_radius[temp.b], min(point_radius[temp.a] + 0.1, max_radius));
				}
				model.creatCylinder(12, point_radius[temp.a], point_radius[temp.b], temp.a, temp.b);
				merge_off(support_off_point, support_off_face, model.cylinderPoint, model.cylinderFace);
			}

		}
	}
	print_off("support.off", support_off_point, support_off_face);
}


void normalize(Point3d& a)
{
	double dis = a.x*a.x + a.y*a.y + a.z*a.z;
	dis = sqrt(dis);
	a.x /= dis;
	a.y /= dis;
	a.z /= dis;
}


Point3d support_point::calculateTheIntersectionOfTwoCone(const Point3d &A, const Point3d & B, double alpha)//A是
{
	//该点一定在两点连线的的下方
	Point3d C;
	double disz_to_xy;
	disz_to_xy = (A.z - B.z)/tan(alpha);
	double disxy = sqrt(pow(A.x - B.x, 2) + pow(A.y - B.y, 2));
	C.z = B.z - (disxy - disz_to_xy)/2 *tan(alpha);
	double temp = (disxy - disz_to_xy) / 2;
	C.x = B.x + (A.x - B.x)*temp / disxy;
	C.y = B.y + (A.y - B.y)*temp / disxy;
	return C;
}
