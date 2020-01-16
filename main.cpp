// main.cpp : 此文件包含 "main" 函数。程序执行将在此处开始并结束。
//Can I achieve it before the Spring Festival.oligay!!!
#include <iostream>
#include "support.h"
#include <time.h>
int main()
{
	support_point test;
	test.min_radius = 0.25;
	test.max_radius = 1.0;
	test.top = 0.3;
	test.normal_top=0.3;
	test.normal_cone_length=0.9;
	test.normal_cyline_length = 0.2;
	test.cone_length = 0.9;
	test.bottom_radius = 8.0;
	test.bottom_height = 0.5;
	test.support_angle = 45.0;
	test.to_mesh_angle = 55.0;
	test.bottom_down = 1.5;
	test.dis_x = 2.0;
	test.dis_y = 2.0;
	test.min_z = 100000.0;
	test.min_x = 100000.0;
	test.min_y = 100000.0;
	test.max_x = -100000.0;
	test.max_y = -100000.0;
	test.file_name = "C:\\Users\\Bamboo\\Desktop\\CLEAR.off";
	clock_t start,finish;
	start=clock();
	test.readoff(test.file_name);
	test.GetSupportPoint();
	finish=clock();
	cout<<(finish-start)/CLOCKS_PER_SEC<<endl;
	//getchar();
}
