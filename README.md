

# support DLP树形支撑算法

# 需要配置cgal，在vs2015做过实验。

  test.min_radius = 0.25; //支撑和面接触的圆锥的截面半径
  
	test.max_radius = 1.0; //支撑的最大半径
  
	test.top = 0.22;        //支撑插进去模型的长度
  
	test.cone_length = 0.5; //支撑和模型接触的锥的长度
  
	test.bottom_height = 0.5;//底垫的高度
  
	test.support_angle = 45.0;//需要加支撑的最小角度，这是和平面形成的角度，这个值越大，支撑越多
  
	test.to_mesh_angle = 45.0;//支撑如果加到模型上，角度最小值，这个和support_angle一样也可以。如果加到模型上的支撑可能和相交则
  
	test.bottom_down = 1.5;   //支撑和地垫的距离
  
	test.dis_x=2.0;           //需要支撑的X轴密度
  
	test.dis_y=2.0;            //需要支撑的y轴密度
