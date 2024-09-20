#include "stdafx.h"
#include "gird.h"

//初始化函数
void gird::init(int numX,int numY,int size,int left,int right)
{
	m_numX = numX;
	m_numY = numY;
	m_size = size;
	m_cur_open = NULL;
	m_single_step = false;

	//x 开始位置
	m_startX = int((left-numX*size)/2) - (int)(size/2); 
	//y 开始位置
	m_startY = int((right-numY*size)/2) - (int)(size/2);

	//网格数组
	m_grid_array.clear();
	for(int i = 0;i < m_numX;++i)
	{
		m_grid_array.push_back(vector<astar_point>());
		vector<astar_point> &temp = m_grid_array.back();
		for(int j = 0;j < m_numY;++j)
		{
			temp.push_back(astar_point());
		}
	}
}

//画图入口函数
void gird::display(LPD3DXLINE line,ID3DXFont *font)
{
	draw_grid(line);
	draw_path(line);
	draw_step(line,font);
	draw_cur_search_point(line);
}

//点击某个点
void gird::click_point(int x,int y,IDirect3DVertexBuffer9 *vectex)
{
	if(!is_in_area(x,y))
	{
		return ;
	}
	//二维数组 x,y 索引
	int indexX = 0;
	int indexY = 0;
	//一组数组索引
	int index = get_point_index(x,y,indexX,indexY);
	map<int,cell>::iterator it = m_cell_list.find(index);
	if(it == m_cell_list.end())
	{
		m_cell_list[index] = cell();
		make_cell(&m_cell_list[index],indexX,indexY,0x0);
	}
	//不为 NULL 表示删除
	else
	{
		m_cell_list.erase(it);
	}
	refresh_cell(vectex);
}

//寻路开始
void gird::path_begin(const vector3D &start)
{
	m_open_list.clear();	
	clean_path_flag();
	int indexX = 0;
	int indexY = 0;
	//获是起始点
	int start_index = get_point_index(start.m_X,start.m_Y,indexX,indexY);
	astar_point *start_point = &m_grid_array[indexX][indexY];
	start_point->m_index = start_index;
	//放入 m_open_list 列表中
	m_open_list.push_back(start_point);
}

//比较函数
bool compare(astar_point *left,astar_point *right)
{
	return left->m_f > right->m_f;
}	

//寻路(支持单步寻路)
int gird::find_path(const vector3D &end)
{
	//终点索引	
	int index_endX = 0;
	int index_endY = 0;
	int result_index = get_point_index(end.m_X,end.m_Y,index_endX,index_endY);
	astar_point *result_point = NULL;
	map<int,cell>::iterator it;
	do
	{
		astar_point *cur_point = m_open_list.back();
		m_cur_open = cur_point;
		m_open_list.pop_back();
		cur_point->m_in_close = true;
		//找到终点
		if(result_index == cur_point->m_index)
		{
			m_cur_open = NULL;
			result_point = cur_point;
			break;
		}
		int indexX = cur_point->m_index % m_numX;
		int indexY = cur_point->m_index / m_numX;
		//遍历周围 9 个格,将符合条件的格子放入 open_list 中
		for(int i = -1;i <= 1;++i)
		{
			int tempX = indexX + i;
			//x 超出网格
			if(tempX < 0 || tempX >= m_numX)
			{
				continue;
			}
			for(int j = -1;j <= 1;++j)
			{
				//不允许走斜角
				if(get_pos_type(i,j) == ENM_POS_INVALID)
				{
					continue;
				}
				//y 超出网格
				int tempY = indexY + j;
				if(tempY < 0 || tempY >= m_numY)
				{
					continue;
				}
				astar_point *temp_point = &m_grid_array[tempX][tempY];
				//自己
				if(cur_point == temp_point)
				{
					continue;
				}
				//已在 close 不要再放入 open 中
				if(temp_point->m_in_close)
				{
					continue;
				}
				int cur_index = m_numX*tempY + tempX;
				it = m_cell_list.find(cur_index);
				//阻挡格子
				if(it != m_cell_list.end())
				{
					continue;
				}
				//计算 g 值
				int cur_g = get_g(indexX,indexY,tempX,tempY);
				//已经在 m_open_list 中
				if(temp_point->m_in_open)	
				{
					int f = cur_point->m_g + cur_g + temp_point->m_h;
					//经过当前结点的总估值代价大过
					if(f >= temp_point->m_f)
					{
						continue;
					}
					//修正 g f 值和父结点
					temp_point->m_g = cur_point->m_g + cur_g;
					temp_point->m_f = temp_point->m_h + temp_point->m_g;
					temp_point->m_father = cur_point;
					sort(m_open_list.begin(),m_open_list.end(),compare);
					continue;
				}
				//计算 h 值
				int cur_h = get_h(index_endX,index_endY,tempX,tempY);
				temp_point->m_in_open = true;
				temp_point->m_father = cur_point;	
				//h 值为固定值
				temp_point->m_h = cur_h;
				temp_point->m_g = cur_point->m_g + cur_g;
				temp_point->m_f = cur_h + temp_point->m_g;
				temp_point->m_index = cur_index;
				m_open_list.push_back(temp_point);
				sort(m_open_list.begin(),m_open_list.end(),compare);
			}
		}
	}while(!m_single_step && m_open_list.size() > 0);

	//单步寻路
	if(m_single_step)
	{
		if(result_point == NULL)
		{
			if(m_open_list.size() == 0)
			{
				return ENM_PATH_RESULT_FAILD;
			}
			return ENM_PATH_RESULT_ING;
		}
	}
	else
	{
		//一次寻路结果为 NULL 时返回失败
		if(result_point == NULL)
		{
			return ENM_PATH_RESULT_FAILD;
		}
	}

	m_path_list.clear();
	//设置路径
	while(result_point != NULL)
	{
		int indexX = result_point->m_index % m_numX;
		int indexY = result_point->m_index / m_numX;
		vector3D temp_vec;
		temp_vec.m_X = m_startX + indexX*m_size + m_size/2; 
		temp_vec.m_Y = m_startY + indexY*m_size + m_size/2;
		m_path_list.push_back(temp_vec);
		result_point = result_point->m_father;
	}
	
	return ENM_PATH_RESULT_SUCCESS;
}

//设置寻路起始点、终点
void gird::set_path(IDirect3DVertexBuffer9 *vectex,int x,int y,PathState path_type,DWORD color)
{
	//二维数组 x,y 索引
	int indexX = 0;
	int indexY = 0;
	cell &c = m_path_cell[path_type];
	get_point_index(x,y,indexX,indexY);	
	make_cell(&c,indexX,indexY,color);
	refresh_cell(vectex);
}

//清理路径
void gird::clean_path(IDirect3DVertexBuffer9 *vectex)
{
	clean_path_flag();
	m_cur_open = NULL;
	m_path_list.clear();
	m_path_cell.clear();
	m_open_list.clear();
	refresh_cell(vectex);
}

//清理除阻挡墙
void gird::clean_cell(IDirect3DVertexBuffer9 *vectex)
{
	m_cell_list.clear();
	refresh_cell(vectex);
}

//获得格子个数
int gird::get_cell_size()
{
	return m_cell_list.size() + m_path_cell.size();
}

//是否在区域内
bool gird::is_in_area(int x,int y)
{
	if(x <= m_startX)
	{
		return false;
	}
	if(y <= m_startY)
	{
		return false;
	}
	int endX = m_startX + m_numX*m_size;
	if(x >= endX)
	{
		return false;
	}
	int endY = m_startY + m_numY*m_size;
	if(y >= endY)
	{
		return false;
	}
	return true;
}

//某个点是否可以单击
bool gird::is_can_click(int x,int y)
{
	if(!is_in_area(x,y))
	{
		return false;
	}
	//二维数组 x,y 索引
	int indexX = 0;
	int indexY = 0;
	int index = get_point_index(x,y,indexX,indexY);
	map<int,cell>::iterator it = m_cell_list.find(index);	
	if(it != m_cell_list.end())
	{
		return false;
	}
	return true;
}

//画直线
void gird::draw_line(LPD3DXLINE line,D3DXVECTOR2 *vec_list,int num,DWORD color,float width)
{
	line->SetWidth(width);
    line->SetAntialias(true);
	line->Draw(vec_list,num,color);
}

//画路径
void gird::draw_path(LPD3DXLINE line)
{
	if(m_path_list.size() == 0)
	{
		return ;
	}
	//暂时写 1000 个
	static D3DXVECTOR2 vec_list[1000];
	for(int i = 0;i < (int)m_path_list.size();++i)
	{
		vec_list[i].x = m_path_list[i].m_X;
		vec_list[i].y = m_path_list[i].m_Y;
	}
	draw_line(line,vec_list,m_path_list.size(),D3DCOLOR_XRGB(255,0,0),2.0f);
}

//画网格
void gird::draw_grid(LPD3DXLINE line)
{
	D3DXVECTOR2 vec_list[2];
	//水平直线
	for(int i = 0;i < m_numY+1;++i)
	{
		//起点
		vec_list[0].x = (float)m_startX;
		vec_list[0].y = (float)m_startY + i*m_size;

		//终点
		vec_list[1].x = (float)m_startX + m_size*m_numX;
		vec_list[1].y = (float)m_startY + i*m_size;
		draw_line(line,vec_list,2,D3DCOLOR_XRGB(0,0,0),2.0f);
	}
	//垂直直线
	for(int i = 0;i < m_numX+1;++i)
	{
		//起点
		vec_list[0].x = (float)m_startX + i*m_size;
		vec_list[0].y = (float)m_startY;

		//终点
		vec_list[1].x = (float)m_startX + i*m_size;
		vec_list[1].y = (float)m_startY + m_size*m_numY;
		draw_line(line,vec_list,2,D3DCOLOR_XRGB(0,0,0),2.0f);
	}
}

//画单步寻路标记
void gird::draw_step(LPD3DXLINE line,ID3DXFont *font)
{
	if(!m_single_step)
	{
		return;
	}
	for(int i = 0;i < m_numX;++i)	
	{
		for(int j = 0;j < m_numY;++j)
		{
			astar_point *point = &m_grid_array[i][j];	
			if(!point->m_in_open && !point->m_in_close)
			{
				continue;
			}
			draw_text(font,point);
			draw_direct(line,point);
		}
	}
}

//画文本,RECT 表示左上角坐标(x,y)、右下角坐标(x,y)
void gird::draw_text(ID3DXFont *font,astar_point *point)
{
	//当前 open list 中最小点,最小点在数组末尾
	astar_point *min_point = NULL;
	if(m_open_list.size() != 0)
	{
		min_point = m_open_list.back();
	}
	//数组索引
	int indexX = point->m_index % m_numX;
	int indexY = point->m_index / m_numX;

	//起点位置
	int x = m_startX + indexX*m_size;
	int y = m_startY + indexY*m_size;
	/*以下文本坐标中的常量值(例如, 2 4 8 等这些数据)是调出来的,无需理解表达含义*/
	int size = 40; 
	WCHAR temp_str[256];
	DWORD color = 0;
	//文本 x 轴偏移
	int offsetX1 = 6;
	int offsetX2 = 4;
	int offsetX3 = 7;

	//1、画 F 值
	RECT rect_f = {x + offsetX3,y + m_size/2 - FONT_SIZE/2 + 4,x + offsetX3 + size,y + m_size/2 - FONT_SIZE/2 + size + 4};
	swprintf(temp_str,L"F=%d",point->m_f);	
	color = D3DCOLOR_XRGB(0,0,0);
	//当前最小 F 值显示颜色不一样
	if(min_point != NULL && point->m_f == min_point->m_f && !point->m_in_close)
	{
		color = D3DCOLOR_XRGB(0,205,0);
	}
	font->DrawText(NULL,temp_str,-1,&rect_f,DT_TOP | DT_LEFT,color);

	//2、画 g 值
	RECT rect_g = {x + offsetX1,y + m_size - FONT_SIZE - 8,x + offsetX1 + size,y + m_size - FONT_SIZE + size - 8};
	swprintf(temp_str,L"g=%d",point->m_g);
	font->DrawText(NULL,temp_str,-1,&rect_g,DT_TOP | DT_LEFT,D3DCOLOR_XRGB(0,0,0));  

	//3、画 h 值
	RECT rect_h = {x + offsetX2 + m_size/2,y + m_size - FONT_SIZE - 8,x + offsetX2 + m_size/2 + size,y + m_size - FONT_SIZE + size - 8};
	swprintf(temp_str,L"h=%d",point->m_h);	
	font->DrawText(NULL,temp_str,-1,&rect_h,DT_TOP | DT_LEFT,D3DCOLOR_XRGB(0,0,0));

	//4、画 open close next 标记 
	RECT rect_word = {x + offsetX1,y + 4,x + offsetX1 + size,y + 4 + size};
	color = 0;
	//open 中的点
	if(!point->m_in_close)
	{
		//下次被选中做为搜索源的点要显示成 next 文本
		if(point == min_point)
		{
			color = D3DCOLOR_XRGB(255,0,128);
			swprintf(temp_str,L"next");	
		}
		else	
		{
			color = D3DCOLOR_XRGB(255,0,0);
			swprintf(temp_str,L"open");
		}
	}
	//close 中的点
	else
	{
		swprintf(temp_str,L"close");
		color = D3DCOLOR_XRGB(0,0,0);
	}
	font->DrawText(NULL,temp_str,-1,&rect_word,DT_TOP | DT_LEFT,color);
}

//画路径方向
void gird::draw_direct(LPD3DXLINE line,astar_point *point)
{
	if(point->m_father == NULL)
	{
		return ;
	}
	//当前结点数组索引
	int cur_indexX = point->m_index % m_numX;
	int cur_indexY = point->m_index / m_numX;

	//父结点数组索引
	int father_indexX = point->m_father->m_index % m_numX;
	int father_indexY = point->m_father->m_index / m_numX;

	int x = father_indexX - cur_indexX;
	int y = father_indexY - cur_indexY;
	//直线
	D3DXVECTOR2 vec_line[2];
	pos_position_type type = get_pos_type(x,y);
	//当前结点直线起始坐标
	get_pos_by_type(point,type,x,y);
	vec_line[0].x = x;
	vec_line[0].y = y;

	//当前结点直线终点坐标
	type = change_relation(type);
	get_pos_by_type(point->m_father,type,x,y);
	vec_line[1].x = x;
	vec_line[1].y = y;
	//画直线
	draw_line(line,vec_line,2,D3DCOLOR_XRGB(0,0,0),2.0f);

	//箭头
	D3DXVECTOR2 vec_arrow[3];
	get_arrow_pos(type,x,y,vec_arrow);
	//画箭头
	draw_line(line,vec_arrow,3,D3DCOLOR_XRGB(0,0,0),2.0f);
}

//画当前搜索点边框
void gird::draw_cur_search_point(LPD3DXLINE line)
{
	//画当前搜索点边框
	if(m_cur_open == NULL || !m_single_step)
	{
		return;
	}
	D3DXVECTOR2 vec_list[5];
	int indexX = m_cur_open->m_index % m_numX;
	int indexY = m_cur_open->m_index / m_numX;
	vec_list[0].x = (float)m_startX + indexX*m_size;
	vec_list[0].y = (float)m_startY + indexY*m_size;

	vec_list[1].x = (float)m_startX + (indexX + 1)*m_size;
	vec_list[1].y = (float)m_startY + indexY*m_size;

	vec_list[2].x = (float)m_startX + (indexX + 1)*m_size;
	vec_list[2].y = (float)m_startY + (indexY + 1)*m_size;

	vec_list[3].x = (float)m_startX + indexX*m_size;
	vec_list[3].y = (float)m_startY + (indexY + 1)*m_size;

	vec_list[4].x = (float)m_startX + indexX*m_size;
	vec_list[4].y = (float)m_startY + indexY*m_size;
	draw_line(line,vec_list,5,D3DCOLOR_XRGB(255,0,0),4.0f);
}

//刷新阻挡格子
void gird::refresh_cell(IDirect3DVertexBuffer9 *vectex)
{
	vector3D *vectors;
	vectex->Lock(0, 0, (void**)&vectors, 0);
	map<int,cell>::iterator it = m_cell_list.begin();
	int index = 0;
	//一个格子 6 个顶点
	//阻挡格子
	for(;it != m_cell_list.end();++it)
	{
		swap_cell_data(vectors,it->second.m_point,index);
		index += CELL_POINT_NUM;
	}
	//路径格子
	it = m_path_cell.begin();
	for(;it != m_path_cell.end();++it)
	{
		swap_cell_data(vectors,it->second.m_point,index);
		index += CELL_POINT_NUM;
	}
	vectex->Unlock();
}

//格子内存数据放入缓冲区
void gird::swap_cell_data(vector3D *buffer,vector3D *src,int index)
{
	buffer[index + 0] = src[0];	
	buffer[index + 1] = src[1];
	buffer[index + 2] = src[2];
	buffer[index + 3] = src[3];	
	buffer[index + 4] = src[4];
	buffer[index + 5] = src[5];
}

//获得点索引
int gird::get_point_index(int x,int y,int &indexX,int &indexY)
{
	//相对位置,从 0 开始
	int relativeX = x - m_startX;	
	int relativeY = y - m_startY; 

	//二维数组 x,y 索引
	indexX = relativeX/m_size;
	indexY = relativeY/m_size;

	//一组数组索引
	int index = indexY*m_numX + indexX;	
	return index;
}

//获得 G 值
int gird::get_g(int srcX,int srcY,int targetX,int targetY)
{
	return 1;
}

//获得 H 值
int gird::get_h(int srcX,int srcY,int targetX,int targetY)
{
	int diffX = abs(srcX - targetX);
	int diffY = abs(srcY - targetY);
	return diffX + diffY;
}

//生成一个格子
void gird::make_cell(cell *c,int indexX,int indexY,DWORD color)
{
	vector3D *ppoint = c->m_point; 
	/* a-----b
	   |     |
	   |     |
	   d-----c
	   一个阻挡格子两个三角形,一个三角形 3 个顶点
	*/
	//a 点
	ppoint[0] = vector3D(m_startX + indexX*m_size,m_startY + indexY*m_size,color);
	//b 点
	ppoint[1] = vector3D(m_startX + (indexX+1)*m_size,m_startY + indexY*m_size,color); 
	//c 点
	ppoint[2] = vector3D(m_startX + (indexX+1)*m_size,m_startY + (indexY+1)*m_size,color); 
	//c 点
	ppoint[3] = vector3D(m_startX + (indexX+1)*m_size,m_startY + (indexY+1)*m_size,color);  
	//d 点
	ppoint[4] = vector3D(m_startX + indexX*m_size,m_startY + (indexY+1)*m_size,color); 
	//a 点
	ppoint[5] = vector3D(m_startX + indexX*m_size,m_startY + indexY*m_size,color);
}

//清理寻路标记
void gird::clean_path_flag()
{
	//网格数组
	for(int i = 0;i < m_numX;++i)
	{
		for(int j = 0;j < m_numY;++j)
		{
			m_grid_array[i][j].init();
		}
	}
}

//获得位置类型
pos_position_type gird::get_pos_type(int i,int j)
{
	//1、上方
	if(i == 0 && j == -1)
	{
		return ENM_POS_TOP;
	}
	//2、左方
	if(i == -1 && j == 0)
	{
		return ENM_POS_LEFT;
	}
	//3、右方
	if(i == 1 && j == 0)
	{
		return ENM_POS_RIGHT;
	}
	//4、下方
	if(i == 0 && j == 1)
	{
		return ENM_POS_LOWER;
	}	
	return ENM_POS_INVALID;
}

//根据位置获得坐标点
void gird::get_pos_by_type(astar_point *point,pos_position_type type,int &x,int &y)
{
	//数组索引
	int indexX = point->m_index % m_numX;
	int indexY = point->m_index / m_numX;
	float percentA = 1.f/10;
	float percentB = 9.f/10;
	switch(type)
	{
	case ENM_POS_TOP:
		{
			x = m_startX + indexX*m_size + m_size/2;
			y = m_startY + indexY*m_size + m_size*percentA;
			break;
		}
	case ENM_POS_LOWER:
		{
			x = m_startX + indexX*m_size + m_size/2;
			y = m_startY + indexY*m_size + m_size*percentB;
			break;
		}
	case ENM_POS_LEFT:
		{
			x = m_startX + indexX*m_size + m_size*percentA;
			y = m_startY + indexY*m_size + m_size/2;
			break;
		}
	case ENM_POS_RIGHT:
		{
			x = m_startX + indexX*m_size + m_size*percentB;
			y = m_startY + indexY*m_size + m_size/2;
			break;
		}
	}
}

//获得箭头坐标
void gird::get_arrow_pos(pos_position_type type,int x,int y,D3DXVECTOR2 *vec_list)
{
	vec_list[1].x = x;
	vec_list[1].y = y;
	int size = 5;
	switch(type)
	{
	case ENM_POS_TOP:
		{
			vec_list[0].x = x - size;
			vec_list[0].y = y - size;

			vec_list[2].x = x + size;
			vec_list[2].y = y - size;
			break;
		}
	case ENM_POS_LOWER:
		{
			vec_list[0].x = x - size;
			vec_list[0].y = y + size;

			vec_list[2].x = x + size;
			vec_list[2].y = y + size;
			break;
		}
	case ENM_POS_LEFT:
		{
			vec_list[0].x = x - size;
			vec_list[0].y = y - size;

			vec_list[2].x = x - size;
			vec_list[2].y = y + size;
			break;
		}
	case ENM_POS_RIGHT:
		{
			vec_list[0].x = x + size;
			vec_list[0].y = y - size;

			vec_list[2].x = x + size;
			vec_list[2].y = y + size;
			break;
		}
	}	
}

//关系转换
pos_position_type gird::change_relation(pos_position_type type)
{
	switch(type)
	{
	case ENM_POS_TOP:
		{
			return ENM_POS_LOWER;
		}
	case ENM_POS_LOWER:
		{
			return ENM_POS_TOP;
		}
	case ENM_POS_LEFT:
		{
			return ENM_POS_RIGHT;
		}
	case ENM_POS_RIGHT:
		{
			return ENM_POS_LEFT;
		}
	}
	return ENM_POS_INVALID;
}