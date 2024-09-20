#ifndef __GirdH__
#define __GirdH__

#include <vector>
#include <algorithm>
#include <map>
#include "d3dUtility.h"

using namespace std;

#define CELL_POINT_NUM 6 //格子顶点个数 
#define FONT_SIZE 17     //字体大小

//界面类型
enum SurfaceType
{
	ENM_BLOCK_SURFACE = 1,       //画阻挡格子界面
	ENM_FIND_PATH_SURFACE = 2,   //寻路界面
};

//寻路状态
enum PathState
{
	ENM_START_PATH_STATE = 1,  //寻路起点状态
	ENM_END_PATH_STATE = 2,    //寻路终点状态
	ENM_SINGLE_STEP_STATE = 3, //单步寻路状态
	ENM_RULL_STEP_STATE = 4,   //一次寻路状态	
};

//--------向量类--------
class vector3D
{
public:
	vector3D()
	{
		init(0,0,0);
	}

	vector3D(int x,int y,DWORD color)
	{
		init(x,y,color);	
	}

	void init(int x,int y,DWORD color)
	{
		m_X = x;
		m_Y = y;
		m_Z = 0;
		m_rhw = 1.0f;
		m_color = color;
	}

	~vector3D(){}

	float m_X;
	float m_Y;
	float m_Z;
	float m_rhw;
	DWORD m_color;
};

//寻路结果类型
enum path_result_type
{
	ENM_PATH_RESULT_ING = 1,     //正在寻路
	ENM_PATH_RESULT_SUCCESS = 2, //寻路成功
	ENM_PATH_RESULT_FAILD = 3,   //寻路失败
};

//点所在格子位置类型
enum pos_position_type
{
	ENM_POS_INVALID = 0, //无效
	ENM_POS_TOP = 1,     //上方
	ENM_POS_LOWER = 2,   //下方
	ENM_POS_LEFT = 3,    //左边
	ENM_POS_RIGHT = 4,   //右边
}; 

//---------格子类--------
class cell
{
public:
	cell(){}

	~cell(){}

	vector3D m_point[CELL_POINT_NUM];
};

//---Astar 点----
class astar_point
{
public:
	astar_point()
	{
		init();
	}

	void init()
	{
		m_h = 0;
		m_g = 0;
		m_f = 0;
		m_index = 0;
		m_in_open = false;
		m_in_close = false;
		m_father = NULL;
	}
	//初始状态经由状态 n 到目标路径代价总估值
	int m_h;
	//从初始状态到状态 n 的实际代价值
	int m_g;
	//从状态 n 到目标状态的估值
	int m_f;
	//数组索引
	int m_index;
	//是否已在 open_list 中
	bool m_in_open;
	//是否在 close_list 中 
	bool m_in_close;
	//父结点
	astar_point *m_father;
};

//------------------------网格类-------------------------
class gird
{
public:
	//初始化函数
	void init(int numX,int numY,int size,int left,int right);

	//画图入口函数
	void display(LPD3DXLINE line,ID3DXFont *font);

public:
	/*----画阻挡墙界面操作----*/
	//点击某个点
	void click_point(int x,int y,IDirect3DVertexBuffer9 *pvectex);

public:
	/*----寻路界面操作----*/
	//寻路开始
	void path_begin(const vector3D &start);

	//寻路(支持单步寻路)
	int find_path(const vector3D &end);

	//设置寻路起始点、终点
	void set_path(IDirect3DVertexBuffer9 *pvectex,int x,int y,PathState path_type,DWORD color);

public:
	/*----清理函数----*/
	//清理路径
	void clean_path(IDirect3DVertexBuffer9 *pvectex);

	//清理除阻挡墙
	void clean_cell(IDirect3DVertexBuffer9 *pvectex);

public:
	/*----公共部分函数----*/
	//获得格子个数
	int get_cell_size();

	//是否在区域内
	bool is_in_area(int x,int y);

	//某个点是否可以单击
	bool is_can_click(int x,int y);

	//设置单步寻路
	void set_single_step(bool single_step){m_single_step = single_step;}

	//获得单步寻路变量
	bool get_single_step(){return m_single_step;}

private:	
	/*----画图部分函数----*/
	//画直线
	void draw_line(LPD3DXLINE line,D3DXVECTOR2 *vec_list,int num,DWORD color,float width);

	//画路径
	void draw_path(LPD3DXLINE line);

	//画网格
	void draw_grid(LPD3DXLINE line);

	//画单步寻路标记
	void draw_step(LPD3DXLINE line,ID3DXFont *font);

	//画文本,RECT 表示左上角坐标(x,y)、右下角坐标(x,y)
	void draw_text(ID3DXFont *font,astar_point *point);

	//画路径方向
	void draw_direct(LPD3DXLINE line,astar_point *point);

	//画当前搜索点边框
	void draw_cur_search_point(LPD3DXLINE line);

private:
	//刷新阻挡格子
	void refresh_cell(IDirect3DVertexBuffer9 *pvectex);

	//格子内存数据放入缓冲区
	void swap_cell_data(vector3D *buffer,vector3D *src,int index);

	//获得点索引
	int get_point_index(int x,int y,int &indexX,int &indexY);

	//获得 G 值
	int get_g(int srcX,int srcY,int targetX,int targetY);

	//获得 H 值
	int get_h(int srcX,int srcY,int targetX,int targetY);

	//生成一个格子
	void make_cell(cell *c,int indexX,int indexY,DWORD color);

	//清理寻路标记
	void clean_path_flag();

	//获得位置类型
	pos_position_type get_pos_type(int i,int j);

	//根据位置获得坐标点
	void get_pos_by_type(astar_point *point,pos_position_type type,int &x,int &y);

	//获得箭头坐标
	void get_arrow_pos(pos_position_type type,int x,int y,D3DXVECTOR2 *vec_list);

	//关系转换
	pos_position_type change_relation(pos_position_type type);

private:
	/*--------以下为寻路时用到变量----------*/
	//当前点 open 节点
	astar_point *m_cur_open;
	//单步寻路
	bool m_single_step;
	//寻路起始、结束格子
	map<int,cell> m_path_cell;
	//路径点
	vector<vector3D> m_path_list;
	//开放列表
	vector<astar_point*> m_open_list;

	/*--------以下为网格部分用到变量--------*/
	//x 轴方向格子数
	int m_numX; 
	//y 轴方向格子数
	int m_numY; 
	//格子大小
	int m_size;  
	//起始 x 坐标点
	int m_startX;
	//起始 y 坐标点
	int m_startY;
	//阻挡格子
	map<int,cell> m_cell_list;
	//网格数组
	vector<vector<astar_point>> m_grid_array;
};

#endif