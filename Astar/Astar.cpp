// Astar.cpp : 定义控制台应用程序的入口点。
//
#include "stdafx.h"
#include "d3dUtility.h"
#include "gird.h"

#define D3D_FVF_VECTOR (D3DFVF_XYZRHW | D3DFVF_DIFFUSE)

//窗口大小
const int Width  = 1600;
const int Height = 900;

//A* 网格个数
const int AstarX = 15;
const int AstarY = 10; 
//阻挡格子大小
const int CellSize = 75;

//Direct3D线对象
IDirect3DDevice9* Device = NULL;
//顶点
IDirect3DVertexBuffer9 *g_vectex = NULL;
//文本字体
ID3DXFont* g_font = NULL;
//a* 格子 f、g、h 值字体
ID3DXFont* g_astar_font = NULL;
//直线
LPD3DXLINE g_line = NULL;

//网格
gird g_gird;

//界面类型
SurfaceType g_surface = ENM_BLOCK_SURFACE;
//寻路状态
PathState g_path_state = ENM_START_PATH_STATE;

//寻路起始点
vector3D g_start_point;
//寻路终点
vector3D g_end_point;

//阻挡墙界面文本
WCHAR g_block_prompt[256] = {
	L"单击格子生成阻挡墙  按回车进入寻路界面 按右键清除阻挡格子"
};

//寻路界面文本
WCHAR g_path_prompt[3][256] = {
	{L"单击格子生成起始寻路点  当前[%s]寻路按右键切换状态  按回车进入生成阻挡墙界面"},
	{L"单击格子生成终点寻路点  当前[%s]寻路按右键切换状态  按回车进入生成阻挡墙界面"},
	{L"点击鼠标左键单步寻路  当前[%s]寻路状右键切换状态  按回车进入生成阻挡墙界面"}
};

//寻路状态
void deal_path_state(int x,int y)
{
	switch(g_path_state)
	{
	//寻路起点
	case ENM_START_PATH_STATE:
		{
			g_gird.clean_path(g_vectex);
			if(g_gird.is_can_click(x,y))
			{
				g_start_point.m_X = x;
				g_start_point.m_Y = y;	
				g_path_state = ENM_END_PATH_STATE;
				g_gird.set_path(g_vectex,x,y,ENM_START_PATH_STATE,D3DCOLOR_XRGB(0,128,0));
			}
			break;
		}
	//寻路终点
	case ENM_END_PATH_STATE:
		{
			if(!g_gird.is_can_click(x,y))
			{
				break;
			}
			g_end_point.m_X = x;
			g_end_point.m_Y = y;	
			g_gird.path_begin(g_start_point);
			g_gird.set_path(g_vectex,x,y,ENM_END_PATH_STATE,D3DCOLOR_XRGB(255,0,0));
			//单步寻路
			if(g_gird.get_single_step())
			{
				g_path_state = ENM_SINGLE_STEP_STATE;
				deal_path_state(0,0);
			}
			//一次寻路
			else
			{
				g_gird.find_path(g_end_point);
				g_start_point.init(0,0,0);
				g_end_point.init(0,0,0);
				g_path_state = ENM_START_PATH_STATE;
			}
			break;
		}
	//单步寻路
	case ENM_SINGLE_STEP_STATE:
		{
			int type = g_gird.find_path(g_end_point);
			if(type == ENM_PATH_RESULT_SUCCESS || type == ENM_PATH_RESULT_FAILD)
			{
				g_start_point.init(0,0,0);
				g_end_point.init(0,0,0);
				g_path_state = ENM_START_PATH_STATE;
			}
			break;
		}
	}
}

//获得寻路执行状态文本
WCHAR* get_path_execute_state()
{
	static WCHAR temp[256];
	if(g_gird.get_single_step())
	{
		swprintf(temp,L"单步");
	}
	else
	{
		swprintf(temp,L"一次");
	}
	return temp;
}

//文本提示
void draw_prompt_text()
{
	RECT rect = {Width/2 - 700,6,Width/2 + 650,40};
	WCHAR *str = NULL;
	switch(g_surface)
	{
	//画阻挡格子界面
	case ENM_BLOCK_SURFACE:
		{
			str = g_block_prompt;
			break;
		}
	//寻路
	case ENM_FIND_PATH_SURFACE:
		{
			static WCHAR temp[256];
			str = temp;
			swprintf(temp,g_path_prompt[g_path_state-1],get_path_execute_state());
			break;
		}
	}
	g_font->DrawText(
		NULL,
		str,
		-1, // size of string or -1 indicates null terminating string
		&rect,            // rectangle text is to be formatted to in windows coords
		DT_TOP | DT_CENTER, // draw in the top left corner of the viewport
		D3DCOLOR_XRGB(0,205,0));      // black text
}

//回车键按下
void deal_return_key()
{
	switch(g_surface)
	{
	//画阻挡格子界面
	case ENM_BLOCK_SURFACE:
		{
			g_surface = ENM_FIND_PATH_SURFACE;
			break;
		}
	//寻路界面
	case ENM_FIND_PATH_SURFACE:
		{
			g_gird.clean_cell(g_vectex);
			g_gird.clean_path(g_vectex);
			g_start_point.init(0,0,0);
			g_end_point.init(0,0,0);
			g_surface = ENM_BLOCK_SURFACE;
			g_path_state = ENM_START_PATH_STATE;
			break;
		}
	}
}

//鼠标左键按下
void deal_left_button_down(int x,int y)
{
	switch(g_surface)
	{
	//画阻挡格子界面
	case ENM_BLOCK_SURFACE:
		{
			g_gird.click_point(x,y,g_vectex);
			break;
		}
	//寻路状态
	case ENM_FIND_PATH_SURFACE:
		{
			deal_path_state(x,y);
			break;
		}
	}
}

//鼠标右键按下
void deal_right_button_down()
{
    switch(g_surface)
    {
    //画阻挡格子界面
    case ENM_BLOCK_SURFACE:
        {
            g_gird.clean_cell(g_vectex);
            break;
        }
    //寻路界面
    case ENM_FIND_PATH_SURFACE:
        {
            g_gird.set_single_step(!g_gird.get_single_step());
            break;
        }
    }
}

LRESULT CALLBACK d3d::WndProc(HWND hwnd, UINT msg, WPARAM wParam, LPARAM lParam)
{
	switch( msg )
	{
	case WM_DESTROY:
		::PostQuitMessage(0);
		break;
	//按下键盘按键
	case WM_KEYDOWN:
		//Esc 关闭窗口
		if( wParam == VK_ESCAPE )
		{
			::DestroyWindow(hwnd);
		}
		//回车键
		if(wParam == VK_RETURN)
		{
			deal_return_key();
		}
		break;
	//鼠标左键按下
	case WM_LBUTTONDOWN:
		{
			int x = LOWORD(lParam);
			int y = HIWORD(lParam);
			deal_left_button_down(x,y);
			break;
		}
	//鼠标右键按下
	case WM_RBUTTONDOWN:
		{
			deal_right_button_down();
			break;
		}
	}
	return ::DefWindowProc(hwnd, msg, wParam, lParam);
}

bool Setup()
{
	// 创建直线对象
	if (FAILED(D3DXCreateLine(Device, &g_line)))
	{
		return false;
	}
	D3DXFONT_DESC font_desc;
	// create the font
	ZeroMemory(&font_desc, sizeof(font_desc));
	// set font descripter
	lstrcpyW(font_desc.FaceName, TEXT("Arial"));
	font_desc.Height = 32;
	// Creates a font object indirectly for both a device and a font
	D3DXCreateFontIndirect(Device, &font_desc, &g_font);

	// create the font
	ZeroMemory(&font_desc, sizeof(font_desc));
	// set font descripter
	lstrcpyW(font_desc.FaceName, TEXT("Arial"));
	font_desc.Height = FONT_SIZE;
	D3DXCreateFontIndirect(Device, &font_desc, &g_astar_font);

	D3DXMATRIX proj;
	D3DXMatrixPerspectiveFovLH(
		&proj,
		D3DX_PI * 0.5f, // 90 - degree
		(float)Width / (float)Height,
		1.0f,
		1000.0f);
	Device->SetTransform(D3DTS_PROJECTION, &proj);
	Device->SetRenderState(D3DRS_LIGHTING, false);
	int num = AstarX*AstarY*CELL_POINT_NUM*sizeof(vector3D);
	//创建顶点,用于画格子
	HRESULT hr = Device->CreateVertexBuffer(num , 0, D3D_FVF_VECTOR, D3DPOOL_SYSTEMMEM, &g_vectex, NULL);
	if (FAILED(hr)) {
		return false;
	}
	//初始化网格
	g_gird.init(AstarX,AstarY,CellSize,Width,Height);
	return true;
}

//绘画 direct
bool Display(float timeDelta)
{
	if(Device)
	{
		Device->Clear(0, 0, D3DCLEAR_TARGET | D3DCLEAR_ZBUFFER,0xdddddd/*D3DCOLOR_XRGB(43, 43, 43)*/, 1.0f, 0);
		Device->SetStreamSource(0, g_vectex, 0, sizeof(vector3D));
		Device->SetFVF(D3D_FVF_VECTOR);
		Device->BeginScene();
		g_gird.display(g_line,g_astar_font);
		int num = g_gird.get_cell_size();
		if(num > 0)
		{
			Device->DrawPrimitive(D3DPT_TRIANGLELIST,0,num*2);
		}
		draw_prompt_text();
		Device->EndScene();
		Device->Present(0, 0, 0, 0);
	}
	return true;
}

int WINAPI WinMain(HINSTANCE hinstance,
				   HINSTANCE prevInstance,
				   PSTR cmdLine,
				   int showCmd)
{
	if(!d3d::InitD3D(hinstance,Width, Height, true, D3DDEVTYPE_HAL, &Device))
	{
		::MessageBox(0, L"InitD3D() - FAILED", 0, 0);
		return 0;
	}
	if(!Setup())
	{
		::MessageBox(0, L"Setup() - FAILED", 0, 0);
		return 0;
	}
	d3d::EnterMsgLoop( Display );
	Device->Release();
	return 0;
}