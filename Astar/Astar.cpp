// Astar.cpp : �������̨Ӧ�ó������ڵ㡣
//
#include "stdafx.h"
#include "d3dUtility.h"
#include "gird.h"

#define D3D_FVF_VECTOR (D3DFVF_XYZRHW | D3DFVF_DIFFUSE)

//���ڴ�С
const int Width  = 1600;
const int Height = 900;

//A* �������
const int AstarX = 15;
const int AstarY = 10; 
//�赲���Ӵ�С
const int CellSize = 75;

//Direct3D�߶���
IDirect3DDevice9* Device = NULL;
//����
IDirect3DVertexBuffer9 *g_vectex = NULL;
//�ı�����
ID3DXFont* g_font = NULL;
//a* ���� f��g��h ֵ����
ID3DXFont* g_astar_font = NULL;
//ֱ��
LPD3DXLINE g_line = NULL;

//����
gird g_gird;

//��������
SurfaceType g_surface = ENM_BLOCK_SURFACE;
//Ѱ·״̬
PathState g_path_state = ENM_START_PATH_STATE;

//Ѱ·��ʼ��
vector3D g_start_point;
//Ѱ·�յ�
vector3D g_end_point;

//�赲ǽ�����ı�
WCHAR g_block_prompt[256] = {
	L"�������������赲ǽ  ���س�����Ѱ·���� ���Ҽ�����赲����"
};

//Ѱ·�����ı�
WCHAR g_path_prompt[3][256] = {
	{L"��������������ʼѰ·��  ��ǰ[%s]Ѱ·���Ҽ��л�״̬  ���س����������赲ǽ����"},
	{L"�������������յ�Ѱ·��  ��ǰ[%s]Ѱ·���Ҽ��л�״̬  ���س����������赲ǽ����"},
	{L"�������������Ѱ·  ��ǰ[%s]Ѱ·״�Ҽ��л�״̬  ���س����������赲ǽ����"}
};

//Ѱ·״̬
void deal_path_state(int x,int y)
{
	switch(g_path_state)
	{
	//Ѱ·���
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
	//Ѱ·�յ�
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
			//����Ѱ·
			if(g_gird.get_single_step())
			{
				g_path_state = ENM_SINGLE_STEP_STATE;
				deal_path_state(0,0);
			}
			//һ��Ѱ·
			else
			{
				g_gird.find_path(g_end_point);
				g_start_point.init(0,0,0);
				g_end_point.init(0,0,0);
				g_path_state = ENM_START_PATH_STATE;
			}
			break;
		}
	//����Ѱ·
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

//���Ѱ·ִ��״̬�ı�
WCHAR* get_path_execute_state()
{
	static WCHAR temp[256];
	if(g_gird.get_single_step())
	{
		swprintf(temp,L"����");
	}
	else
	{
		swprintf(temp,L"һ��");
	}
	return temp;
}

//�ı���ʾ
void draw_prompt_text()
{
	RECT rect = {Width/2 - 700,6,Width/2 + 650,40};
	WCHAR *str = NULL;
	switch(g_surface)
	{
	//���赲���ӽ���
	case ENM_BLOCK_SURFACE:
		{
			str = g_block_prompt;
			break;
		}
	//Ѱ·
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

//�س�������
void deal_return_key()
{
	switch(g_surface)
	{
	//���赲���ӽ���
	case ENM_BLOCK_SURFACE:
		{
			g_surface = ENM_FIND_PATH_SURFACE;
			break;
		}
	//Ѱ·����
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

//����������
void deal_left_button_down(int x,int y)
{
	switch(g_surface)
	{
	//���赲���ӽ���
	case ENM_BLOCK_SURFACE:
		{
			g_gird.click_point(x,y,g_vectex);
			break;
		}
	//Ѱ·״̬
	case ENM_FIND_PATH_SURFACE:
		{
			deal_path_state(x,y);
			break;
		}
	}
}

//����Ҽ�����
void deal_right_button_down()
{
    switch(g_surface)
    {
    //���赲���ӽ���
    case ENM_BLOCK_SURFACE:
        {
            g_gird.clean_cell(g_vectex);
            break;
        }
    //Ѱ·����
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
	//���¼��̰���
	case WM_KEYDOWN:
		//Esc �رմ���
		if( wParam == VK_ESCAPE )
		{
			::DestroyWindow(hwnd);
		}
		//�س���
		if(wParam == VK_RETURN)
		{
			deal_return_key();
		}
		break;
	//����������
	case WM_LBUTTONDOWN:
		{
			int x = LOWORD(lParam);
			int y = HIWORD(lParam);
			deal_left_button_down(x,y);
			break;
		}
	//����Ҽ�����
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
	// ����ֱ�߶���
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
	//��������,���ڻ�����
	HRESULT hr = Device->CreateVertexBuffer(num , 0, D3D_FVF_VECTOR, D3DPOOL_SYSTEMMEM, &g_vectex, NULL);
	if (FAILED(hr)) {
		return false;
	}
	//��ʼ������
	g_gird.init(AstarX,AstarY,CellSize,Width,Height);
	return true;
}

//�滭 direct
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