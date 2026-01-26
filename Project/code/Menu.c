#include "zf_common_headfile.h"                 // Device header
//菜单选项数目
#define NUM_SELECTION		5

#define Main_Menu			0
#define Mode_1_Menu			1
#define Mode_2_Menu			2
#define Mode_3_Menu			3
#define Mode_4_Menu			4
#define Mode_5_Menu			5
uint8 Act_Menu = Main_Menu;

#define Browse_Mode			0
#define Edit_Mode			1
uint8 Act_Mode = Browse_Mode;

uint8 Cursor_Pos = 1;

void Show_Main_Menu(uint8 Cursor_Pos)
{
	OLED_Clear();
    if(Cursor_Pos != 5)
	{
		OLED_ShowChar(Cursor_Pos, 1, '>');

		OLED_ShowString(1, 2, "Mode_1");
		OLED_ShowString(2, 2, "Mode_2");
		OLED_ShowString(3, 2, "Mode_3");
		OLED_ShowString(4, 2, "Mode_4");

	}
	else
	{
		OLED_ShowChar(1, 1, '>');
		OLED_ShowString(1, 2, "Mode_5");
	}

}

void Show_Mode_1_Menu(void)
{
	OLED_Clear();
   
	if(Act_Mode == Edit_Mode)
	{
		OLED_ShowChar(1, 15, 'E');
	}

    OLED_ShowChar(Cursor_Pos, 1, '>');

}

void Show_Mode_2_Menu(void)
{
	OLED_Clear();
   
	if(Act_Mode == Edit_Mode)
	{
		OLED_ShowChar(1, 15, 'E');
	}

    OLED_ShowChar(Cursor_Pos, 1, '>');
	
}

void Show_Mode_3_Menu(void)
{
	OLED_Clear();
   
	if(Act_Mode == Edit_Mode)
	{
		OLED_ShowChar(1, 15, 'E');
	}

    OLED_ShowChar(Cursor_Pos, 1, '>');
	
}

void Show_Mode_4_Menu(void)
{
	OLED_Clear();
   
	if(Act_Mode == Edit_Mode)
	{
		OLED_ShowChar(1, 15, 'E');
	}

    OLED_ShowChar(Cursor_Pos, 1, '>');
	
}

void Show_Mode_5_Menu(void)
{
	OLED_Clear();
   
	if(Act_Mode == Edit_Mode)
	{
		OLED_ShowChar(1, 15, 'E');
	}

    OLED_ShowChar(Cursor_Pos, 1, '>');
	
}

void Handle_Key()
{
	Key_Tick();
	if(Act_Menu == Main_Menu)
	{
		if(Key_Check(KEY_NAME_UP,KEY_SINGLE))  // 上键
		{
			Cursor_Pos = (Cursor_Pos == 1) ? NUM_SELECTION : (Cursor_Pos - 1);
			Show_Main_Menu(Cursor_Pos);
		}
		else if(Key_Check(KEY_NAME_DOWN,KEY_SINGLE))  // 下键
		{
			Cursor_Pos = (Cursor_Pos == NUM_SELECTION) ? 1 : (Cursor_Pos + 1);
			Show_Main_Menu(Cursor_Pos);
		}
		else if(Key_Check(KEY_NAME_COMFIRM,KEY_SINGLE))  // 确认键
	    {	    
			if(Cursor_Pos == 1)
		    {
				Act_Menu = Mode_1_Menu;
				Show_Mode_1_Menu();
     	    }
     	    else if(Cursor_Pos == 2)
    	    {
    		    Act_Menu = Mode_2_Menu;
      	        Cursor_Pos = 2;
				Show_Mode_2_Menu();
     	    }
			else if(Cursor_Pos == 3)
    	    {
   	         	Act_Menu = Mode_3_Menu;
    	        Cursor_Pos = 2;
    	        Show_Mode_3_Menu();
    	    }
			else if(Cursor_Pos == 4)
    	    {
    	        Act_Menu = Mode_4_Menu;
    	        Cursor_Pos = 2;
				Show_Mode_4_Menu();
			}
			else if(Cursor_Pos == 5)
    	    {
    	        Act_Menu = Mode_5_Menu;
    	        Cursor_Pos = 2;
				Show_Mode_5_Menu();
			}
    	}
	}
	else if(Act_Menu == Mode_1_Menu)
	{
		if(Act_Mode == Browse_Mode)  // 浏览模式
		{
			if(Key_Check(KEY_NAME_UP,KEY_SINGLE))  // 上键
			{
				Cursor_Pos = (Cursor_Pos == 2) ? 4 : (Cursor_Pos - 1);
				Show_Mode_1_Menu();
			}
			else if(Key_Check(KEY_NAME_DOWN,KEY_SINGLE))  // 下键
			{
				Cursor_Pos = (Cursor_Pos == 4) ? 2 : (Cursor_Pos + 1);
				Show_Mode_1_Menu();
			}
			else if(Key_Check(KEY_NAME_COMFIRM,KEY_SINGLE))  // 确认键
			{
				Act_Mode = Edit_Mode;
				Show_Mode_1_Menu();
			}
			else if(Key_Check(KEY_NAME_BACK,KEY_SINGLE))  // 返回键
			{
				Act_Menu = Main_Menu;
				Cursor_Pos = 1;
				Show_Main_Menu(Cursor_Pos);
			}
		}
		else // 编辑模式
		{
			if(Key_Check(KEY_NAME_UP,KEY_SINGLE))  // 上键
			{
				//此处执行Mode_1编辑上键操作
				Show_Mode_1_Menu();
			}
			else if(Key_Check(KEY_NAME_DOWN,KEY_SINGLE))  // 下键
			{
				//此处执行Mode_1编辑下键操作
				Show_Mode_1_Menu();
			}
			else if(Key_Check(KEY_NAME_COMFIRM,KEY_SINGLE))  // 确认键
			{
				Act_Mode = Browse_Mode;
				Show_Mode_1_Menu();
			}
			else if(Key_Check(KEY_NAME_BACK,KEY_SINGLE))  // 返回键
			{
				Act_Mode = Browse_Mode;
				Show_Mode_1_Menu();
			}
		}
	}
	else if(Act_Menu == Mode_2_Menu)
	{
		if(Act_Mode == Browse_Mode)  // 浏览模式
		{
			if(Key_Check(KEY_NAME_UP,KEY_SINGLE))  // 上键
			{
				Cursor_Pos = (Cursor_Pos == 2) ? 4 : (Cursor_Pos - 1);
				Show_Mode_2_Menu();
			}
			else if(Key_Check(KEY_NAME_DOWN,KEY_SINGLE))  // 下键
			{
				Cursor_Pos = (Cursor_Pos == 4) ? 2 : (Cursor_Pos + 1);
				Show_Mode_2_Menu();
			}
			else if(Key_Check(KEY_NAME_COMFIRM,KEY_SINGLE))  // 确认键
			{
				Act_Mode = Edit_Mode;
				Show_Mode_2_Menu();
			}
			else if(Key_Check(KEY_NAME_BACK,KEY_SINGLE))  // 返回键
			{
				Act_Menu = Main_Menu;
				Cursor_Pos = 2;
				Show_Main_Menu(Cursor_Pos);
			}
		}
		else // 编辑模式
		{
			if(Key_Check(KEY_NAME_UP,KEY_SINGLE))  // 上键
			{
				//此处执行Mode_2编辑上键操作
				Show_Mode_2_Menu();
			}
			else if(Key_Check(KEY_NAME_DOWN,KEY_SINGLE))  // 下键
			{
				//此处执行Mode_2编辑下键操作
				Show_Mode_2_Menu();
			}
			else if(Key_Check(KEY_NAME_COMFIRM,KEY_SINGLE))  // 确认键
			{
				Act_Mode = Browse_Mode;
				Show_Mode_2_Menu();
			}
			else if(Key_Check(KEY_NAME_BACK,KEY_SINGLE))  // 返回键
			{
				Act_Mode = Browse_Mode;
				Show_Mode_2_Menu();
			}
		}
	}
	else if(Act_Menu == Mode_1_Menu)
	{
		if(Act_Mode == Browse_Mode)  // 浏览模式
		{
			if(Key_Check(KEY_NAME_UP,KEY_SINGLE))  // 上键
			{
				Cursor_Pos = (Cursor_Pos == 2) ? 4 : (Cursor_Pos - 1);
				Show_Mode_3_Menu();
			}
			else if(Key_Check(KEY_NAME_DOWN,KEY_SINGLE))  // 下键
			{
				Cursor_Pos = (Cursor_Pos == 4) ? 2 : (Cursor_Pos + 1);
				Show_Mode_3_Menu();
			}
			else if(Key_Check(KEY_NAME_COMFIRM,KEY_SINGLE))  // 确认键
			{
				Act_Mode = Edit_Mode;
				Show_Mode_3_Menu();
			}
			else if(Key_Check(KEY_NAME_BACK,KEY_SINGLE))  // 返回键
			{
				Act_Menu = Main_Menu;
				Cursor_Pos = 3;
				Show_Main_Menu(Cursor_Pos);
			}
		}
		else // 编辑模式
		{
			if(Key_Check(KEY_NAME_UP,KEY_SINGLE))  // 上键
			{
				//此处执行Mode_3编辑上键操作
				Show_Mode_3_Menu();
			}
			else if(Key_Check(KEY_NAME_DOWN,KEY_SINGLE))  // 下键
			{
				//此处执行Mode_3编辑下键操作
				Show_Mode_3_Menu();
			}
			else if(Key_Check(KEY_NAME_COMFIRM,KEY_SINGLE))  // 确认键
			{
				Act_Mode = Browse_Mode;
				Show_Mode_3_Menu();
			}
			else if(Key_Check(KEY_NAME_BACK,KEY_SINGLE))  // 返回键
			{
				Act_Mode = Browse_Mode;
				Show_Mode_3_Menu();
			}
		}
	}
	else if(Act_Menu == Mode_1_Menu)
	{
		if(Act_Mode == Browse_Mode)  // 浏览模式
		{
			if(Key_Check(KEY_NAME_UP,KEY_SINGLE))  // 上键
			{
				Cursor_Pos = (Cursor_Pos == 2) ? 4 : (Cursor_Pos - 1);
				Show_Mode_4_Menu();
			}
			else if(Key_Check(KEY_NAME_DOWN,KEY_SINGLE))  // 下键
			{
				Cursor_Pos = (Cursor_Pos == 4) ? 2 : (Cursor_Pos + 1);
				Show_Mode_4_Menu();
			}
			else if(Key_Check(KEY_NAME_COMFIRM,KEY_SINGLE))  // 确认键
			{
				Act_Mode = Edit_Mode;
				Show_Mode_4_Menu();
			}
			else if(Key_Check(KEY_NAME_BACK,KEY_SINGLE))  // 返回键
			{
				Act_Menu = Main_Menu;
				Cursor_Pos = 4;
				Show_Main_Menu(Cursor_Pos);
			}
		}
		else // 编辑模式
		{
			if(Key_Check(KEY_NAME_UP,KEY_SINGLE))  // 上键
			{
				//此处执行Mode_4编辑上键操作
				Show_Mode_4_Menu();
			}
			else if(Key_Check(KEY_NAME_DOWN,KEY_SINGLE))  // 下键
			{
				//此处执行Mode_4编辑下键操作
				Show_Mode_4_Menu();
			}
			else if(Key_Check(KEY_NAME_COMFIRM,KEY_SINGLE))  // 确认键
			{
				Act_Mode = Browse_Mode;
				Show_Mode_4_Menu();
			}
			else if(Key_Check(KEY_NAME_BACK,KEY_SINGLE))  // 返回键
			{
				Act_Mode = Browse_Mode;
				Show_Mode_4_Menu();
			}
		}
	}
	else if(Act_Menu == Mode_1_Menu)
	{
		if(Act_Mode == Browse_Mode)  // 浏览模式
		{
			if(Key_Check(KEY_NAME_UP,KEY_SINGLE))  // 上键
			{
				Cursor_Pos = (Cursor_Pos == 2) ? 4 : (Cursor_Pos - 1);
				Show_Mode_5_Menu();
			}
			else if(Key_Check(KEY_NAME_DOWN,KEY_SINGLE))  // 下键
			{
				Cursor_Pos = (Cursor_Pos == 4) ? 2 : (Cursor_Pos + 1);
				Show_Mode_5_Menu();
			}
			else if(Key_Check(KEY_NAME_COMFIRM,KEY_SINGLE))  // 确认键
			{
				Act_Mode = Edit_Mode;
				Show_Mode_5_Menu();
			}
			else if(Key_Check(KEY_NAME_BACK,KEY_SINGLE))  // 返回键
			{
				Act_Menu = Main_Menu;
				Cursor_Pos = 5;
				Show_Main_Menu(Cursor_Pos);
			}
		}
		else // 编辑模式
		{
			if(Key_Check(KEY_NAME_UP,KEY_SINGLE))  // 上键
			{
				//此处执行Mode_5编辑上键操作
				Show_Mode_5_Menu();
			}
			else if(Key_Check(KEY_NAME_DOWN,KEY_SINGLE))  // 下键
			{
				//此处执行Mode_5编辑下键操作
				Show_Mode_5_Menu();
			}
			else if(Key_Check(KEY_NAME_COMFIRM,KEY_SINGLE))  // 确认键
			{
				Act_Mode = Browse_Mode;
				Show_Mode_5_Menu();
			}
			else if(Key_Check(KEY_NAME_BACK,KEY_SINGLE))  // 返回键
			{
				Act_Mode = Browse_Mode;
				Show_Mode_5_Menu();
			}
		}
	}
}
