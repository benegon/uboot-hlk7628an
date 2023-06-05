/*************************************************************************
	> File Name: led.c
	> Author: 
	> Mail: 
	> Created Time: 2015年04月11日 星期六 15时22分19秒
 ************************************************************************/
#include <led.h>
#include <rt_mmap.h>

#include <configs/rt2880.h>

void init_power_led(void)
{
	RALINK_REG(RALINK_SYSCTL_BASE+0x64) = 0x1;
	
	RALINK_REG(RALINK_SYSCTL_BASE+0x604) = (1<<12);
    //GPIO#44
    
	RALINK_REG(RALINK_SYSCTL_BASE+0x634) = (1<<12);
	//RALINK_REG(RALINK_SYSCTL_BASE+0x644) = (1<<12);
	//RALINK_REG(RALINK_SYSCTL_BASE+0x624) = (1<<12);
}

//升级时 控制闪烁
void control_power_led(int flag)//写0熄灭 写1点亮
{
	//printf("##### GPIO MODE2 :%x Flag :%d #####\n", RALINK_REG(RALINK_SYSCTL_BASE+0x64), flag);
	if (flag == 1)
		RALINK_REG(RALINK_SYSCTL_BASE+0x634) = (1<<12);
	else
		RALINK_REG(RALINK_SYSCTL_BASE+0x644) = (1<<12);
}


