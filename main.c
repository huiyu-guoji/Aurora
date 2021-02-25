/*********************************************************************************************************************
* @file				main
* @author			auraro
* @version			查看doc内version文件 版本说明
* @Software			IAR 8.3 or MDK 5.24
* @Target core		MM32SPIN2XPs
* @date				2020-2-20
********************************************************************************************************************/

#include "headfile.h"
#include "math.h"
// **************************** 宏定义 ****************************

// 
// **************************** 宏定义 ****************************

// **************************** 宏定义 ****************************

// **************************** 宏定义 ****************************
#define  range_gyro  2000
#define  range_acc   8
#define  rangeadc    32767
#define  time1       0.003
#define  pi          3.1415926
#define  error       0.2343
// **************************** 变量定义 ****************************
float angler_x=0;
float angler_y=0;
float angler_z=0;
float anglea_x=0;
float anglea_y=0;
float anglea_z=0;
float racc_x;
float racc_y;
float racc_z;
float racc;
float rgyro_x;
float rgyro_y;
float rgyro_z;
// **************************** 变量定义 ****************************
bool interrupt_flag = false;
float angle;
float Q_bias;
float angle_hubu;
// **************************** 代码区域 ****************************
// **************************** 函数声明 ****************************
void kalman_filter (float new_angle,float new_gyro);
float complementary_filter (float new_anglea,float new_gyro,float k);
float jiaquan(float a,float b,float k);



// **************************** 主函数 ****************************
int main(void)
{
	board_init(true);																// 初始化 debug 输出串口

	//此处编写用户代码(例如：外设初始化代码等)
	icm20602_init_spi();															// 初始化硬件SPI接口的 ICM20602
    tim_interrupt_init(TIM_1, 250, 1);

	while(1)
	{

		get_icm20602_accdata_spi();													// 获取ICM20602的测量数值
		get_icm20602_gyro_spi();													// 获取ICM20602的测量数值
//		printf("\r\nICM20602 acc data: x-%d, y-%d, z-%d", icm_acc_x, icm_acc_y, icm_acc_z);
//	    printf("\r\nICM20602 gyro data: x=  %d", rgyro_x);
//		systick_delay_ms(1000);
		//此处编写需要循环执行的代码
        rgyro_x=((double)icm_gyro_x/rangeadc)*range_gyro+error;          //计算角速度
        rgyro_y=((double)icm_gyro_y/rangeadc)*range_gyro+error; 
        rgyro_z=((double)icm_gyro_z/rangeadc)*range_gyro+error; 
   
// **************************** 加速度计 ****************************        
        racc_x=((double)icm_acc_x/rangeadc)*range_acc;
        racc_y=((double)icm_acc_y/rangeadc)*range_acc;
        racc_z=((double)icm_acc_z/rangeadc)*range_acc;             //计算分加速度
        racc=sqrt(racc_x*racc_x+racc_y*racc_y+racc_z*racc_z);
        anglea_x=(float)acos(racc_x/racc)/pi*180-90;
        anglea_y=(float)acos(racc_y/racc)/pi*180-90;
        anglea_z=(float)acos(racc_z/racc)/pi*180-90;
// **************************** 虚拟示波器****************************
//        software_float(angler_x);
//      software_2_float(angle_hubu, anglea_x);
      float swear[4];
      swear[0]=angle;
      swear[1]=angle_hubu;
      swear[2]=complementary_filter(angle,angle_hubu,0.5);
      swear[3]=anglea_x;
      software_more_float(4,swear);
// **************************** 中断区域 *****************************
       if(interrupt_flag)
       {
           interrupt_flag=false;         
         /*  angler_x+=(double)(rgyro_x*time1);
           angler_y+=(double)(rgyro_y*time1);
           angler_z+=(double)(rgyro_z*time1);
         */
           kalman_filter(anglea_x,rgyro_y);
           angle_hubu=complementary_filter (anglea_x,rgyro_y,0.38);
       }             
	}
}
// **************************** 代码区域 ****************************


//-------------------------------------------------------------------------------------------------------------------
// @brief		卡尔曼滤波
// @param		加速度计测量值   陀螺仪的值 
// @return		void
//-------------------------------------------------------------------------------------------------------------------

void kalman_filter (float new_angle,float new_gyro)
{
// **************************** 变量定义 ****************************
    float Q_angle=0.001;
    float Q_gyro=0.003;
    float R_angle=0.03;
    float dt=0.001;
    float p[2][2]={1,1,1,1};
    float measure;
    extern float angle;
    extern float Q_bias;
    float ko;
    float ki;
    int a=1;
 // **************************** 代码区域 ****************************
    if(a)
    {
        angle=new_angle;
        a=0;
    }
    angle= angle-Q_bias*dt+new_gyro*dt;//先验估计
    
    p[0][0]=p[0][0]+Q_angle-(p[0][1]-p[1][0])*dt;
    p[0][1]=p[0][1]-p[1][1]*dt;
    p[1][0]=p[1][0]-p[1][1]*dt;
    p[1][1]=p[1][1]+Q_gyro;            //协方差矩阵
    
    measure=new_angle;//观测方程
    
    ko=p[0][0]/(p[0][0]+R_angle);
    ki=p[1][0]/(p[0][0]+R_angle);      //kalman增益
   // printf("ko=%f\n",ko);
    angle=angle+ko*(new_angle-angle);
    Q_bias=Q_bias+ki*(new_angle-angle);
   // printf("angle=%f\n",angle);
    p[0][0]=p[0][0]-ko*p[0][0];
    p[0][1]=p[0][1]-ko*p[0][1];
    p[1][0]=p[1][0]-ki*p[1][0];
    p[1][1]=p[1][1]-ki*p[1][1];         //更新协方差矩阵
    
   // new_gyro-=Q_bias;
 
}

//-------------------------------------------------------------------------------------------------------------------
// @brief		互补滤波
// @param		加速度计测量值   陀螺仪的值 
// @return		float
//-------------------------------------------------------------------------------------------------------------------
float complementary_filter (float new_anglea,float new_gyro,float k)
{
     int a=1;
     float final_angle;
     float temp_angle; 
     float error_angle=0;    
     float dt=0.03;    
     if(a)
    {
        final_angle=angler_y;
        a=0;
        temp_angle=angler_y;
    }
  
        temp_angle+=(new_gyro+error_angle)*dt;
        error_angle=(new_anglea-final_angle)*k;
        final_angle=k*new_anglea+(1-k)*temp_angle;
         
      return final_angle;
}

//-------------------------------------------------------------------------------------------------------------------
// @brief		加权函数
// @param		任意值 
// @return		最终值
//-------------------------------------------------------------------------------------------------------------------
float jiaquan(float a,float b,float k)
{
    float final;
    final=k*a+(1-k)*b;
    return final;
}