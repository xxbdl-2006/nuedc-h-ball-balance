#include "timer.h"
#include "encoder.h"
#include "bno08x_uart_rvc.h"
#include "pid_control.h"

// 定义状态枚举
typedef enum {
    STATE_TRACKING,               // 正常循迹状态
    STATE_MOVE_AFTER_DETECTION,       // 转向微调找黑线
    STATE_TURN_RIGHT,
    STATE_STOP,
} FirstTaskState;
typedef enum {
    STATE_AUTO,
    STATE_SELF,
} SecondTaskState;
SecondTaskState second_task_state = STATE_AUTO;
FirstTaskState first_task_state = STATE_TRACKING;  // 初始状态

int circle_count = 0;          // 当前完成圈数
int target_circles = 1;        // 目标圈数(可在1-5之间设置)
int move_after_detection = 0;  // 检测后移动计数
int TURN_DURATION=10;
int MOVE_DURATION=14;  // 向前走的时长
int left_three_detected = 0;   // 左侧三线检测标志
int task=1;
int start_flag=0;    //是否开启
int detection_count=0;
int angleturn_flag=0;
int angleturn_count=0;
uint16_t last_count = 0;
int turn_timer=0;
uint8_t initial_search_direction = 0;  // 可根据需要设置为0或1
// 在结构体或全局变量中添加
uint8_t y_error_count = 0;      // 用于计数y轴误差小于10的次数
uint8_t y_pid_disabled = 0;     // y轴PID控制是否已禁用的标志位
// 全局变量：用于激光定时控制（放在函数外部定义）
uint16_t laser_timer = 0;  // 50ms计数变量，单位：50ms
uint16_t timer = 0;
uint16_t servo_timer = 0;
uint8_t laser_state = 0;   // 激光状态：0-关闭，1-打开

uint16_t task2_laser_timer = 0;  // 50ms计数单位，0.8s = 16个单位
uint8_t task2_laser_opened = 0;  // 激光是否已打开的标志


void timer_init(void)
{
	/* 模块全部初始化完成后，再开放 50ms 控制定时器中断。 */
	NVIC_ClearPendingIRQ(TIMER_TICK_INST_INT_IRQN);
	NVIC_EnableIRQ(TIMER_TICK_INST_INT_IRQN);
}


void TIMER_TICK_INST_IRQHandler(void)
{
	if( DL_TimerA_getPendingInterrupt(TIMER_TICK_INST) == DL_TIMER_IIDX_ZERO )
	{
		/* 50ms 快照一次编码器增量，再执行一轮三 PID 控制。 */
		encoder_update();
		pid_control_update(bno08x_data.yaw,
			get_encoder_count_L(), get_encoder_count_R());

   }
}



