#include "step_count.h"
#include "float.h"
#define filter_cnt 4

#define MAX(a,b) ((a) > (b) ? (a) : (b)) 
#define MIN(a,b) ((a) < (b) ? (a) : (b))
#define SAMPLE_SIZE   15
#define peak_max FLT_MAX

#define ABS(a) (a) > 0 ? (a) : -(a)
#define DYNAMIC_PRECISION     			0.8f     	 /*动态精度*/
 
#define MOST_ACTIVE_NULL      			0      	 /*未找到最活跃轴*/
#define MOST_ACTIVE_X					1		 /*最活跃轴X*/	
#define MOST_ACTIVE_Y					2        /*最活跃轴Y*/	
#define MOST_ACTIVE_Z					3        /*最活跃轴Z*/	
 
#define ACTIVE_PRECISION      			1.0f       /*活跃轴最小变化值*/

//用于均值滤波
typedef struct
{
    IMUdata avg[filter_cnt];
    unsigned char cnt;
}filter_avg_t;

//用于动态阈值即找出最活跃的一个轴
typedef struct {
	IMUdata newmax;
	IMUdata newmin;
	IMUdata oldmax;
	IMUdata oldmin;
}peak_value_t;

/*一个线性移位寄存器，用于过滤高频噪声*/
typedef struct slid_reg{
	IMUdata new_sample;
	IMUdata old_sample;
}slid_reg_t;

static struct {
    filter_avg_t filter;
    peak_value_t peak;
    slid_reg_t slid;
    uint32_t step_count;
} step_ctx;

static void Peakvalue_Init(peak_value_t *peak)
{
    peak->newmax.x = -peak_max;
    peak->newmax.y = -peak_max;
    peak->newmax.z = -peak_max;

    peak->newmin.x = peak_max;
    peak->newmin.y = peak_max;
    peak->newmin.z = peak_max;
}

//均值滤波
static void Filter_Mean(IMUdata *sample,filter_avg_t *filter)
{
    filter->avg[filter->cnt % filter_cnt] = *sample;
    filter->cnt++;

    float sum[3]={};
    for (size_t i = 0; i < filter_cnt; i++)
    {
        sum[0] += filter->avg[i].x;
        sum[1] += filter->avg[i].y;
        sum[2] += filter->avg[i].z; 
    }
    sample->x = sum[0]/filter_cnt;
    sample->y = sum[1]/filter_cnt;
    sample->z = sum[2]/filter_cnt;
}

//动态阈值
static void peak_update(peak_value_t *peak, IMUdata *cur_sample)
{
  	static unsigned int sample_size = 0;
    sample_size ++;
	if (sample_size > SAMPLE_SIZE) {
		/*采样达到50个，更新一次*/
		sample_size = 1;
		peak->oldmax = peak->newmax;
		peak->oldmin = peak->newmin;
      	//初始化
      	Peakvalue_Init(peak);
	}
	peak->newmax.x = MAX(peak->newmax.x, cur_sample->x);
	peak->newmax.y = MAX(peak->newmax.y, cur_sample->y);
	peak->newmax.z = MAX(peak->newmax.z, cur_sample->z);
	
	peak->newmin.x = MIN(peak->newmin.x, cur_sample->x);
	peak->newmin.y = MIN(peak->newmin.y, cur_sample->y);
	peak->newmin.z = MIN(peak->newmin.z, cur_sample->z);
}

static char slid_update(slid_reg_t *slid, IMUdata *cur_sample)
{
  	char res = 0;
    slid->old_sample.x = slid->new_sample.x;
    slid->old_sample.y = slid->new_sample.y;
    slid->old_sample.z = slid->new_sample.z;
  	if (ABS((cur_sample->x - slid->new_sample.x)) > DYNAMIC_PRECISION) 
    {
		slid->new_sample.x = cur_sample->x;
		res = 1;
	}
	if (ABS((cur_sample->y - slid->new_sample.y)) > DYNAMIC_PRECISION) 
    {
		slid->new_sample.y = cur_sample->y;
		res = 1;
	} 
	if (ABS((cur_sample->z - slid->new_sample.z)) > DYNAMIC_PRECISION) 
    {
		slid->new_sample.z = cur_sample->z;
		res = 1;
	}
	return res;
}

/*判断当前最活跃轴*/
static char is_most_active(peak_value_t *peak)
{
	char res = MOST_ACTIVE_NULL;
	float x_change = ABS((peak->newmax.x - peak->newmin.x));
	float y_change = ABS((peak->newmax.y - peak->newmin.y));
	float z_change = ABS((peak->newmax.z - peak->newmin.z));

	// ESP_LOGI("is_most_active","x_change:%f,y_change:%f,z_change:%f",x_change,y_change,z_change);
	if (x_change > y_change && x_change > z_change && x_change >= ACTIVE_PRECISION) 
    {
		res = MOST_ACTIVE_X;
	} 
    else if (y_change > x_change && y_change > z_change && y_change >= ACTIVE_PRECISION) 
    {
		res = MOST_ACTIVE_Y;
	} 
    else if (z_change > x_change && z_change > y_change && z_change >= ACTIVE_PRECISION) 
    {
		res = MOST_ACTIVE_Z;
	}
	return res;
}
 
// 获取QMI8658的加速度数据
static void get_filtered_accel(IMUdata *out) 
{
    Filter_Mean(out, &step_ctx.filter);  // 均值滤波
}

/*判断是否走步*/
static int detect_step(peak_value_t *peak, slid_reg_t *slid, IMUdata *cur_sample)
{
	static int step_cnt = 0;
	char res = is_most_active(peak);
	// ESP_LOGI("MOST_ACTIVE","peak->oldmax.x:%f,peak->oldmin.x:%f",peak->oldmax.x,peak->oldmin.x);
	// ESP_LOGI("MOST_ACTIVE","peak->oldmax.y:%f,peak->oldmin.y:%f",peak->oldmax.y,peak->oldmin.y);
	// ESP_LOGI("MOST_ACTIVE","peak->oldmax.z:%f,peak->oldmin.z:%f",peak->oldmax.z,peak->oldmin.z);
	switch (res) 
    {
		case MOST_ACTIVE_NULL: 
        {
			//fix
			break;
		}
		case MOST_ACTIVE_X: 
        {
			float range_x = peak->oldmax.x - peak->oldmin.x;
    		float threshold_x = peak->oldmin.x + range_x * 0.3f; // 动态范围40%位置
			if (slid->old_sample.x > threshold_x && slid->new_sample.x < threshold_x) 
            {
				step_cnt++;
			}
			break;
		}
		case MOST_ACTIVE_Y: 
        {
			float range_y = peak->oldmax.y - peak->oldmin.y;
    		float threshold_y = peak->oldmin.x + range_y * 0.3f; // 动态范围40%位置
			if (slid->old_sample.y > threshold_y && slid->new_sample.y < threshold_y) 
            {
				step_cnt++;
			}
			break;
		}
		case MOST_ACTIVE_Z: 
        {
			float range_z = peak->oldmax.z - peak->oldmin.z;
    		float threshold_z = peak->oldmin.x + range_z * 0.3f; // 动态范围40%位置
			if (slid->old_sample.z > threshold_z && slid->new_sample.z < threshold_z) 
            {
				step_cnt++;
			}
			break;
		}
		default: 
			break;
	}
	// ESP_LOGI("detect_step","步数：%d",step_cnt);
    return step_cnt;
}

// 计步模块初始化
void step_counter_init(void) 
{
    memset(&step_ctx, 0, sizeof(step_ctx));
    Peakvalue_Init(&step_ctx.peak);  // 初始化动态阈值
}

int step_counter_update(IMUdata *current_accel) 
{
    // 1. 获取滤波后数据
    get_filtered_accel(current_accel);
    
    // 2. 更新动态阈值
    peak_update(&step_ctx.peak, current_accel);
    
    // 3. 更新滑动窗口
    if(slid_update(&step_ctx.slid, current_accel)) 
    {
        // 4. 检测步伐
        return detect_step(&step_ctx.peak, &step_ctx.slid, current_accel);
    }
    return 0;
}
