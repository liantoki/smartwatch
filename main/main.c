#include "ST77916.h"
#include "PCF85063.h"
#include "QMI8658.h"
#include "SD_MMC.h"
#include "Wireless.h"
#include "TCA9554PWR.h"

#include "BAT_Driver.h"
#include "PWR_Key.h"
#include "PCM5101.h"
#include "MIC_Speech.h"

#include "step_count.h"

QueueHandle_t     step_cnt;
// SemaphoreHandle_t accel_mutex;

QueueHandle_t     bat_queue;

QueueHandle_t     time_queue;
// SemaphoreHandle_t time_mutex;

//计步任务
void Step_Process_Task(void *parameter)
{
    IMUdata data;
    int     current_step;
    
    step_counter_init();
    TickType_t xLastWakeTime = xTaskGetTickCount();
    step_cnt                 = xQueueCreate(10,sizeof(int));
    // accel_mutex              = xSemaphoreCreateMutex();
    while (1)
    {
        if(QMI8658_Loop(&data))
        {
            current_step = step_counter_update(&data);
            if(current_step)
            {   
                ESP_LOGI("Step_Process_Task","步数：%d",current_step);
                xQueueSend(step_cnt,&current_step,0);
            }
        }

        vTaskDelayUntil(&xLastWakeTime,pdMS_TO_TICKS(20));
    }
    
}

void Power_Mgr(void *pvParams) 
{
    float bat_volts;
    bat_queue = xQueueCreate(10,sizeof(float));
    while(1) 
    {
        bat_volts = BAT_Get_Volts();  // 电池电压检测
        xQueueSend(bat_queue,&bat_volts,0);
        PWR_Loop();       // 电源状态管理
        vTaskDelay(pdMS_TO_TICKS(5000)); // 5秒周期
    }
}

void RTC_Task(void *pvParams) 
{
    datetime_t time;
    time_queue = xQueueCreate(10,sizeof(datetime));
    // time_mutex = xSemaphoreCreateMutex();
    // if (xSemaphoreTake(time_mutex,10))
    // {
        PCF85063_Init();
        // xSemaphoreGive(time_mutex);
    // }
    while(1) 
    {
        // if (xSemaphoreTake(time_mutex,10))
        // {
            PCF85063_Read_Time(&time);
            // xSemaphoreGive(time_mutex);
        // }
        xQueueSend(time_queue,&time,0);
        vTaskDelay(pdMS_TO_TICKS(1000)); // 1秒周期
    }
}

void Driver_Init(void)
{
    PWR_Init();
    BAT_Init();
    I2C_Init();
    EXIO_Init();                    // Example Initialize EXIO
    Flash_Searching();
    Wireless_Init();
    QMI8658_Init();
    
    xTaskCreate(Step_Process_Task, "Step Task", 4096, NULL, 4, NULL);
    xTaskCreate(RTC_Task, "RTC", 2048, NULL, 2, NULL);
    xTaskCreate(Power_Mgr, "Power", 2048, NULL, 1, NULL);
}
void app_main(void)
{
    Driver_Init();
    // Wireless_Init();
    SD_Init();
    LCD_Init();
    Audio_Init();
    // MIC_Speech_init();
    // Play_Music("/sdcard","AAA.mp3");
    LVGL_Init();   // returns the screen object

// /********************* Demo *********************/
    // Lvgl_Example1();
    // lv_demo_widgets();
    // lv_demo_keypad_encoder();
    // lv_demo_benchmark();
    // lv_demo_stress();
    // lv_demo_music();

    while (1) 
    {
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}






