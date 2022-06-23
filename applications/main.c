/*
 * Copyright (c) 2006-2020, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2020-09-02     RT-Thread    first version
 */

#include "stm32h7xx.h"
#include <rtthread.h>
#include <rtdevice.h>
#include "drv_common.h"
#include "drv_spi_ili9488.h"  // spi lcd driver 
#include <lcd_spi_port.h>  // lcd ports
#include <rt_ai_person_yolo_model.h>
#include <rt_ai.h>
#include <backend_cubeai.h>
#include<stdio.h>
#include<stdlib.h>
#include<string.h>
//#include <img5_jpeg_flt.h>
//#include <img5_jpeg_int.h>
#include <yolo_layer.h>

#define LED_PIN GET_PIN(I, 8)
#define PWM_PIN GET_PIN(B,1)
#define THREAD_PRIORITY         2
#define THREAD_STACK_SIZE       512
#define THREAD_TIMESLICE        5

#define N 3//字符串的大小

#define SAMPLE_UART_NAME       "uart1"    /* 串口设备名称 */
static rt_device_t serial;                /* 串口设备句柄 */
struct serial_configure config = RT_SERIAL_CONFIG_DEFAULT;  /* 初始化配置参数 */

struct rt_event ov2640_event;
rt_uint8_t *input_gray;
rt_uint8_t *input_gray_160;
float *input_buf;
rt_uint8_t ai_flag = 0;

extern void DCMI_Start();
extern int rt_gc0328c_init();
static inline void _itof(float *dst,rt_uint8_t *src, uint32_t size, float div);
void rgb2gray(unsigned char *src,unsigned char *dst, int width,int height);
void bilinera_interpolation(rt_uint8_t* in_array, short height, short width, 
                            rt_uint8_t* out_array, short out_height, short out_width);
void thread_serial(int x1,int y1,int x2,int y2,int x3,int y3);
//void reverse(char *a,int len);
//void int2string(int b, char *c);
void servo_open(void);//机械爪打开
void servo_close(void);//机械爪关闭
yolo_region_layer r1;
void ai_camera();
int detected=1;//摄像头检测到目标的标志位
int flag;//树莓派检测成功的标志位，char类型
char useful_detect[] = "0";//储存树莓派传输的数据，如果是1即为有效，如果为0为无效
int main(void)
{
    rt_pin_mode(LED_PIN, PIN_MODE_OUTPUT);

    input_gray = rt_malloc(320*240);
    input_gray_160 = rt_malloc(160*160);
    input_buf = rt_malloc(160*160*sizeof(float));
    
    /* init spi data notify event */
    rt_event_init(&ov2640_event, "ov2640", RT_IPC_FLAG_FIFO);
    struct drv_lcd_device *lcd;
    lcd = (struct drv_lcd_device *)rt_device_find("lcd");
    struct rt_device_rect_info rect_info = {0, 0, LCD_WIDTH, 240};
    lcd->parent.control(&lcd->parent, RTGRAPHIC_CTRL_RECT_UPDATE, &rect_info);

    // find ai model handle
    rt_ai_t person_d = NULL;
    person_d = rt_ai_find(RT_AI_PERSON_YOLO_MODEL_NAME);
    if(!person_d) {rt_kprintf("ai find err\n"); return -1;}
    // alloc calculate mem
    rt_ai_buffer_t *work_buf = rt_malloc(RT_AI_PERSON_YOLO_WORK_BUFFER_BYTES);
    if(!work_buf) {rt_kprintf("malloc err\n");return -1;}
    // ai out
    rt_ai_buffer_t *_out = rt_malloc(RT_AI_PERSON_YOLO_OUT_1_SIZE_BYTES);
    if(!_out) {rt_kprintf("malloc err\n"); return -1;}
    
    if(rt_ai_init(person_d,work_buf) != 0){rt_kprintf("ai init err\n"); return -1;}
    rt_ai_config(person_d,CFG_INPUT_0_ADDR,(rt_ai_buffer_t*)input_buf);
    rt_ai_config(person_d,CFG_OUTPUT_0_ADDR,_out);
    
    // yolo_box
    yolo_box *boxs = NULL;
    yolo_box *p  = NULL;
    yolo_region_layer_init(&r1,125,0.2,1,5);
    int _x1 = 0;
    int _y1 = 0;
    int _x2 = 0;
    int _y2 = 0;
//    int Position[6];
    rt_pin_mode(PWM_PIN, PIN_MODE_OUTPUT);
    ai_camera();

    while(1){
        rt_pin_write(LED_PIN, PIN_LOW);
        rt_event_recv(&ov2640_event,
                        1,
                        RT_EVENT_FLAG_AND | RT_EVENT_FLAG_CLEAR,
                        RT_WAITING_FOREVER,
                        RT_NULL);
        rt_pin_write(LED_PIN, PIN_HIGH);
        lcd->parent.control(&lcd->parent, RTGRAPHIC_CTRL_RECT_UPDATE, &rect_info);
        if(ai_flag > 0){
//          rt_kprintf("ai start");
            rgb2gray(lcd->lcd_info.framebuffer ,input_gray, 320,240);
            bilinera_interpolation(input_gray, 240, 320, input_gray_160, 160, 160);
            _itof(input_buf,input_gray_160, 160*160, 255.0);
            rt_ai_run(person_d, NULL, NULL);
            boxs = (yolo_box *)rt_ai_output(person_d,0);
            yolo_decode((float*) boxs);
            do_nms_sort(&r1,boxs);
            p = boxs;
            for(int i=0;i<125;i++){
                    p = &boxs[i] ; 
                    if((p->class_score * p->objectness)>0.2){
                    _x1 = (int)(p->x*320 - (p->w*320*0.5));  _x1 = _x1>0 ? _x1:1; _x1 = _x1<320 ? _x1:319;  
                    _y1 = (int)(p->y*240 - (p->h*160*1.5*0.5)); _y1 = _y1>0 ? _y1:1; _y1 = _y1 < 240 ? _y1:239;
                    _x2 = (int)(p->x*320 + (p->w*320*0.5)); _x2 = _x2>0 ? _x2:1; _x2 = _x2<320 ? _x2:319;
                    _y2 = (int)(p->y*240 + (p->h*160*1.5*0.5)); _y2 = _y2>0 ? _y2:1; _y2 = _y2 < 240 ? _y2:239;
                    lcd_draw_rectangle(_x1, _y1, _x2, _y2);
                    int x=(_x1+_x2)/2;
                    int y=(_y1+_y2)/2;
                   //rt_kprintf("%d %d %d %d %d %d \r\n", _x1,_y1,_x2,_y2,x,y);
                    thread_serial(_x1,_y1,_x2,_y2,x,y);
//                    receive_data=rt_device_read(serial, 0, &flag, (sizeof(flag)-1));
                    rt_kprintf("%s \r\n",useful_detect);
                    if(useful_detect[0]=='1')
                    {
                        servo_open();
                        rt_thread_mdelay(1000);
                        servo_close();

                    }
                    break;
                }
            }
        DCMI_Start();
        }  
    }
    
    return RT_EOK;
}


static int vtor_config(void)
{
   /* Vector Table Relocation in Internal QSPI_FLASH */
   SCB->VTOR = QSPI_BASE;
   return 0;
}
INIT_BOARD_EXPORT(vtor_config);

/* 串口数据发送函数 */
void thread_serial(int x1,int y1,int x2,int y2,int x3,int y3)
{
//        char X1[3],Y1[3],X2[3],Y2[3],X3[3],Y3[3];//int转数组
        /* step1：查找串口设备 */
        serial = rt_device_find(SAMPLE_UART_NAME);
        int sign=1010;
        /* step2：修改串口配置参数 */
        config.baud_rate = BAUD_RATE_9600;        //修改波特率为 9600
        config.data_bits = DATA_BITS_8;           //数据位 8
        config.stop_bits = STOP_BITS_1;           //停止位 1
        config.bufsz     = 128;                   //修改缓冲区 buff size 为 128
        config.parity    = PARITY_NONE;           //无奇偶校验位

        /* step3：控制串口设备。通过控制接口传入命令控制字，与控制参数 */
        rt_device_control(serial, RT_DEVICE_CTRL_CONFIG, &config);

        /* step4：打开串口设备。以中断接收及轮询发送模式打开串口设备 */
        rt_device_open(serial, RT_DEVICE_FLAG_INT_RX);
        /* 发送数据 */
//        rt_device_write(serial, 0, &sign, (sizeof(sign)));
//        rt_device_write(serial, 0, &sign, (sizeof(sign)));
//        rt_device_write(serial, 0, &x1, (sizeof(x1)));
//        rt_device_write(serial, 0, &y1, (sizeof(y1)));
//        rt_device_write(serial, 0, &x2, (sizeof(x2)));
//        rt_device_write(serial, 0, &y2, (sizeof(y2)));
//        rt_device_write(serial, 0, &x3, (sizeof(x3)));
//        rt_device_write(serial, 0, &y3, (sizeof(y3)));
        rt_device_write(serial, 0, &detected, (sizeof(detected)));
        /*接收数据*/
        flag=rt_device_read(serial, 0, useful_detect, (sizeof(useful_detect)));
}

void ai_camera()
{
    rt_gc0328c_init();
    ai_flag = 1;
    DCMI_Start();
}
// MSH_CMD_EXPORT(ai_camera, Start the AI camera to recognize person);

static inline void _itof(float *dst,rt_uint8_t *src, uint32_t size, float div){
    if (div == 0){
        return ;
    }else{
        int i = 0;
        for(i = 0; i < size; i++){
            dst[i] = (float)src[i] / div;
        }
    }
}

// img covnert to gray: Gray = 0.2989*R + 0.5870*G + 0.1140*B
// better: 4898*R + 9618*G + 1868*B >> 14
// int8: 76*R + 150*G + 30*B >> 8
void rgb2gray(unsigned char *src, unsigned char *dst, int width, int height)
{
    int r, g, b;
    for (int i=0; i<width*height; ++i)
    {
        r = *src++; // load red
        g = *src++; // load green
        b = *src++; // load blue
        // build weighted average:
        *dst++ = (r * 76 + g * 150 + b * 30) >> 8;
    }
}

int is_in_array(short x, short y, short height, short width)
{
    if (x >= 0 && x < width && y >= 0 && y < height)
        return 1;
    else
        return 0;
}

void bilinera_interpolation(rt_uint8_t* in_array, short height, short width, 
                            rt_uint8_t* out_array, short out_height, short out_width)
{
    double h_times = (double)out_height / (double)height,
           w_times = (double)out_width / (double)width;
    short  x1, y1, x2, y2, f11, f12, f21, f22;
    double x, y;

    for (int i = 0; i < out_height; i++){
        for (int j = 0; j < out_width; j++){
            x = j / w_times;
            y = i / h_times;
          
            x1 = (short)(x - 1);
            x2 = (short)(x + 1);
            y1 = (short)(y + 1);
            y2 = (short)(y - 1);
            f11 = is_in_array(x1, y1, height, width) ? in_array[y1*width+x1] : 0;
            f12 = is_in_array(x1, y2, height, width) ? in_array[y2*width+x1] : 0;
            f21 = is_in_array(x2, y1, height, width) ? in_array[y1*width+x2] : 0;
            f22 = is_in_array(x2, y2, height, width) ? in_array[y2*width+x2] : 0;
            out_array[i*out_width+j] = (rt_uint8_t)(((f11 * (x2 - x) * (y2 - y)) +
                                       (f21 * (x - x1) * (y2 - y)) +
                                       (f12 * (x2 - x) * (y - y1)) +
                                       (f22 * (x - x1) * (y - y1))) / ((x2 - x1) * (y2 - y1)));
        }
    }
}

void servo_open(void){
    for(int i=0;i<10;i++){
        rt_thread_mdelay(18.1);
        rt_pin_write(PWM_PIN, PIN_HIGH);
        rt_thread_mdelay(1.9);
        rt_pin_write(PWM_PIN, PIN_LOW);
    }
}//控制舵机打开

void servo_close(void){
    for(int i=0;i<10;i++){
        rt_thread_mdelay(18);
        rt_pin_write(PWM_PIN, PIN_HIGH);
        rt_thread_mdelay(2);
        rt_pin_write(PWM_PIN, PIN_LOW);
    }
}//控制舵机关闭
