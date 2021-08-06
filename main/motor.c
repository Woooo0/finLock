#include <stdio.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_spi_flash.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "driver/gpio.h"

#include "esp_log.h"

#define GPIO_LED_NUM 2 
#define FIN_LOCK_CTRL 26 

#define UART_NUM1        	(UART_NUM_1)//端口
#define BUF_SIZE            (256)//缓存,大于128byte
#define UART1_RX_PIN      	(17)//关联引脚
#define UART1_TX_PIN      	(16)
#define UART1_RTS_PIN  		(UART_PIN_NO_CHANGE)
#define UART1_CTS_PIN  		(UART_PIN_NO_CHANGE)

#define PG_GET 0
#define GPIO_INPUT_IO_0     25
#define GPIO_INPUT_PIN_SEL  ((1ULL<<GPIO_INPUT_IO_0)|(1ULL<<PG_GET))
#define ESP_INTR_FLAG_DEFAULT 0

static xQueueHandle gpio_evt_queue = NULL;

//获取图像
static void PS_GetImage(void)
{
    char PS_GetImage_HEX[12] = {0xEF,0x01,0xFF,0xFF,0xFF,0xFF,0x01,0x00,0x03,0x01,0x00,0x05}; //录入图像
    char *UART_PS_GetImage = PS_GetImage_HEX;
    uart_write_bytes(UART_NUM1, (const char *)UART_PS_GetImage, 12);
}
//生成特征
static void PS_GenChar(int bufferID)
{
    char PS_GenChar_HEX[13] = {0xEF,0x01,0xFF,0xFF,0xFF,0xFF,0x01,0x00,0x04,0x02,bufferID,0x00,0x08}; //生成特征
    char *UART_PS_GenChar = PS_GenChar_HEX;
}

//合并特征
static void PS_RegModel(void)
{
    char PS_RegModel_HEX[12] = {0xEF,0x01,0xFF,0xFF,0xFF,0xFF,0x01,0x00,0x03,0x05,0x00,0x09};  //合并特征
    char *UART_PS_RegMode = PS_RegModel_HEX;
    uart_write_bytes(UART_NUM1, (const char *)UART_PS_RegMode, 12);
}

//指纹模板注册
static void PS_EnrollModel(void)
{
    PS_GetImage();
}

//自动验证
static void PS_AutoIdentify(void)
{
    char PS_AutoIdentify_HEX[17] = {0xEF,0x01,0xFF,0xFF,0xFF,0xFF,0x01,0x00,0x08,0x32,0x01,0xFF,0xFF,0x00,0x04,0x02,0x3E}; //自动验证
    char *UART_PS_AutoIdentify = PS_AutoIdentify_HEX;
    uart_write_bytes(UART_NUM1, (const char *)UART_PS_AutoIdentify, 17);
}
 
static void IRAM_ATTR gpio_isr_handler(void* arg)
{
    uint32_t gpio_num = (uint32_t) arg;
    xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
}

static void gpio_task_example(void* arg)
{
    uint32_t io_num;
    for(;;) {
        if(xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY)) {
            switch (io_num)
            {
            case 0:
                printf("%s\n","开始录入指纹");
                break;

            case 25:
                printf("%s\n","开始验证");
                PS_AutoIdentify();
                break;   
            
            default:
                break;
            }
        }
    }
}

void motor(void)
{
    gpio_config_t io_conf;

    //interrupt of rising edge
    io_conf.intr_type = GPIO_INTR_POSEDGE;
    //bit mask of the pins, use GPIO4/5 here
    io_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL;
    //set as input mode
    io_conf.mode = GPIO_MODE_INPUT;
    //enable pull-up mode
    io_conf.pull_up_en = 1;
    gpio_config(&io_conf);

    //change gpio intrrupt type for one pin
    gpio_set_intr_type(GPIO_INPUT_IO_0, GPIO_INTR_POSEDGE);
    gpio_set_intr_type(PG_GET, GPIO_INTR_POSEDGE);

    //create a queue to handle gpio event from isr
    gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));
    //start gpio task
    xTaskCreate(gpio_task_example, "gpio_task_example", 2048, NULL, 10, NULL);

    //install gpio isr service
    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
    //hook isr handler for specific gpio pin
    gpio_isr_handler_add(GPIO_INPUT_IO_0, gpio_isr_handler, (void*) GPIO_INPUT_IO_0);
    gpio_isr_handler_add(PG_GET, gpio_isr_handler, (void*) PG_GET);

    uint8_t recvbuf[BUF_SIZE];
	int len;
    /* 定义一个gpio配置结构体 */
    gpio_config_t gpio_config_structure;

     uart_config_t uart_config =
    {
        .baud_rate = 57600,//波特率
        .data_bits = UART_DATA_8_BITS,//8位数据
        .parity    = UART_PARITY_DISABLE,//不校验
        .stop_bits = UART_STOP_BITS_1,//1位停止位
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,//禁用流控
        .source_clk = UART_SCLK_APB,//apb时钟源
    };
    uart_driver_install(UART_NUM1, BUF_SIZE, 0, 0, NULL, 0);//驱动安装
    uart_param_config(UART_NUM1, &uart_config);//配置参数
    uart_set_pin(UART_NUM1, UART1_TX_PIN, UART1_RX_PIN, UART1_RTS_PIN, UART1_CTS_PIN);//配置引脚

    /* 初始化gpio配置结构体*/
    gpio_config_structure.pin_bit_mask = ((1ULL << GPIO_LED_NUM)|(1ULL << FIN_LOCK_CTRL));/* 选择gpio2 */
    gpio_config_structure.mode = GPIO_MODE_OUTPUT;              /* 输出模式 */
    gpio_config_structure.pull_up_en = 0;                       /* 不上拉 */
    gpio_config_structure.pull_down_en = 0;                     /* 不下拉 */
    gpio_config_structure.intr_type = GPIO_PIN_INTR_DISABLE;    /* 禁止中断 */
    
    gpio_set_level(FIN_LOCK_CTRL, 1);

    /* 根据设定参数初始化并使能 */  
	gpio_config(&gpio_config_structure);

    while(1)
    {
        gpio_set_level(GPIO_LED_NUM, 0);        /* 熄灭 */
        vTaskDelay(100 / portTICK_PERIOD_MS);   /* 延时500ms*/
        gpio_set_level(GPIO_LED_NUM, 1);        /* 点亮 */
        vTaskDelay(100 / portTICK_PERIOD_MS);   /* 延时500ms*/

		len = uart_read_bytes(UART_NUM1, recvbuf, BUF_SIZE, 100 / portTICK_RATE_MS);//读取数据
        if(len>0){
            for(int i = 0; i<len; i++){
                printf("%02X ",recvbuf[i]);
            }
            printf("\n");
            // if(recvbuf[0] == 0xEF
            //   &&recvbuf[1] == 0x01
            //   &&recvbuf[2] == 0xFF
            //   &&recvbuf[3] == 0xFF
            //   &&recvbuf[4] == 0xFF
            //   &&recvbuf[5] == 0xFF
            //   &&recvbuf[6] == 0x07){
            //   printf("%s\n","验证成功");
            // }
            // else{
            //     printf("%s\n","验证失败");
            // }
         
        }
     
    }       
}