/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "fdcan.h"
#include "memorymap.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "string.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define default_pwm_val 50
#define default_pwm_shut 0
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
uint8_t rx_buf[5];
char *str = "Hello, world!";
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

// CAN接收相关全局变量
extern volatile uint8_t can_msg_received;  // 从fdcan.c引用接收标志

// CAN接收数据变量
uint32_t received_can_id = 0;      // 接收到的CAN消息ID
uint8_t received_can_data[8] = {0}; // 接收到的CAN数据
uint8_t received_can_length = 0;   // 接收到的CAN数据长度
uint32_t can_receive_count = 0;    // 接收消息计数器

uint8_t rx_dma_buf[1024];
volatile uint8_t dma_rx_complete = 0;  // DMA接收完成标志
uint8_t rx_data_copy[1024];            // 复制出来的数据存储数组

// USART2 DMA接收相关变量
uint8_t usart2_dma_buf[32];           // USART2 DMA接收缓冲区
volatile uint8_t usart2_dma_complete = 0;  // USART2 DMA接收完成标志
uint8_t usart2_data_copy[32];         // USART2复制出来的数据存储数组

// IMU数据结构
typedef struct {
    int16_t accel_x, accel_y, accel_z;     // 加速度
    int16_t gyro_x, gyro_y, gyro_z;        // 角速度
    int16_t roll, pitch, yaw;              // 角度
    int16_t temperature;                   // 温度
} IMU_Data_t;

IMU_Data_t imu_data;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void UART_Start_DMA_Receive(uint8_t *buffer, uint16_t size);
void USART2_Start_DMA_Receive(uint8_t *buffer, uint16_t size);
void Process_Received_Data(void);
void Process_USART2_Data(void);
void volatile_delay_tick(uint32_t tick)
{
    for(volatile int i=0; i<tick; i++)
    {
        __NOP(); 
    }
}
uint8_t Parse_IMU_Frame(uint8_t *data, uint16_t start_pos, uint16_t frame_num);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/**
 * @brief 重定向printf到UART1
 */
int fputc(int ch, FILE *f)
{
    // 采用轮询方式发送1字节数据，超时时间设置为无限等待
    HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
    return ch;
}

/**
 * @brief 重定向scanf从UART1
 */
int fgetc(FILE *f)
{
    uint8_t ch;
    // 采用轮询方式接收1字节数据，超时时间设置为无限等待
    HAL_UART_Receive(&huart1, (uint8_t*)&ch, 1, HAL_MAX_DELAY);
    return ch;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  	int temp_ccr;
//    if(huart->Instance == UART7)
//    {
//      dma_rx_complete = 1;
//      HAL_UART_Receive_DMA(&huart7, rx_dma_buf, sizeof(rx_dma_buf));
//    }
    if(huart->Instance == USART2)
    {
      // 检查是否是DMA接收完成
      usart2_dma_complete = 1;
      HAL_UART_Receive_DMA(&huart2, usart2_dma_buf, sizeof(usart2_dma_buf));
     
    }
}
//		for(int i=0;i<5;i++)rx_buf[i]=0;
// } 



/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_ADC1_Init();
  MX_FDCAN1_Init();
  MX_FDCAN2_Init();
  MX_UART7_Init();
  MX_UART8_Init();
  MX_USART6_UART_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  MX_TIM8_Init();
  MX_TIM12_Init();
  MX_UART4_Init();
  /* USER CODE BEGIN 2 */
  
  // 配置CAN过滤器 - 接收必需
  CAN_Config_Filter();

  // 启动FDCAN - 接收必需
  HAL_FDCAN_Start(&hfdcan1);

  // 启动CAN接收中断 - 接收必需
  CAN_Start_Receive();
  
  // 启动UART7 DMA接收，数据自动存储到rx_dma_buf
  //UART_Start_DMA_Receive(rx_dma_buf, sizeof(rx_dma_buf));
  
  // 启动USART2 DMA接收，数据自动存储到usart2_dma_buf
  USART2_Start_DMA_Receive(usart2_dma_buf, sizeof(usart2_dma_buf));
  
  //启用pwm
    HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_4);
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_4);
	HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_4);
  __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_3, default_pwm_shut);
	__HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_1, default_pwm_shut);
	__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_3, default_pwm_shut);
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_3, default_pwm_shut);
	// volatile_delay_tick(100000); // 确保PWM初始化完成
	__HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_4, default_pwm_shut);
	__HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_2, default_pwm_shut);
	__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_4, default_pwm_shut);
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_4, default_pwm_shut);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  { 
    //检查UART7 DMA接收完成标志
    if(dma_rx_complete)
    {
        // 复制数据到备份数组
        memcpy(rx_data_copy, rx_dma_buf, sizeof(rx_dma_buf));
        
        // 处理接收到的数据 
        //Process_Received_Data();
        
        // 处理完后清除标志
        dma_rx_complete = 0;
    }
    
    // 检查USART2 DMA接收完成标志
    if(usart2_dma_complete)
    {
        // 复制数据到备份数组
        memcpy(usart2_data_copy, usart2_dma_buf, sizeof(usart2_dma_buf));
        
        // 处理USART2 DMA数据
        Process_USART2_Data();
        
			
			
        // 处理完后清除标志
        usart2_dma_complete = 0;
    }
    
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
   }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 5;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_2;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
/**
 * @brief 解析单个IMU数据帧并输出结果
 * @param data: 数据缓冲区
 * @param start_pos: 帧起始位置
 * @param frame_num: 帧序号
 * @return 1-成功解析，0-解析失败
 */
uint8_t Parse_IMU_Frame(uint8_t *data, uint16_t start_pos, uint16_t frame_num)
{
    // 检查帧头 0x55
    if(data[start_pos] != 0x55) return 0;
    
    uint8_t frame_type = data[start_pos + 1];
    
    printf("--- Frame #%d (pos:%d) ---\r\n", frame_num, start_pos);
    printf("Header: 0x%02X, Type: 0x%02X\r\n", data[start_pos], frame_type);
    
    switch(frame_type)
    {
        case 0x51: // 加速度数据帧
        {
            imu_data.accel_x = (int16_t)((data[start_pos + 3] << 8) | data[start_pos + 2]);
            imu_data.accel_y = (int16_t)((data[start_pos + 5] << 8) | data[start_pos + 4]);
            imu_data.accel_z = (int16_t)((data[start_pos + 7] << 8) | data[start_pos + 6]);
            imu_data.temperature = (int16_t)((data[start_pos + 9] << 8) | data[start_pos + 8]);
            

            printf("Raw: X=%d, Y=%d, Z=%d, Temp=%d\r\n", 
                   imu_data.accel_x, imu_data.accel_y, imu_data.accel_z, imu_data.temperature);
            printf("Physical: X=%.3fg, Y=%.3fg, Z=%.3fg, Temp=%.1fC\r\n", 
                   imu_data.accel_x/32768.0*16, 
                   imu_data.accel_y/32768.0*16, 
                   imu_data.accel_z/32768.0*16,
                   imu_data.temperature/340.0 + 36.25);
            break;
        }
            
        case 0x52: // 角速度数据帧
        {
            imu_data.gyro_x = (int16_t)((data[start_pos + 3] << 8) | data[start_pos + 2]);
            imu_data.gyro_y = (int16_t)((data[start_pos + 5] << 8) | data[start_pos + 4]);
            imu_data.gyro_z = (int16_t)((data[start_pos + 7] << 8) | data[start_pos + 6]);
            

            printf("Raw: X=%d, Y=%d, Z=%d\r\n", 
                   imu_data.gyro_x, imu_data.gyro_y, imu_data.gyro_z);
            printf("Physical: X=%.2fdps, Y=%.2fdps, Z=%.2fdps\r\n", 
                   imu_data.gyro_x/32768.0*2000, 
                   imu_data.gyro_y/32768.0*2000, 
                   imu_data.gyro_z/32768.0*2000);
            break;
        }
            
        case 0x53: // 角度数据帧
        {
            imu_data.roll = (int16_t)((data[start_pos + 3] << 8) | data[start_pos + 2]);
            imu_data.pitch = (int16_t)((data[start_pos + 5] << 8) | data[start_pos + 4]);
            imu_data.yaw = (int16_t)((data[start_pos + 7] << 8) | data[start_pos + 6]);
            

            printf("Raw: Roll=%d, Pitch=%d, Yaw=%d\r\n", 
                   imu_data.roll, imu_data.pitch, imu_data.yaw);
            printf("Physical: Roll=%.2fdeg, Pitch=%.2fdeg, Yaw=%.2fdeg\r\n", 
                   imu_data.roll/32768.0*180, 
                   imu_data.pitch/32768.0*180, 
                   imu_data.yaw/32768.0*180);
            break;
        }
            
        default:
            printf("[Unknown Frame Type] 0x%02X\r\n", frame_type);
            return 0; // 未知帧类型
    }
    HAL_Delay(1); // 确保输出稳定
    printf("\r\n");
    return 1; // 解析成功
}

/**
 * @brief 处理接收到的IMU数据并逐帧解算输出
 */
void Process_Received_Data(void)
{
    // printf("========================================\r\n");
    // printf("=== IMU Data Processing Start (1024 bytes) ===\r\n");
    // printf("========================================\r\n");
    
    uint16_t valid_frames = 0;
    uint16_t invalid_frames = 0;
    uint16_t i = 0;
    
    // 遍历整个缓冲区寻找IMU数据帧
    while(i < 1024 - 10) // 确保至少有11字节进行帧检查
    {
        // 寻找帧头 0x55
        if(rx_data_copy[i] == 0x55)
        {
            // 检查是否有足够字节构成完整帧(11字节)
            if(i + 11 <= 1024)
            {
                // 尝试解析IMU数据帧
                if(Parse_IMU_Frame(rx_data_copy, i, valid_frames + 1))
                {
                    valid_frames++;
                    i += 11; // 跳过已解析的帧
                }
                else
                {
                    invalid_frames++;
                    i++; // 继续寻找下一个可能的帧头
                }
            }
            else
            {
                printf("--- Incomplete Frame (pos:%d, remaining:%d) ---\r\n", i, 1024-i);
                break; // 剩余字节不足，退出循环
            }
        }
        else
        {
            i++; // 继续寻找帧头
        }
    }
    
    // printf("========================================\r\n");
    // printf("=== Processing Statistics ===\r\n");
    // printf("Valid frames: %d\r\n", valid_frames);
    // printf("Invalid frames: %d\r\n", invalid_frames);
    // printf("Total bytes processed: 1024\r\n");
    
    // 如果有有效帧，显示最终IMU状态
    // if(valid_frames > 0)
    // {
    //     printf("\r\n=== Final IMU Status Summary ===\r\n");
    //     printf("Accelerometer: X=%.3fg, Y=%.3fg, Z=%.3fg\r\n", 
    //            imu_data.accel_x/32768.0*16, 
    //            imu_data.accel_y/32768.0*16, 
    //            imu_data.accel_z/32768.0*16);
    //     printf("Gyroscope: X=%.2fdps, Y=%.2fdps, Z=%.2fdps\r\n", 
    //            imu_data.gyro_x/32768.0*2000, 
    //            imu_data.gyro_y/32768.0*2000, 
    //            imu_data.gyro_z/32768.0*2000);
    //     printf("Angle: Roll=%.2fdeg, Pitch=%.2fdeg, Yaw=%.2fdeg\r\n", 
    //            imu_data.roll/32768.0*180, 
    //            imu_data.pitch/32768.0*180, 
    //            imu_data.yaw/32768.0*180);
    //     printf("Temperature: %.1fC\r\n", imu_data.temperature/340.0 + 36.25);
    // }
    
    // printf("========================================\r\n");
    // printf("=== IMU Data Processing Complete ===\r\n");
    // printf("========================================\r\n\r\n");
}

/**
 * @brief 处理USART2接收到的数据帧
 */
void Process_USART2_Data(void)
{
    uint16_t valid_frames = 0;
    uint16_t invalid_frames = 0;
    uint16_t i = 0;
    
    // 遍历整个缓冲区寻找数据帧
    while(i < 32 - 13) // 确保至少有14字节进行完整帧检查
    {
        // 寻找帧头 0xA0 0xA5
        if(usart2_data_copy[i] == 0xA0 && usart2_data_copy[i + 1] == 0xA5)
        {
            // 检查是否有足够字节构成完整帧(14字节)
            if(i + 14 <= 32)
            {
                // 检查帧尾 0xC0 0xC5
                if(usart2_data_copy[i + 12] == 0xC0 && usart2_data_copy[i + 13] == 0xC5)
                {
                    // 校验CRC（对帧内9个字节进行异或）
                    uint8_t calculated_crc = 0;
                    for(int j = 0; j < 9; j++)
                    {
                        calculated_crc ^= usart2_data_copy[i + 2 + j];
                    }
                    
                    uint8_t received_crc = usart2_data_copy[i + 11];
                    
                    if(calculated_crc == received_crc)
                    {
                        // CRC校验通过，提取帧内容到9字节数组
                        uint8_t frame_content[9];
                        memcpy(frame_content, &usart2_data_copy[i + 2], 9);
                        if(frame_content[0]==0x0F){
                            __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_4, default_pwm_shut);
                            volatile_delay_tick(10000);
                            __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_3, frame_content[1]);
                        }
                        else if(frame_content[0]==0xF0){
                            __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_3, default_pwm_shut);
                            volatile_delay_tick(10000);
                            __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_4, frame_content[1]);
                        }
                        else{
                            __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_3, default_pwm_shut);
                            __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_4, default_pwm_shut);
                        }

                        if(frame_content[2]==0xF0){
                            __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_2, default_pwm_shut);
                            volatile_delay_tick(10000);
                            __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_1, frame_content[3]);
                        }
                        else if(frame_content[2]==0x0F){
                            __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_1, default_pwm_shut);
                            volatile_delay_tick(10000);
                            __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_2, frame_content[3]);
                        }
                        else{
                            __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_1, default_pwm_shut);
                            __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_2, default_pwm_shut);
                        }

                        if(frame_content[4]==0xF0){
                            __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_4, default_pwm_shut);
                            volatile_delay_tick(10000);
                            __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_3, frame_content[5]);
                        }
                        else if(frame_content[4]==0x0F){
                            __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_3, default_pwm_shut);
                            volatile_delay_tick(10000);
                            __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_4, frame_content[5]);
                        }
                        else{
                            __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_3, default_pwm_shut);
                            __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_4, default_pwm_shut);
                        }

                        if(frame_content[6]==0x0F){
                            __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_4, default_pwm_shut);
                            volatile_delay_tick(10000);
                            __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_3, frame_content[7]);
                        }
                        else if(frame_content[6]==0xF0){
                            __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_3, default_pwm_shut);
                            volatile_delay_tick(10000);
                            __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_4, frame_content[7]);
                        }
                        else{
                            __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_3, default_pwm_shut);
                            __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_4, default_pwm_shut);
                        }
                        if(frame_content[8]==0xF0){
                            HAL_GPIO_WritePin(GPIOB,GPIO_PIN_1,1);
                            HAL_GPIO_WritePin(GPIOB,GPIO_PIN_2,0);
                        }
                        else if(frame_content[8]==0x0F){
                            HAL_GPIO_WritePin(GPIOB,GPIO_PIN_1,0);
                            HAL_GPIO_WritePin(GPIOB,GPIO_PIN_2,1);
                        }


                        valid_frames++;
                        
                        
                        return;
                    }
                    else
                    {
                        invalid_frames++;
                        i++; // 继续寻找下一个可能的帧头
                    }
                }
                else
                {
                    // 帧尾不匹配
                    invalid_frames++;
                    i++; // 继续寻找下一个可能的帧头
                }
            }
            else
            {
                break; // 剩余字节不足，退出循环
            }
        }
        else
        {
            i++; // 继续寻找帧头
        }
    }
    
}

/**
 * @brief 简单启动UART DMA接收到指定数组
 * @param buffer: 接收数据的数组
 * @param size: 数组大小
 */
void UART_Start_DMA_Receive(uint8_t *buffer, uint16_t size)
{
    // 清空缓冲区和标志
    memset(buffer, 0, size);
    dma_rx_complete = 0;
    
    // 启动DMA接收，数据会自动存储到指定数组
    HAL_UART_Receive_DMA(&huart7, buffer, size);
}

/**
 * @brief 启动USART2 DMA接收到指定数组
 * @param buffer: 接收数据的数组
 * @param size: 数组大小
 */
void USART2_Start_DMA_Receive(uint8_t *buffer, uint16_t size)
{
    // 清空缓冲区和标志
    memset(buffer, 0, size);
    usart2_dma_complete = 0;
    
    // 启动DMA接收，数据会自动存储到指定数组
    HAL_UART_Receive_DMA(&huart2, buffer, size);
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
