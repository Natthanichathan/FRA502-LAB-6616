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
#include "cmsis_os.h"
#include "dma.h"
#include "i2c.h"
#include "iwdg.h"
#include "usart.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <uxr/client/transport.h>
#include <rmw_microxrcedds_c/config.h>
#include <rmw_microros/rmw_microros.h>
#include <rcl/init_options.h>
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/float32_multi_array.h>
#include <math.h>

#include <micro_ros_utilities/string_utilities.h>
#include <sensor_msgs/msg/imu.h>
#include <mpu6050.h>
#include <math.h>
#include <sensor_interface/msg/sensor_parameter.h>
#include <std_msgs/msg/u_int32.h>


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define G2M_S2 9.81 // Gravity to Meter per Sec square
#define DEG2RAD M_PI / 180.0 // Degree to Radian
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define RCSOFTCHECK(fn) if (fn != RCL_RET_OK) {};
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
  rclc_support_t support;
  rcl_allocator_t allocator;
  rcl_node_t node;
  rcl_init_options_t init_options;
  rcl_timer_t mpu6050_timer;
  rclc_executor_t executor;

  // ------- IMU -------
  MPU6050_t MPU6050;

  // Sensor Publisher
  rcl_publisher_t sensor_publisher;
  sensor_interface__msg__SensorParameter sensor_msg;

  // encoder read
  uint32_t encoder1;
  uint32_t encoder2;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */
bool cubemx_transport_open(struct uxrCustomTransport *transport);
bool cubemx_transport_close(struct uxrCustomTransport *transport);
size_t cubemx_transport_write(struct uxrCustomTransport *transport,
		const uint8_t *buf, size_t len, uint8_t *err);
size_t cubemx_transport_read(struct uxrCustomTransport *transport, uint8_t *buf,
		size_t len, int timeout, uint8_t *err);

void* microros_allocate(size_t size, void *state);
void microros_deallocate(void *pointer, void *state);
void* microros_reallocate(void *pointer, size_t size, void *state);
void* microros_zero_allocate(size_t number_of_elements, size_t size_of_element,
		void *state);
//uint64_t micros();
//void QEIEncoderPosVel_Update1();


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
	if (timer != NULL) {

		// for refresh auto just once
		HAL_IWDG_Refresh(&hiwdg);

		MPU6050_Read_All(&hi2c1, &MPU6050);

		encoder1 = __HAL_TIM_GET_COUNTER(&htim2);
		encoder2 = __HAL_TIM_GET_COUNTER(&htim5);

		sensor_msg.encoder1.data = encoder1;
		sensor_msg.encoder2.data = encoder2;
		sensor_msg.imu_mpu6050.header.stamp.sec = rmw_uros_epoch_millis() / 1000; // sec
		sensor_msg.imu_mpu6050.header.stamp.nanosec = rmw_uros_epoch_nanos();
		sensor_msg.imu_mpu6050.linear_acceleration.x = G2M_S2 * MPU6050.Ax; // translation_x_acc (m/s^2)
		sensor_msg.imu_mpu6050.linear_acceleration.y = G2M_S2 * MPU6050.Ay;
		sensor_msg.imu_mpu6050.linear_acceleration.z = G2M_S2 * MPU6050.Az;
		sensor_msg.imu_mpu6050.angular_velocity.x = DEG2RAD * MPU6050.Gx; // rotation_x_vel (rad/s)
		sensor_msg.imu_mpu6050.angular_velocity.y = DEG2RAD * MPU6050.Gy;
		sensor_msg.imu_mpu6050.angular_velocity.z = DEG2RAD * MPU6050.Gz;

		RCSOFTCHECK(rcl_publish(&sensor_publisher, &sensor_msg, NULL));

	}
}

void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */

  // micro-ROS configuration

  rmw_uros_set_custom_transport(
    true,
    (void *) &hlpuart1,
    cubemx_transport_open,
    cubemx_transport_close,
    cubemx_transport_write,
    cubemx_transport_read);

  rcl_allocator_t freeRTOS_allocator = rcutils_get_zero_initialized_allocator();
  freeRTOS_allocator.allocate = microros_allocate;
  freeRTOS_allocator.deallocate = microros_deallocate;
  freeRTOS_allocator.reallocate = microros_reallocate;
  freeRTOS_allocator.zero_allocate =  microros_zero_allocate;

  if (!rcutils_set_default_allocator(&freeRTOS_allocator)) {
      printf("Error on default allocators (line %d)\n", __LINE__);
  }

  // micro-ROS app
  allocator = rcl_get_default_allocator();

  //create init_options
  init_options = rcl_get_zero_initialized_init_options();
  RCSOFTCHECK(rcl_init_options_init(&init_options, allocator));
  RCSOFTCHECK(rcl_init_options_set_domain_id(&init_options, 16));

  rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator);

  // create node
  rclc_node_init_default(&node, "sensor_fusion_node", "", &support);

  // sync timer
  rmw_uros_sync_session(1000);

  // create timer
  rclc_timer_init_default(
	  &mpu6050_timer,
	  &support,
	  RCL_MS_TO_NS(10),
	  timer_callback
  );

  rclc_publisher_init_best_effort(
      &sensor_publisher,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_interface, msg, SensorParameter),
      "sensor_parameter_publisher"
  );
  sensor_interface__msg__SensorParameter__init(&sensor_msg);
  std_msgs__msg__UInt32__init(&sensor_msg.encoder1);
  std_msgs__msg__UInt32__init(&sensor_msg.encoder2);
  sensor_msgs__msg__Imu__init(&sensor_msg.imu_mpu6050);

  sensor_msg.imu_mpu6050.header.frame_id = micro_ros_string_utilities_init("imu_frame");

  // create executor
  executor = rclc_executor_get_zero_initialized_executor();
  rclc_executor_init(&executor, &support.context, 1, &allocator); // mpu6050_timer_callback
  rclc_executor_add_timer(&executor, &mpu6050_timer);

  rclc_executor_spin(&executor);

  for(;;)
  {
    osDelay(10);
  }
  /* USER CODE END 5 */
}
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
  MX_LPUART1_UART_Init();
  MX_I2C1_Init();
  MX_IWDG_Init();
  MX_TIM5_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

  // encoder vel
  HAL_TIM_Encoder_Start(&htim2,TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim5, TIM_CHANNEL_ALL);

  // IMU read
  while (MPU6050_Init(&hi2c1) == 1);
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();  /* Call init function for freertos objects (in cmsis_os2.c) */
  MX_FREERTOS_Init();

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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
#ifdef USE_FULL_ASSERT
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
