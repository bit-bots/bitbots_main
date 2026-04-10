#ifndef ANALYSIS_DATA_H
#define ANALYSIS_DATA_H

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdint.h>

/*------------------------------------------------Type define--------------------------------------------------*/
typedef enum
{
	crc_err = -3,
	data_len_err = -2,
	para_err = -1,
	analysis_ok = 0,
	analysis_done = 1
}analysis_res_t;

#pragma pack(1)

typedef struct
{
	unsigned char header1;	/*0x59*/
	unsigned char header2;	/*0x53*/
	unsigned short tid;		/*1 -- 60000*/
	unsigned char len;		/*length of payload, 0 -- 255*/
}output_data_header_t;

typedef struct
{
	unsigned char data_id;
	unsigned char data_len;
}payload_data_t;

#pragma pack()

typedef enum ClassType{
	PRODUCT_INFOMATION       = 0x00,
	UART_BAUDRATE            = 0x02,
	OUTPUT_FREEQUENCY        = 0x03,
	OUTPUT_CONTENT           = 0x04,
	CALIBRATION_PARAM_SET    = 0x05,
	MODE_SETTING             = 0x4D,
	NEMA0183_OUTPUT_CONTENT  = 0x4E
}ClassType;

typedef enum ClassID{
	QUERY_STATE     = 0x00,
	SET_STATE_MEM   = 0x01,
	SET_STATE_FLASH = 0x02
}ClassID;

typedef enum BaudRate{
	BAUDRATE_9600 = 0x01,
	BAUDRATE_38400,
	BAUDRATE_115200,
	BAUDRATE_460800,
	BAUDRATE_921600,
	BAUDRATE_19200,
	BAUDRATE_57600,
	BAUDRATE_78600,
	BAUDRATE_230400,
}BaudRate;

typedef enum Freequency{
	FREEQUENCY_1_HZ = 0x01,
	FREEQUENCY_2_HZ,
	FREEQUENCY_5_HZ,
	FREEQUENCY_10_HZ,
	FREEQUENCY_20_HZ,
	FREEQUENCY_25_HZ,
	FREEQUENCY_50_HZ,
	FREEQUENCY_100_HZ,
	FREEQUENCY_200_HZ,
	FREEQUENCY_250_HZ,
	FREEQUENCY_500_HZ,
	FREEQUENCY_1000_HZ
}Freequency;

typedef enum OUTPUT_CONTENT_INFO{
	SPEED_ENABLE                    = 0x00,
	LOCATION_ENABLE,
	UTC_ENABLE,
	QUATERNION_ENABLE,
	EULER_ENABLE,
	MAGNETIC_ENABLE,
	ANGULAR_VELICITY_ENABLE,
	ACCEL_INCREAMENT_ENABLE,
	VELICITY_INCREAMENT_ENABLE,
	QUATERNION_INCREAMENT_ENABLE,
	IMU_TEMP_ENABLE,
	SECOND_IMU_ANGLE_ENABLE,
	SECOND_IMU_ACCEL_ENABLE,
	SECOND_IMU_TEMP_ENABLE,
	FREE_ACCEL_ENABLE,
	TIMESTAMP_ENABLE	
}OUTPUT_CONTENT_INFO;

typedef struct 
{
#pragma pack(1)

	struct
	{
		uint16_t year;
		uint8_t month;
		uint8_t date;
		uint8_t hour;
		uint8_t min;
		uint8_t sec;
		uint16_t ms;
	} utc_time;

#pragma pack()

	struct 
	{
		double lon;
		double lat;
		float alt;
	} location;

	struct 
	{
		float lon;
		float lat;
		float alt;
	} location_error;

	float speed;
	float yaw;
	uint8_t status;
	uint8_t star_cnt;
	float p_dop;
	uint8_t site_id;

}imu_gnss_data_t;

typedef struct
{
	unsigned int sample_timestamp;		/*unit: us*/
	unsigned int out_sync_timestamp;	/*unit: us*/ 

	/*0x01*/
	float imu_temp;                /*unit: °C*/    

	/*0x02*/
	float second_imu_temp;         /*unit: °C*/     
	
	/*0x11*/
	float free_accel_x;            /*unit: m/s2*/     
	float free_accel_y;
	float free_accel_z;
	
	/*0x12*/
	float speed_inc_x;             /*unit: m/s*/     
	float speed_inc_y;
	float speed_inc_z;
	
	/*0x18*/
	float second_accel_x;          /*unit: m/s2*/ 
	float second_accel_y;
	float second_accel_z;
	
	/*0x28*/
	float second_angle_x;          /*unit: ° (deg)/s*/
	float second_angle_y;
	float second_angle_z;
	
	/*0x42*/
	float quaternion_inc_data0;
	float quaternion_inc_data1;
	float quaternion_inc_data2;
	float quaternion_inc_data3;

	float accel_x;			/*unit: m/s2*/
	float accel_y;
	float accel_z;

	float angle_x;			/*unit: ° (deg)/s*/
	float angle_y;
	float angle_z;

	float mag_x;			/*unit: 归一化值*/
	float mag_y;
	float mag_z;

	float raw_mag_x;		/*unit: mGauss*/
	float raw_mag_y;
	float raw_mag_z;
	
	float pitch;			/*unit: ° (deg)*/
	float roll;
	float yaw;
	
	float quaternion_data0;
	float quaternion_data1;	
	float quaternion_data2;
	float quaternion_data3;
	
	double latitude;					/*unit: deg*/
	double longtidue;					/*unit: deg*/
	float altidue;						/*unit: m*/
	
	float vel_n;						/*unit: m/s */
	float vel_e;
	float vel_d;

	uint8_t status;

	imu_gnss_data_t gnss; /* master GNSS */

	struct
	{
		float dual_ant_yaw;
		float dual_ant_yaw_error;
		float dual_ant_baseline_len;
	} gnss_slave; /* slave GNSS */

}protocol_info_t;

extern protocol_info_t g_output_info;

unsigned char parse_data_by_id(unsigned char id, unsigned char len, unsigned char *data);

#ifdef __cplusplus
}
#endif

#endif
