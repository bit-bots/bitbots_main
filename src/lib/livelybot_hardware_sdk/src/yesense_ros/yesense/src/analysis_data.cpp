#include <stdint.h>
#include <stddef.h>
#include "analysis_data.h"
#include <ros/ros.h>

/*------------------------------------------------MARCOS define------------------------------------------------*/

#define PROTOCOL_FIRST_BYTE			(unsigned char)0x59
#define PROTOCOL_SECOND_BYTE		(unsigned char)0x53

#define PROTOCOL_FIRST_BYTE_POS 		0
#define PROTOCOL_SECOND_BYTE_POS		1

#define PROTOCOL_TID_LEN				2
#define PROTOCOL_MIN_LEN				7	/*header(2B) + tid(2B) + len(1B) + CK1(1B) + CK2(1B)*/

#define CRC_CALC_START_POS				2
#define CRC_CALC_LEN(payload_len)		((payload_len) + 3)	/*3 = tid(2B) + len(1B)*/
#define PROTOCOL_CRC_DATA_POS(payload_len)			(CRC_CALC_START_POS + CRC_CALC_LEN(payload_len))

#define PAYLOAD_POS						5

#define SINGLE_DATA_BYTES				4

/*data id define*/
#define IMU_TEMP_ID             (unsigned char)0x01
#define SECOND_IMU_TEMP_ID      (unsigned char)0x02
#define FREE_ACCEL_ID           (unsigned char)0x11
#define SPEED_INCREMENT_ID	    (unsigned char)0x12
#define SECOND_ACCEL_ID         (unsigned char)0x18
#define SECOND_ANGLE_ID         (unsigned char)0x28
#define QUATERNION_INCREMENT_ID (unsigned char)0x42

#define ACCEL_ID				(unsigned char)0x10
#define ANGLE_ID				(unsigned char)0x20
#define MAGNETIC_ID				(unsigned char)0x30     /*归一化值*/
#define RAW_MAGNETIC_ID			(unsigned char)0x31     /*原始值*/
#define EULER_ID				(unsigned char)0x40
#define QUATERNION_ID			(unsigned char)0x41
#define UTC_ID					(unsigned char)0x50
#define SAMPLE_TIMESTAMP_ID		(unsigned char)0x51
#define DATA_READY_TIMESTAMP_ID	(unsigned char)0x52
#define LOCATION_ID				(unsigned char)0x68     /*high precision location*/
#define SPEED_ID				(unsigned char)0x70
#define STATUS_ID				(unsigned char)0x80 	/*imu status*/

#define GNSS_MASTER_ID			(unsigned char)0xc0
#define GNSS_SLAVE_ID			(unsigned char)0xf0

/*length for specific data id*/
#define IMU_TEMP_DATA_LEN               (unsigned char)2
#define SECOND_IMU_TEMP_DATA_LEN        (unsigned char)2
#define FREE_ACCEL_DATA_LEN             (unsigned char)12
#define SPEED_INCREMENT_DATA_LEN	    (unsigned char)12
#define SECOND_ACCEL_DATA_LEN           (unsigned char)12
#define SECOND_ANGLE_DATA_LEN           (unsigned char)12
#define QUATERNION_INCREMENT_DATA_LEN   (unsigned char)16

#define ACCEL_DATA_LEN					(unsigned char)12
#define ANGLE_DATA_LEN					(unsigned char)12
#define MAGNETIC_DATA_LEN				(unsigned char)12
#define MAGNETIC_RAW_DATA_LEN			(unsigned char)12
#define EULER_DATA_LEN					(unsigned char)12
#define QUATERNION_DATA_LEN				(unsigned char)16
#define UTC_DATA_LEN					(unsigned char)11
#define SAMPLE_TIMESTAMP_DATA_LEN		(unsigned char)4
#define DATA_READY_TIMESTAMP_DATA_LEN	(unsigned char)4
#define LOCATION_DATA_LEN				(unsigned char)20 /*high precision location data len*/
#define SPEED_DATA_LEN          		(unsigned char)12
#define STATUS_DATA_LEN					(unsigned char)1  /*imu status*/

#define GNSS_MASTER_DATA_LEN			(unsigned char)45
#define GNSS_SLAVE_DATA_LEN				(unsigned char)6

/*factor for sensor data*/
#define NOT_MAG_DATA_FACTOR			0.000001f
#define MAG_RAW_DATA_FACTOR			0.001f

#define IMU_TEMP_FACTOR             0.01f

/*factor for gnss data*/
#define LONG_LAT_DATA_FACTOR		0.0000001
#define HIGH_PRECI_LONG_LAT_DATA_FACTOR 0.0000000001
#define ALT_DATA_FACTOR				0.001f
#define SPEED_DATA_FACTOR			0.001f

/*------------------------------------------------Variables define------------------------------------------------*/

protocol_info_t g_output_info;

/*------------------------------------------------Functions declare------------------------------------------------*/

static int16_t get_int16(uint8_t *buf, uint16_t offset);
static int32_t get_int32(uint8_t *buf, uint16_t offset);
static int64_t get_int64(uint8_t *buf, uint16_t offset);

/*-------------------------------------------------------------------------------------------------------------*/
unsigned char parse_data_by_id(unsigned char id, unsigned char len, unsigned char *data)
{
	unsigned char ret = 0xff;

	switch(id)
	{
		case IMU_TEMP_ID:
		{
			if(IMU_TEMP_DATA_LEN == len)
			{
				ret = (unsigned char)0x1;
				uint16_t temp = (((uint16_t)data[1]) << 8) | ((uint16_t)data[0]);
				g_output_info.imu_temp = ((float)temp) * IMU_TEMP_FACTOR;
				//ROS_INFO("IMU_TEMP: 0x%04x", temp);
			}
			else
			{
				ret = (unsigned char)0x00;
			}
		}
		break;

		case SECOND_IMU_TEMP_ID:
		{
			if(SECOND_IMU_TEMP_DATA_LEN == len)
			{
				ret = (unsigned char)0x1;
				uint16_t temp = (((uint16_t)data[1]) << 8) | ((uint16_t)data[0]);
				g_output_info.second_imu_temp = ((float)temp) * IMU_TEMP_FACTOR;
				//ROS_INFO("SECOND_IMU_TEMP: 0x%04x", temp);
			}
			else
			{
				ret = (unsigned char)0x00;
			}
		}
		break;

		case FREE_ACCEL_ID:
		{
			if(FREE_ACCEL_DATA_LEN == len)
			{
				ret = (unsigned char)0x1;
				g_output_info.free_accel_x = get_int32(data, 0) * NOT_MAG_DATA_FACTOR;
				g_output_info.free_accel_y = get_int32(data, SINGLE_DATA_BYTES) * NOT_MAG_DATA_FACTOR;
				g_output_info.free_accel_z = get_int32(data, SINGLE_DATA_BYTES * 2) * NOT_MAG_DATA_FACTOR;
				//ROS_INFO("FREE_ACCEL, x: %f, y: %f, z: %f \r\n",g_output_info.free_accel_x,g_output_info.free_accel_y,g_output_info.free_accel_z);
			}
			else
			{
				ret = (unsigned char)0x00;
			}
		}
		break;

		case SPEED_INCREMENT_ID:
		{
			if(SPEED_INCREMENT_DATA_LEN == len)
			{
				ret = (unsigned char)0x1;
				g_output_info.speed_inc_x = get_int32(data, 0) * NOT_MAG_DATA_FACTOR;
				g_output_info.speed_inc_y = get_int32(data, SINGLE_DATA_BYTES) * NOT_MAG_DATA_FACTOR;
				g_output_info.speed_inc_z = get_int32(data, SINGLE_DATA_BYTES * 2) * NOT_MAG_DATA_FACTOR;
			}
			else
			{
				ret = (unsigned char)0x00;
			}
		}
		break;

		case SECOND_ACCEL_ID:
		{
			if(SECOND_ACCEL_DATA_LEN == len)
			{
				ret = (unsigned char)0x1;
				g_output_info.second_accel_x = get_int32(data, 0) * NOT_MAG_DATA_FACTOR;
				g_output_info.second_accel_y = get_int32(data, SINGLE_DATA_BYTES) * NOT_MAG_DATA_FACTOR;
				g_output_info.second_accel_z = get_int32(data, SINGLE_DATA_BYTES * 2) * NOT_MAG_DATA_FACTOR;
			}
			else
			{
				ret = (unsigned char)0x00;
			}
		}
		break;

		case SECOND_ANGLE_ID:
		{
			if(SECOND_ANGLE_DATA_LEN == len)
			{
				ret = (unsigned char)0x1;
				g_output_info.second_angle_x = get_int32(data, 0) * NOT_MAG_DATA_FACTOR;
				g_output_info.second_angle_y = get_int32(data, SINGLE_DATA_BYTES) * NOT_MAG_DATA_FACTOR;
				g_output_info.second_angle_z = get_int32(data, SINGLE_DATA_BYTES * 2) * NOT_MAG_DATA_FACTOR;
			}
			else
			{
				ret = (unsigned char)0x00;
			}
		}
		break;

		case QUATERNION_INCREMENT_ID:
		{
			if(QUATERNION_INCREMENT_DATA_LEN == len)
			{
				ret = (unsigned char)0x1;
				g_output_info.quaternion_inc_data0 = get_int32(data, 0) * NOT_MAG_DATA_FACTOR;
				g_output_info.quaternion_inc_data1 = get_int32(data, SINGLE_DATA_BYTES) * NOT_MAG_DATA_FACTOR;
				g_output_info.quaternion_inc_data2 = get_int32(data, SINGLE_DATA_BYTES * 2) * NOT_MAG_DATA_FACTOR;
				g_output_info.quaternion_inc_data3 = get_int32(data, SINGLE_DATA_BYTES * 3) * NOT_MAG_DATA_FACTOR;
			}
			else
			{
				ret = (unsigned char)0x00;
			}
		}
		break;

		case ACCEL_ID:
		{
			if(ACCEL_DATA_LEN == len)
			{
				ret = (unsigned char)0x1;
				g_output_info.accel_x = get_int32(data, 0) * NOT_MAG_DATA_FACTOR;
				g_output_info.accel_y = get_int32(data, SINGLE_DATA_BYTES) * NOT_MAG_DATA_FACTOR;
				g_output_info.accel_z = get_int32(data, SINGLE_DATA_BYTES * 2) * NOT_MAG_DATA_FACTOR;
				// ROS_INFO("accel x: %.3f, accel y: %.3f, accel z: %.3f", g_output_info.accel_x, g_output_info.accel_y, g_output_info.accel_z);
			}
			else
			{
				ret = (unsigned char)0x00;
			}
		}
		break;

		case ANGLE_ID:
		{
			if(ANGLE_DATA_LEN == len)
			{
				ret = (unsigned char)0x1;
				g_output_info.angle_x = get_int32(data, 0) * NOT_MAG_DATA_FACTOR;
				g_output_info.angle_y = get_int32(data, SINGLE_DATA_BYTES) * NOT_MAG_DATA_FACTOR;
				g_output_info.angle_z = get_int32(data, SINGLE_DATA_BYTES * 2) * NOT_MAG_DATA_FACTOR;
			}
			else
			{
				ret = (unsigned char)0x00;
			}
		}
		break;

		case MAGNETIC_ID:
		{
			if(MAGNETIC_DATA_LEN == len)
			{
				ret = (unsigned char)0x1;
				g_output_info.mag_x = get_int32(data, 0) * NOT_MAG_DATA_FACTOR;
				g_output_info.mag_y = get_int32(data, SINGLE_DATA_BYTES) * NOT_MAG_DATA_FACTOR;
				g_output_info.mag_z = get_int32(data, SINGLE_DATA_BYTES * 2) * NOT_MAG_DATA_FACTOR;
			}
			else
			{
				ret = (unsigned char)0x00;
			}
		}
		break;

		case RAW_MAGNETIC_ID:
		{
			if(MAGNETIC_RAW_DATA_LEN == len)
			{
				ret = (unsigned char)0x1;
				g_output_info.raw_mag_x = get_int32(data, 0) * MAG_RAW_DATA_FACTOR;
				g_output_info.raw_mag_y = get_int32(data, SINGLE_DATA_BYTES) * MAG_RAW_DATA_FACTOR;
				g_output_info.raw_mag_z = get_int32(data, SINGLE_DATA_BYTES * 2) * MAG_RAW_DATA_FACTOR;
			}
			else
			{
				ret = (unsigned char)0x00;
			}
		}
		break;

		case EULER_ID:
		{
			if(EULER_DATA_LEN == len)
			{
				ret = (unsigned char)0x1;
				g_output_info.pitch = get_int32(data, 0) * NOT_MAG_DATA_FACTOR;
				g_output_info.roll = get_int32(data, SINGLE_DATA_BYTES) * NOT_MAG_DATA_FACTOR;
				g_output_info.yaw = get_int32(data, SINGLE_DATA_BYTES * 2) * NOT_MAG_DATA_FACTOR;
			}
			else
			{
				ret = (unsigned char)0x00;
			}
		}
		break;

		case QUATERNION_ID:
		{
			if(QUATERNION_DATA_LEN == len)
			{
				ret = (unsigned char)0x1;
				g_output_info.quaternion_data0 = get_int32(data, 0) * NOT_MAG_DATA_FACTOR;
				g_output_info.quaternion_data1 = get_int32(data, SINGLE_DATA_BYTES) * NOT_MAG_DATA_FACTOR;
				g_output_info.quaternion_data2 = get_int32(data, SINGLE_DATA_BYTES * 2) * NOT_MAG_DATA_FACTOR;
				g_output_info.quaternion_data3 = get_int32(data, SINGLE_DATA_BYTES * 3) * NOT_MAG_DATA_FACTOR;
			}
			else
			{
				ret = (unsigned char)0x00;
			}
		}
		break;

		case LOCATION_ID:
		{
			if(LOCATION_DATA_LEN == len)
			{
				ret = (unsigned char)0x1;
				g_output_info.latitude = get_int64(data, 0) * HIGH_PRECI_LONG_LAT_DATA_FACTOR;
				g_output_info.longtidue = get_int64(data, 8) * HIGH_PRECI_LONG_LAT_DATA_FACTOR;
				g_output_info.altidue = get_int32(data, 16) * ALT_DATA_FACTOR;
				// ROS_INFO("location: %.3f, %.3f, %.3f", g_output_info.longtidue, g_output_info.latitude, g_output_info.altidue);
			}
			else
			{
				ret = (unsigned char)0x00;
			}
		}
		break;
		
		case SPEED_ID:
		{
			if(SPEED_DATA_LEN == len)
			{
				ret = (unsigned char)0x1;
				g_output_info.vel_n = get_int32(data, 0) * SPEED_DATA_FACTOR;
				g_output_info.vel_e = get_int32(data, 4) * SPEED_DATA_FACTOR;
				g_output_info.vel_d = get_int32(data, 8) * SPEED_DATA_FACTOR;
			}
			else
			{
				ret = (unsigned char)0x00;
			}
		}
		break;

		case STATUS_ID:
		{
			if(STATUS_DATA_LEN == len)
			{
				ret = (unsigned char)0x1;
				g_output_info.status = *data;
				//ROS_INFO("status code: %d", g_output_info.status);
			}
			else
			{
				ret = (unsigned char)0x00;
			}
		}
		break;

		case GNSS_MASTER_ID:
		{
			ret = GNSS_MASTER_DATA_LEN == len;

			if (ret)
			{
				uint8_t *buf = data;

				memcpy(&g_output_info.gnss.utc_time, buf, 9);
				buf += 9;

				g_output_info.gnss.location.lat = get_int64(buf, 0) * HIGH_PRECI_LONG_LAT_DATA_FACTOR;
				g_output_info.gnss.location.lon = get_int64(buf, 8) * HIGH_PRECI_LONG_LAT_DATA_FACTOR;
				g_output_info.gnss.location.alt = get_int32(buf, 16) * ALT_DATA_FACTOR;
				buf += 20;

				g_output_info.gnss.location_error.lat = get_int16(buf, 0) * 0.001f;
				g_output_info.gnss.location_error.lon = get_int16(buf, 2) * 0.001f;
				g_output_info.gnss.location_error.alt = get_int16(buf, 4) * 0.001f;
				buf += 6;

				g_output_info.gnss.speed = get_int16(buf, 0) * 0.01f;
				g_output_info.gnss.yaw = get_int16(buf, 2) * 0.01f;
				buf += 4;

				g_output_info.gnss.status = buf[0];
				g_output_info.gnss.star_cnt = buf[1];
				buf += 2;

				g_output_info.gnss.p_dop = get_int16(buf, 0) * 0.001f;
				g_output_info.gnss.site_id = get_int16(buf, 2);
			}
		}
		break;

		case GNSS_SLAVE_ID:
		{
			ret = GNSS_SLAVE_DATA_LEN == len;

			if (ret)
			{
				uint8_t *buf = data;
				g_output_info.gnss_slave.dual_ant_yaw = get_int16(buf, 0) * 0.01f;
				g_output_info.gnss_slave.dual_ant_yaw_error = get_int16(buf, 2) * 0.001f;
				g_output_info.gnss_slave.dual_ant_baseline_len = get_int16(buf, 4) * 0.001f;
			}
		}
		break;
		
		case SAMPLE_TIMESTAMP_ID:
		{
			if(SAMPLE_TIMESTAMP_DATA_LEN == len)
			{
				ret = (unsigned char)0x1;
				g_output_info.sample_timestamp = *((unsigned int *)data);
			}
			else
			{
				ret = (unsigned char)0x00;
			}
		}
		break;

		case DATA_READY_TIMESTAMP_ID:
		{
			if(DATA_READY_TIMESTAMP_DATA_LEN == len)
			{
				ret = (unsigned char)0x1;
				g_output_info.out_sync_timestamp = *((unsigned int *)data);
			}
			else
			{
				ret = (unsigned char)0x00;
			}
		}
		break;		
		
		default:
		break;
	}

	return ret;
}

static int16_t get_int16(uint8_t *buf, uint16_t offset)
{
	int16_t temp = 0;

	for (int8_t i = 1; i >= 0; i--)
	{
		temp <<= 8;
		temp |= buf[offset + i];
	}

	return temp;
}

static int32_t get_int32(uint8_t *buf, uint16_t offset)
{
	int32_t temp = 0;

	for (int8_t i = 3; i >= 0; i--)
	{
		temp <<= 8;
		temp |= buf[offset + i];
	}

	return temp;
}

static int64_t get_int64(uint8_t *buf, uint16_t offset)
{
	int64_t temp = 0;

	for (int8_t i = 7; i >= 0; i--)
	{
		temp <<= 8;
		temp |= buf[offset + i];
	}

	return temp;
}
