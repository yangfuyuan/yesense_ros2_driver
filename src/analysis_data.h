#ifndef ANALYSIS_DATA_H
#define ANALYSIS_DATA_H

/*------------------------------------------------MARCOS
 * define------------------------------------------------*/
#define PROTOCOL_FIRST_BYTE (unsigned char)0x59
#define PROTOCOL_SECOND_BYTE (unsigned char)0x53

#define PROTOCOL_FIRST_BYTE_POS 0
#define PROTOCOL_SECOND_BYTE_POS 1

#define PROTOCOL_TID_LEN 2
#define PROTOCOL_MIN_LEN \
  7 /*header(2B) + tid(2B) + len(1B) + CK1(1B) + CK2(1B)*/

#define CRC_CALC_START_POS 2
#define CRC_CALC_LEN(payload_len) ((payload_len) + 3) /*3 = tid(2B) + \
                                                         len(1B)*/
#define PROTOCOL_CRC_DATA_POS(payload_len) \
  (CRC_CALC_START_POS + CRC_CALC_LEN(payload_len))

#define PAYLOAD_POS 5

#define SINGLE_DATA_BYTES 4

/*data id define*/
#define ACCEL_ID (unsigned char)0x10
#define ANGLE_ID (unsigned char)0x20
#define MAGNETIC_ID (unsigned char)0x30     /*归一化值*/
#define RAW_MAGNETIC_ID (unsigned char)0x31 /*原始值*/
#define EULER_ID (unsigned char)0x40
#define QUATERNION_ID (unsigned char)0x41
#define UTC_ID (unsigned char)0x50
#define SAMPLE_TIMESTAMP_ID (unsigned char)0x51
#define DATA_READY_TIMESTAMP_ID (unsigned char)0x52
#define LOCATION_ID (unsigned char)0x60
#define SPEED_ID (unsigned char)0x70

/*length for specific data id*/
#define ACCEL_DATA_LEN (unsigned char)12
#define ANGLE_DATA_LEN (unsigned char)12
#define MAGNETIC_DATA_LEN (unsigned char)12
#define MAGNETIC_RAW_DATA_LEN (unsigned char)12
#define EULER_DATA_LEN (unsigned char)12
#define QUATERNION_DATA_LEN (unsigned char)16
#define UTC_DATA_LEN (unsigned char)11
#define SAMPLE_TIMESTAMP_DATA_LEN (unsigned char)4
#define DATA_READY_TIMESTAMP_DATA_LEN (unsigned char)4
#define LOCATION_DATA_LEN (unsigned char)12
#define SPEED_DATA_LEN (unsigned char)12

/*factor for sensor data*/
#define NOT_MAG_DATA_FACTOR 0.000001f
#define MAG_RAW_DATA_FACTOR 0.001f

/*factor for gnss data*/
#define LONG_LAT_DATA_FACTOR 0.0000001
#define ALT_DATA_FACTOR 0.001f
#define SPEED_DATA_FACTOR 0.001f

#ifdef __cplusplus
extern "C" {
#endif

/*------------------------------------------------MARCOS
 * define------------------------------------------------*/

/*------------------------------------------------Type
 * define--------------------------------------------------*/
typedef enum {
  crc_err = -3,
  data_len_err = -2,
  para_err = -1,
  analysis_ok = 0,
  analysis_done = 1
} analysis_res_t;

#pragma pack(1)

typedef struct {
  unsigned char header1; /*0x59*/
  unsigned char header2; /*0x53*/
  unsigned short tid;    /*1 -- 60000*/
  unsigned char len;     /*length of payload, 0 -- 255*/
} output_data_header_t;

typedef struct {
  unsigned char data_id;
  unsigned char data_len;
} payload_data_t;

typedef struct {
  float accel_x; /*unit: m/s2*/
  float accel_y;
  float accel_z;

  float angle_x; /*unit: ° (deg)/s*/
  float angle_y;
  float angle_z;

  float mag_x; /*unit: 归一化值*/
  float mag_y;
  float mag_z;

  float raw_mag_x; /*unit: mGauss*/
  float raw_mag_y;
  float raw_mag_z;

  float pitch; /*unit: ° (deg)*/
  float roll;
  float yaw;

  float quaternion_data0;
  float quaternion_data1;
  float quaternion_data2;
  float quaternion_data3;

  double latitude;  /*unit: deg*/
  double longtidue; /*unit: deg*/
  float altidue;    /*unit: m*/

  float vel_n; /*unit: m/s */
  float vel_e;
  float vel_d;

  unsigned int sample_timestamp;     /*unit: us*/
  unsigned int data_ready_timestamp; /*unit: us*/
} protocol_info_t;

#pragma pack()

/*------------------------------------------------------------------------------------------------------------*/
extern protocol_info_t g_output_info;

/*------------------------------------------------Functions
 * declare--------------------------------------------*/
int analysis_data(unsigned char *data, short len);

#ifdef __cplusplus
}
#endif

#endif
