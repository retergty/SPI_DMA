#ifndef __MPU9250_MY
#define __MPU9250_MY

#include "main.h"

// ����MPU9250�ڲ��Ĵ���
#define MPU9250_REG_SELF_TEST_X_GYRO    0x00        /**< gyro self test x register */
#define MPU9250_REG_SELF_TEST_Y_GYRO    0x01        /**< gyro self test y register */
#define MPU9250_REG_SELF_TEST_Z_GYRO    0x02        /**< gyro self test z register */
#define MPU9250_REG_SELF_TEST_X_ACCEL   0x0D        /**< accel self test x register */
#define MPU9250_REG_SELF_TEST_Y_ACCEL   0x0E        /**< accel self test y register */
#define MPU9250_REG_SELF_TEST_Z_ACCEL   0x0F        /**< accel self test z register */
#define MPU9250_REG_XG_OFFSET_H         0x13        /**< gyro offset x high register */
#define MPU9250_REG_XG_OFFSET_L         0x14        /**< gyro offset x low register */
#define MPU9250_REG_YG_OFFSET_H         0x15        /**< gyro offset y high register */
#define MPU9250_REG_YG_OFFSET_L         0x16        /**< gyro offset y low register */
#define MPU9250_REG_ZG_OFFSET_H         0x17        /**< gyro offset z high register */
#define MPU9250_REG_ZG_OFFSET_L         0x18        /**< gyro offset z low register */
#define MPU9250_REG_SMPRT_DIV           0x19        /**< smprt div register */
#define MPU9250_REG_CONFIG              0x1A        /**< configure register */
#define MPU9250_REG_GYRO_CONFIG         0x1B        /**< gyro configure register */
#define MPU9250_REG_ACCEL_CONFIG        0x1C        /**< accel configure register */
#define MPU9250_REG_ACCEL_CONFIG2       0x1D        /**< accel configure 2 register */
#define MPU9250_REG_LP_ACCEL_ODR        0x1E        /**< low power accel odr register */
#define MPU9250_REG_WOM_THR             0x1F        /**< wom threshold register */
#define MPU9250_REG_MOTION_DURATION     0x20        /**< motion duration register */
#define MPU9250_REG_FIFO_EN             0x23        /**< fifo enable register */
#define MPU9250_REG_I2C_MST_CTRL        0x24        /**< i2c master ctrl register */
#define MPU9250_REG_I2C_MST_STATUS      0x36        /**< i2c master status register */
#define MPU9250_REG_I2C_MST_DELAY_CTRL  0x67        /**< i2c master delay ctrl register */
#define MPU9250_REG_I2C_SLV0_ADDR       0x25        /**< iic slave0 address register */
#define MPU9250_REG_I2C_SLV0_REG        0x26        /**< iic slave0 reg register */
#define MPU9250_REG_I2C_SLV0_CTRL       0x27        /**< iic slave0 ctrl register */
#define MPU9250_REG_I2C_SLV0_DO         0x63        /**< iic slave0 do register */
#define MPU9250_REG_I2C_SLV1_ADDR       0x28        /**< iic slave1 address register */
#define MPU9250_REG_I2C_SLV1_REG        0x29        /**< iic slave1 reg register */
#define MPU9250_REG_I2C_SLV1_CTRL       0x2A        /**< iic slave1 ctrl register */
#define MPU9250_REG_I2C_SLV1_DO         0x64        /**< iic slave1 do register */
#define MPU9250_REG_I2C_SLV2_ADDR       0x2B        /**< iic slave2 address register */
#define MPU9250_REG_I2C_SLV2_REG        0x2C        /**< iic slave2 reg register */
#define MPU9250_REG_I2C_SLV2_CTRL       0x2D        /**< iic slave2 ctrl register */
#define MPU9250_REG_I2C_SLV2_DO         0x65        /**< iic slave2 do register */
#define MPU9250_REG_I2C_SLV3_ADDR       0x2E        /**< iic slave3 address register */
#define MPU9250_REG_I2C_SLV3_REG        0x2F        /**< iic slave3 reg register */
#define MPU9250_REG_I2C_SLV3_CTRL       0x30        /**< iic slave3 ctrl register */
#define MPU9250_REG_I2C_SLV3_DO         0x66        /**< iic slave3 do register */
#define MPU9250_REG_I2C_SLV4_ADDR       0x31        /**< iic slave4 address register */
#define MPU9250_REG_I2C_SLV4_REG        0x32        /**< iic slave4 reg register */
#define MPU9250_REG_I2C_SLV4_CTRL       0x34        /**< iic slave4 ctrl register */
#define MPU9250_REG_I2C_SLV4_DO         0x33        /**< iic slave4 do register */
#define MPU9250_REG_I2C_SLV4_DI         0x35        /**< iic slave4 di register */
#define MPU9250_REG_EXT_SENS_DATA_00    0x49        /**< extern sensor data 00 register */
#define MPU9250_REG_INT_PIN_CFG         0x37        /**< interrupt pin configure register */
#define MPU9250_REG_INT_ENABLE          0x38        /**< interrupt enable register */
#define MPU9250_REG_INT_STATUS          0x3A        /**< interrupt status register */
#define MPU9250_REG_ACCEL_XOUT_H        0x3B        /**< accel xout high register */
#define MPU9250_REG_ACCEL_XOUT_L        0x3C        /**< accel xout low register */
#define MPU9250_REG_ACCEL_YOUT_H        0x3D        /**< accel yout high register */
#define MPU9250_REG_ACCEL_YOUT_L        0x3E        /**< accel yout low register */
#define MPU9250_REG_ACCEL_ZOUT_H        0x3F        /**< accel zout high register */
#define MPU9250_REG_ACCEL_ZOUT_L        0x40        /**< accel zout low register */
#define MPU9250_REG_TEMP_OUT_H          0x41        /**< temp high register */
#define MPU9250_REG_TEMP_OUT_L          0x42        /**< temp low register */
#define MPU9250_REG_GYRO_XOUT_H         0x43        /**< gyro xout high register */
#define MPU9250_REG_GYRO_XOUT_L         0x44        /**< gyro xout low register */
#define MPU9250_REG_GYRO_YOUT_H         0x45        /**< gyro yout high register */
#define MPU9250_REG_GYRO_YOUT_L         0x46        /**< gyro yout low register */
#define MPU9250_REG_GYRO_ZOUT_H         0x47        /**< gyro zout high register */
#define MPU9250_REG_GYRO_ZOUT_L         0x48        /**< gyro zout low register */
#define MPU9250_REG_SIGNAL_PATH_RESET   0x68        /**< signal path reset register */
#define MPU9250_REG_MOT_DETECT_CTRL     0x69        /**< motion detect ctrl register */
#define MPU9250_REG_USER_CTRL           0x6A        /**< user ctrl register */
#define MPU9250_REG_PWR_MGMT_1          0x6B        /**< power mangement 1 register */
#define MPU9250_REG_PWR_MGMT_2          0x6C        /**< power mangement 2 register */
#define MPU9250_REG_BANK_SEL            0x6D        /**< bank sel register */
#define MPU9250_REG_MEM                 0x6F        /**< memory register */
#define MPU9250_REG_PROGRAM_START       0x70        /**< program start register */
#define MPU9250_REG_FIFO_COUNTH         0x72        /**< fifo count high threshold register */
#define MPU9250_REG_FIFO_COUNTL         0x73        /**< fifo count low threshold register */
#define MPU9250_REG_R_W                 0x74        /**< fifo read write data register */
#define MPU9250_REG_WHO_AM_I            0x75        /**< who am I register */
#define MPU9250_REG_XA_OFFSET_H         0x77        /**< accel offset x high register */
#define MPU9250_REG_XA_OFFSET_L         0x78        /**< accel offset x low register */
#define MPU9250_REG_YA_OFFSET_H         0x7A        /**< accel offset y high register */
#define MPU9250_REG_YA_OFFSET_L         0x7B        /**< accel offset y low register */
#define MPU9250_REG_ZA_OFFSET_H         0x7D        /**< accel offset z high register */
#define MPU9250_REG_ZA_OFFSET_L         0x7E        /**< accel offset z low register */
#define AK8963_REG_WIA                  0x00        /**< device id register */
#define AK8963_REG_INFO                 0x01        /**< information register */
#define AK8963_REG_ST1                  0x02        /**< status 1 register */
#define AK8963_REG_HXL                  0x03        /**< x axis data high register */
#define AK8963_REG_HXH                  0x04        /**< x axis data low register */
#define AK8963_REG_HYL                  0x05        /**< y axis data high register */
#define AK8963_REG_HYH                  0x06        /**< y axis data low register */
#define AK8963_REG_HZL                  0x07        /**< z axis data high register */
#define AK8963_REG_HZH                  0x08        /**< z axis data low register */
#define AK8963_REG_ST2                  0x09        /**< status 2 register */
#define AK8963_REG_CNTL1                0x0A        /**< control 1 register */
#define AK8963_REG_CNTL2                0x0B        /**< control 2 register */
#define AK8963_REG_ASTC                 0x0C        /**< self test register */
#define AK8963_REG_TS1                  0x0D        /**< test 1 register */
#define AK8963_REG_TS2                  0x0E        /**< test 2 register */
#define AK8963_REG_I2CDIS               0x0F        /**< iic disable register */
#define AK8963_REG_ASAX                 0x10        /**< x axis sensitivity adjustment value register */
#define AK8963_REG_ASAY                 0x11        /**< y axis sensitivity adjustment value register */
#define AK8963_REG_ASAZ                 0x12        /**< z axis sensitivity adjustment value register */
#define AK8963_ADDR             0x0C        /**Address AK8963**/
#define I2C_READFLAG                     0x80        /**I2C Read Flag**/
int MPU9250_Init(void);
void MPU9250_Read_Data(uint8_t* DataOut);
void I2C_Mag_ReadTest(uint8_t* DataOut);
void MPU9250IntConfig(void);
void SPI_WriteMPU9250Reg(uint8_t RegAdd, const uint8_t *WriteData, uint32_t size);
void SPI_ReadMPU9250Reg(uint8_t RegAdd, uint8_t *ReadData, uint32_t size);
#endif // !__MPU9250_MY

