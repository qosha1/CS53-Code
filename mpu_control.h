#include <stdint.h>

#ifndef MPU_REGISTERS_H
#define MPU_REGISTERS_H

/* Hardware registers needed by driver. */
struct gyro_reg_s {
    unsigned char who_am_i;
    unsigned char rate_div;
    unsigned char lpf;
    unsigned char prod_id;
    unsigned char user_ctrl;
    unsigned char fifo_en;
    unsigned char gyro_cfg;
    unsigned char accel_cfg;
    unsigned char accel_cfg2;
    unsigned char lp_accel_odr;
    unsigned char motion_thr;
    unsigned char motion_dur;
    unsigned char fifo_count_h;
    unsigned char fifo_r_w;
    unsigned char raw_gyro;
    unsigned char raw_accel;
    unsigned char temp;
    unsigned char int_enable;
    unsigned char dmp_int_status;
    unsigned char int_status;
    unsigned char accel_intel;
    unsigned char pwr_mgmt_1;
    unsigned char pwr_mgmt_2;
    unsigned char int_pin_cfg;
    unsigned char mem_r_w;
    unsigned char accel_offs;
    unsigned char i2c_mst;
    unsigned char bank_sel;
    unsigned char mem_start_addr;
    unsigned char prgm_start_h;
#if defined AK89xx_SECONDARY
    unsigned char s0_addr;
    unsigned char s0_reg;
    unsigned char s0_ctrl;
    unsigned char s1_addr;
    unsigned char s1_reg;
    unsigned char s1_ctrl;
    unsigned char s4_ctrl;
    unsigned char s0_do;
    unsigned char s1_do;
    unsigned char i2c_delay_ctrl;
    unsigned char raw_compass;
    /* The I2C_MST_VDDIO bit is in this register. */
    unsigned char yg_offs_tc;
#endif
};
typedef struct  {
    unsigned char lpf;					/* Low pass filter for Accel and gyro*/
    unsigned char user_ctrl;		/* Enable and reset FIFO, I2C_MASTER */
    unsigned char fifo_en;			/* Enable what gets sent to FIFO */
    unsigned char gyro_cfg;			/* Configuration of Gyroscope */
    unsigned char accel_cfg;		/* Configuration of Accelerometer */
    unsigned char int_enable;		/* Configure applicable interrupts */
    unsigned char int_pin_cfg;	/* Configure int pin to OD, PP  */
}mpu_setup_s;

/* Information specific to a particular device. */
struct hw_s {
    unsigned char addr;
    unsigned short max_fifo;
    unsigned char num_reg;
    unsigned short temp_sens;
    short temp_offset;
    unsigned short bank_size;
#if defined AK89xx_SECONDARY
    unsigned short compass_fsr;
#endif
};

typedef struct {
    uint16_t gyro_x;
    uint16_t gyro_y;
    uint16_t gyro_z;
    uint16_t accel_x;
    uint16_t accel_y;
    uint16_t accel_z;
    uint16_t compass_x;
    uint16_t compass_y;
    uint16_t compass_z;
} mpu_data_s;

#define STARTUP_LPF				  (0x00)					// least filtering
#define STARTUP_USERCTRL		BIT_FIFO_EN		  //turn on fifo
#define STARTUP_FIFOEN			(0x78)					// use fifo for all measurements
#define STARTUP_GYROCFG			(0x00)					// use all gyros at small range
#define STARTUP_ACCELCFG		(0x00)					// use all accels at small range
#define STARTUP_INTENABLE		BIT_DATA_RDY_EN | BIT_FIFO_OVERFLOW	// interrupt on daty ready
#define STARTUP_INTPINCFG		(0xA0)					// active low interrupt and wait till cleared

#define BIT_I2C_MST_VDDIO   (0x80)
#define BIT_I2C_MASTER_EN		(0x20)
#define BIT_FIFO_EN         (0x40)
#define BIT_DMP_EN          (0x80)
#define BIT_FIFO_RST        (0x04)
#define BIT_DMP_RST         (0x08)
#define BIT_FIFO_OVERFLOW   (0x10)
#define BIT_DATA_RDY_EN     (0x01)
#define BIT_DMP_INT_EN      (0x02)
#define BIT_MOT_INT_EN      (0x40)
#define BITS_FSR            (0x18)
#define BITS_LPF            (0x07)
#define BITS_HPF            (0x07)
#define BITS_CLK            (0x07)
#define BIT_FIFO_SIZE_1024  (0x40)
#define BIT_FIFO_SIZE_2048  (0x80)
#define BIT_FIFO_SIZE_4096  (0xC0)
#define BIT_RESET           (0x80)
#define BIT_SLEEP           (0x40)
#define BIT_S0_DELAY_EN     (0x01)
#define BIT_S2_DELAY_EN     (0x04)
#define BITS_SLAVE_LENGTH   (0x0F)
#define BIT_SLAVE_BYTE_SW   (0x40)
#define BIT_SLAVE_GROUP     (0x10)
#define BIT_SLAVE_EN        (0x80)
#define BIT_I2C_READ        (0x80)
#define BITS_I2C_MASTER_DLY (0x1F)
#define BIT_AUX_IF_EN       (0x20)
#define BIT_ACTL            (0x80)
#define BIT_LATCH_EN        (0x20)
#define BIT_ANY_RD_CLR      (0x10)
#define BIT_BYPASS_EN       (0x02)
#define BITS_WOM_EN         (0xC0)
#define BIT_LPA_CYCLE       (0x20)
#define BIT_STBY_XA         (0x20)
#define BIT_STBY_YA         (0x10)
#define BIT_STBY_ZA         (0x08)
#define BIT_STBY_XG         (0x04)
#define BIT_STBY_YG         (0x02)
#define BIT_STBY_ZG         (0x01)
#define BIT_STBY_XYZA       (BIT_STBY_XA | BIT_STBY_YA | BIT_STBY_ZA)
#define BIT_STBY_XYZG       (BIT_STBY_XG | BIT_STBY_YG | BIT_STBY_ZG)

typedef enum  /* FSR values for accelerometer settings */
	{
	PLUS_MINUS_TWO 			= 0x00,
	PLUS_MINUS_FOUR		 	= 0x08,
	PLUS_MINUS_EIGHT		= 0x10,
	PLUS_MINUS_SIXTEEN	= 0x18
}accelRange;
	
extern volatile uint32_t msTicks;
extern struct gyro_reg_s *mpu_regs;
extern const struct gyro_reg_s reg;
extern const struct hw_s hw;

mpu_data_s get_Data_Packet();
void display_Register(uint16_t reg, uint16_t value);
void mpu_init(void);
void configure_Mpu(mpu_setup_s *config);
void Delay(uint32_t dlyTicks);
extern void EXTI9_5_IRQHandler(void);

#endif // MPU_REGISTERS_H