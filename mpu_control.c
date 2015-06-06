/* File implements methods for the mpu motion controller */
#include <stdlib.h>					// include c standard
#include "mpu_control.h"								// file header
#include "../Peripherals/i2c_control.h" // communication method
#include "../Peripherals/Display.h"

struct gyro_reg_s *mpu_regs;
const struct gyro_reg_s reg = {
    .who_am_i       = 0x75,
    .rate_div       = 0x19,
    .lpf            = 0x1A,
    .prod_id        = 0x0C,
    .user_ctrl      = 0x6A,
    .fifo_en        = 0x23,
    .gyro_cfg       = 0x1B,
    .accel_cfg      = 0x1C,
    .motion_thr     = 0x1F,
    .motion_dur     = 0x20,
    .fifo_count_h   = 0x72,
    .fifo_r_w       = 0x74,
    .raw_gyro       = 0x43,
    .raw_accel      = 0x3B,
    .temp           = 0x41,
    .int_enable     = 0x38,
    .dmp_int_status = 0x39,
    .int_status     = 0x3A,
    .pwr_mgmt_1     = 0x6B,
    .pwr_mgmt_2     = 0x6C,
    .int_pin_cfg    = 0x37,
    .mem_r_w        = 0x6F,
    .accel_offs     = 0x06,
    .i2c_mst        = 0x24,
    .bank_sel       = 0x6D,
    .mem_start_addr = 0x6E,
    .prgm_start_h   = 0x70
#ifdef AK89xx_SECONDARY
    ,.raw_compass   = 0x49,
    .yg_offs_tc     = 0x01,
    .s0_addr        = 0x25,
    .s0_reg         = 0x26,
    .s0_ctrl        = 0x27,
    .s1_addr        = 0x28,
    .s1_reg         = 0x29,
    .s1_ctrl        = 0x2A,
    .s4_ctrl        = 0x34,
    .s0_do          = 0x63,
    .s1_do          = 0x64,
    .i2c_delay_ctrl = 0x67
#endif
};
const struct hw_s hw = {
    .addr           = 0x68,
    .max_fifo       = 1024,
    .num_reg        = 118,
    .temp_sens      = 340,
    .temp_offset    = -521,
    .bank_size      = 256
#if defined AK89xx_SECONDARY
    ,.compass_fsr    = AK89xx_FSR
#endif
};

void mpu_init(void){
	mpu_setup_s *startup_config;
	boolean is_Started;
	
	// allocate memory and set to zero
	mpu_regs = malloc(sizeof *mpu_regs);
	if(!mpu_regs){
		while(1);
	}
	/* Configure mpu_regs to be a copy  of onboard registers  */
	// mpu actually starts up with two non-zero registers
	mpu_regs->who_am_i = MPU_I2C_ADDRESS;	// initialized with known address
	mpu_regs->pwr_mgmt_1 = BIT_SLEEP; // starts in sleep mode
	/* End configure of mpu_regs */

	NVIC_SetPriority (EXTI9_5_IRQn, 0xff); 			/* set priority to lower than i2c */
	NVIC_DisableIRQ(EXTI9_5_IRQn);							/* we don't want to interrupt during setup */
	
	/* Check to see if MPU is already loaded (from pre-CPU reset) */
	is_Started = mpu_isInitialized();
	
	// restart device 
	enQueue(mpuTxQueue, reg.pwr_mgmt_1);				// enqueue the register number
	enQueue(mpuTxQueue, BIT_RESET);							// enqueue the new value 
	mpu_writeRegister(1, reg.pwr_mgmt_1, false); // send to i2c interface
	
	Delay(500);// wait for reset
	// wakeup device on restart
	mpu_regs->pwr_mgmt_1 &= ~BIT_SLEEP;
	enQueue(mpuTxQueue, reg.pwr_mgmt_1);				// enqueue the register number
	enQueue(mpuTxQueue, mpu_regs->pwr_mgmt_1);	// enqueue the new value 
	mpu_writeRegister(1, reg.pwr_mgmt_1, false); // send to i2c interface
	
	Delay(500);// wait for wakeup
	
	/* Once device is on and awake, set it to default config */
	startup_config = malloc(sizeof *startup_config);
	startup_config->lpf = STARTUP_LPF;
	startup_config->accel_cfg = STARTUP_ACCELCFG;
	startup_config->fifo_en = STARTUP_FIFOEN;
	startup_config->gyro_cfg = STARTUP_GYROCFG;
	startup_config->int_enable = STARTUP_INTENABLE;
	startup_config->int_pin_cfg = STARTUP_INTPINCFG;
	
	configure_Mpu(startup_config);			/* call the function to write values */
	
	free(startup_config);								/* release memory */
	NVIC_ClearPendingIRQ(EXTI9_5_IRQn);	/* clear any initial interrupts */
	NVIC_EnableIRQ(EXTI9_5_IRQn);				/* turn interrupts back on */

}

void configure_Mpu(mpu_setup_s *config){
	uint16_t numBytes;
	
	/* Configure the LPF and gyros/accels */
	enQueue(mpuTxQueue, reg.lpf);								/* enqueue the register */
	enQueue(mpuTxQueue, config->lpf);						/* enqueue values of regs */
	enQueue(mpuTxQueue, config->gyro_cfg);
	enQueue(mpuTxQueue, config->accel_cfg);
	numBytes = 3; 															 /* number of reg VALUES written */
	mpu_writeRegister(numBytes, reg.lpf, false); /* can burst write consecutive regs */
	/* Setup the fifo buffer on mpu */
	enQueue(mpuTxQueue, reg.fifo_en);			  			/* enqueue the register */
	enQueue(mpuTxQueue, config->fifo_en);				 /* enqueue values of regs */
	numBytes = 1; 															 /* number of reg VALUES written */
	mpu_writeRegister(numBytes, reg.fifo_en, false); /* can burst write consecutive regs */
	/* Turn on/off device sectors */
	enQueue(mpuTxQueue, reg.user_ctrl);			  			/* enqueue the register */
	enQueue(mpuTxQueue, config->user_ctrl);				 /* enqueue values of regs */
	numBytes = 1; 															 /* number of reg VALUES written */
	mpu_writeRegister(numBytes, reg.user_ctrl, false); /* can burst write consecutive regs */

	/* Configure interrupts and interrupt sources */
	enQueue(mpuTxQueue, reg.int_pin_cfg);			   /* enqueue the register */
	enQueue(mpuTxQueue, config->int_pin_cfg);		 /* enqueue values of regs */
	enQueue(mpuTxQueue, config->int_enable);	  	 /* enqueue values of regs */
	numBytes = 2; 															 /* number of reg VALUES written */
	mpu_writeRegister(numBytes, reg.int_pin_cfg, false); /* can burst write consecutive regs */

	/* make sure to update local copy of configuration registers */ 
	mpu_regs->lpf = config->lpf;
	mpu_regs->gyro_cfg = config->gyro_cfg;
	mpu_regs->accel_cfg = config->accel_cfg;
	mpu_regs->fifo_en = config->fifo_en;
	mpu_regs->int_pin_cfg = config->int_pin_cfg;
	mpu_regs->int_enable = config->int_enable;
	mpu_regs->user_ctrl = config->user_ctrl;
	
	//while(!queue_isEmpty(mpuTxQueue)){} 				/* wait until the setup sequence is sent */
	/* All done */
	
}
void init_Gyroscope(){
	
}
void init_Accelerometer(){
	
}
void init_Compass(){
	
}
void configure_AccelRange(accelRange range){
	mpu_regs->accel_cfg &= ~BITS_FSR;
	mpu_regs->accel_cfg |= range;
	enQueue(mpuTxQueue, reg.accel_cfg);				// enqueue the register number
	enQueue(mpuTxQueue, mpu_regs->accel_cfg);	// enqueue the new value 
	mpu_writeRegister(1, reg.accel_cfg, false); // send to i2c interface
	
}
/* Function returns a data structure that hols all motion data. Only some fields may be *
 * used depending on the settings of the MPU 		*/
mpu_data_s * get_Data_Packet(){
	mpu_data_s *data = malloc(sizeof(mpu_data_s));
	uint8_t high_byte, low_byte;
	uint8_t packet_size = 0;
	/* Make sure that the queue has a full data packet */
	if(mpu_regs->fifo_en & BIT_FIFO_EN_XYZG)
		packet_size += 3;
	if(mpu_regs->fifo_en & BIT_FIFO_EN_XYZA)
		packet_size += 3;
	if(mpuRxQueue->currentSize >= packet_size){
		if(mpu_regs->fifo_en & BIT_FIFO_EN_XYZG){ /* Check if the fifo is sending gyro data */
				high_byte = deQueue(mpuRxQueue); /* Get data from Queue */
				low_byte = deQueue(mpuRxQueue);
				data->gyro_x = (high_byte << 8) | low_byte; /* Save data in correct slot */
				high_byte = deQueue(mpuRxQueue);
				low_byte = deQueue(mpuRxQueue);
				data->gyro_y = (high_byte << 8) | low_byte;
				high_byte = deQueue(mpuRxQueue);
				low_byte = deQueue(mpuRxQueue);
				data->gyro_z = (high_byte << 8) | low_byte;
			}	
		if(mpu_regs->fifo_en & BIT_FIFO_EN_XYZA){		/* Check if the fifo is sending accel data */
				high_byte = deQueue(mpuRxQueue); /* Get data from Queue */
				low_byte = deQueue(mpuRxQueue);
				data->accel_x = (high_byte << 8) | low_byte; /* Save data in correct slot */
				high_byte = deQueue(mpuRxQueue);
				low_byte = deQueue(mpuRxQueue);
				data->accel_y = (high_byte << 8) | low_byte;
				high_byte = deQueue(mpuRxQueue);
				low_byte = deQueue(mpuRxQueue);
				data->accel_z = (high_byte << 8) | low_byte;
		}		
	}else {
		/* TODO: Error handling */
	}
	return data;
}

void display_Register(uint16_t reg, uint16_t value){
	display_Int(reg);
	display_Int(value);
}

/* MPU-9150 external interrupt handler function */
void EXTI9_5_IRQHandler(void){
	uint16_t count;
	uint16_t  interrupt;
	mpu_readRegister(1, reg.int_status);		/* clear mpu status register */
	while(queue_isEmpty(mpuRxQueue)){}
	interrupt = deQueue(mpuRxQueue);		/* read status bits */
		
	mpu_readRegister(2, reg.fifo_count_h);
	
	while(queue_isEmpty(mpuRxQueue)){}
	count = deQueue(mpuRxQueue) & 0x3;	// get high bits
	while(queue_isEmpty(mpuRxQueue)){}
	count <<= 8; 	// shift to high byte
	count = deQueue(mpuRxQueue);	// get lower byte
	if(count > 0){	// If new data available
		mpu_readRegister(count, reg.fifo_r_w); // finally read all the data!
	}
}

