/* File implements methods for the mpu motion controller */
#include <stdlib.h>					// include c standard
#include "mpu_control.h"								// file header
#include "../Peripherals/i2c_control.h" // communication method
#include "../Peripherals/Display.h"
#include "../timer.h"


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

volatile boolean new_mpu_data;
volatile uint32_t num_data_interrupts; /* Avoid reading data for every mpu intpt */
mpu_data_s *average_data;	/* Keep a running average of the current data */
boolean is_initialized;   /* True if mpu has been initialized to default values */
uint8_t data_packet_size;


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

	NVIC_SetPriority (EXTI9_5_IRQn, 0x07); 			/* set priority to lower than i2c */
	NVIC_DisableIRQ(EXTI9_5_IRQn);							/* we don't want to interrupt during setup */
	
	/* Check to see if MPU is already loaded (from pre-CPU reset) */
	is_Started = mpu_isInitialized();
	
	// restart device 
	enQueue(mpuTxQueue, reg.pwr_mgmt_1);				// enqueue the register number
	enQueue(mpuTxQueue, BIT_RESET);							// enqueue the new value 
	mpu_writeRegister(1, reg.pwr_mgmt_1, false); // send to i2c interface
	
	Delay(1000);// wait for reset
	// wakeup device on restart
	mpu_regs->pwr_mgmt_1 &= MPU_WAKE_UP;
	enQueue(mpuTxQueue, reg.pwr_mgmt_1);				// enqueue the register number
	enQueue(mpuTxQueue, mpu_regs->pwr_mgmt_1);	// enqueue the new value 
	mpu_writeRegister(1, reg.pwr_mgmt_1, false); // send to i2c interface
	
	Delay(500);// wait for wakeup
	mpu_readRegister(1, reg.pwr_mgmt_1);
	while(queue_isEmpty(mpuRxQueue)){ }
	display_Int(deQueue(mpuRxQueue), 0, 0, true);
	
	/* Once device is on and awake, set it to default config */
	startup_config = malloc(sizeof *startup_config);
	startup_config->rate_div = STARTUP_RATE_DIV;
	startup_config->lpf = STARTUP_LPF;
	startup_config->user_ctrl = STARTUP_USERCTRL;
	startup_config->accel_cfg = STARTUP_ACCELCFG;
	startup_config->fifo_en = STARTUP_FIFOEN;
	startup_config->gyro_cfg = STARTUP_GYROCFG;
	startup_config->int_enable = STARTUP_INTNENABLE;
	startup_config->int_pin_cfg = STARTUP_INTPINCFG;
	
	configure_Mpu(startup_config);			/* call the function to write values */
	
	free(startup_config);								/* release memory */
	//NVIC_ClearPendingIRQ(EXTI9_5_IRQn);	/* clear any initial interrupts */
	NVIC_EnableIRQ(EXTI9_5_IRQn);				/* turn interrupts back on */

}

void configure_Mpu(mpu_setup_s *config){
	uint16_t numBytes;
	
	/* Configure the LPF and gyros/accels */
	enQueue(mpuTxQueue, reg.rate_div); 					/* enqueue the register */
	enQueue(mpuTxQueue, config->rate_div);			/* enqueue values of regs */
	enQueue(mpuTxQueue, config->lpf);						/* enqueue values of regs */
	enQueue(mpuTxQueue, config->gyro_cfg);
	enQueue(mpuTxQueue, config->accel_cfg);
	numBytes = 4; 															 /* number of reg VALUES written */
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
	
	data_packet_size = 0;
	if(mpu_regs->fifo_en & BIT_FIFO_EN_XYZG)
		data_packet_size += 3;
	if(mpu_regs->fifo_en & BIT_FIFO_EN_XYZA)
		data_packet_size += 3;

	
	while(!queue_isEmpty(mpuTxQueue)){} 				/* wait until the setup sequence is sent */
	/* All done */
	
}

void stop_Measuring(){
	uint8_t numBytes;
	mpu_regs->fifo_en &= ~(BIT_FIFO_EN_XYZA | BIT_FIFO_EN_XYZG);
	mpu_regs->user_ctrl &= ~(BIT_FIFO_EN);
	enQueue(mpuTxQueue, reg.fifo_en);			  			/* enqueue the register */
	enQueue(mpuTxQueue, mpu_regs->fifo_en);				 /* enqueue values of regs */
	numBytes = 1; 															 /* number of reg VALUES written */
	mpu_writeRegister(numBytes, reg.fifo_en, false); 
	/* Turn on/off device sectors */
	enQueue(mpuTxQueue, reg.user_ctrl);			  			/* enqueue the register */
	enQueue(mpuTxQueue, mpu_regs->user_ctrl);				 /* enqueue values of regs */
	numBytes = 1; 															 /* number of reg VALUES written */
	mpu_writeRegister(numBytes, reg.user_ctrl, false); /* can burst write consecutive regs */

	stop_Timer();
}
void start_Measuring(){
	uint8_t numBytes;
	mpu_regs->fifo_en |= (BIT_FIFO_EN_XYZA | BIT_FIFO_EN_XYZG); /* Turn on sensors */
	mpu_regs->user_ctrl |= (BIT_FIFO_EN);					/* turn on the fifo for data */
	enQueue(mpuTxQueue, reg.fifo_en);			  			/* enqueue the register */
	enQueue(mpuTxQueue, mpu_regs->fifo_en);				 /* enqueue values of regs */
	numBytes = 1; 															 /* number of reg VALUES written */
	mpu_writeRegister(numBytes, reg.fifo_en, false); 
	/* Turn on/off device sectors */
	enQueue(mpuTxQueue, reg.user_ctrl);			  			/* enqueue the register */
	enQueue(mpuTxQueue, mpu_regs->user_ctrl);				 /* enqueue values of regs */
	numBytes = 1; 															 /* number of reg VALUES written */
	mpu_writeRegister(numBytes, reg.user_ctrl, false); /* can burst write consecutive regs */

	init_Timer(MS_PER_SAMPLE);	/* Set timer to periodically update motion */
	start_Timer();
	
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
uint8_t get_Data_Packet(mpu_data_s *data, data_packet_time packet){
	uint8_t high_byte, low_byte;
	do{
		if(data_Packet_Ready()){
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
		/* To get the newest data packet, we need to run through the old ones */
	}while(data_Packet_Ready() && packet == NEWEST_DATA_PACKET);
	
	return 0;
}

boolean data_Packet_Ready(){
	uint8_t packet_size = 0;
	/* Make sure that the queue has a full data packet */
	if(mpu_regs->fifo_en & BIT_FIFO_EN_XYZG)
		packet_size += 3;
	if(mpu_regs->fifo_en & BIT_FIFO_EN_XYZA)
		packet_size += 3;
	if(mpuRxQueue->currentSize >= packet_size && is_initialized){
		return true;
	}
	return false;
}

void display_Register(uint16_t reg, uint16_t value){

}

void display_Motion_Data(mpu_data_s *data){
	display_String("Ax", ACCELX_DISPLAY_PAGE, DATA_LABELS_COLSTART, false);
	display_Int(data->accel_x, ACCELX_DISPLAY_PAGE, MOTION_DATA_COLSTART, true);
	display_String("Ay", ACCELY_DISPLAY_PAGE, DATA_LABELS_COLSTART, false);
	display_Int(data->accel_y, ACCELX_DISPLAY_PAGE, MOTION_DATA_COLSTART, true);
	display_String("Az", ACCELZ_DISPLAY_PAGE, DATA_LABELS_COLSTART, false);
	display_Int(data->accel_z, ACCELX_DISPLAY_PAGE, MOTION_DATA_COLSTART, true);
	display_String("Gx", GYROX_DISPLAY_PAGE, DATA_LABELS_COLSTART, false);
	display_Int(data->gyro_x, GYROX_DISPLAY_PAGE, MOTION_DATA_COLSTART, true);
	display_String("Gy", GYROY_DISPLAY_PAGE, DATA_LABELS_COLSTART, false);
	display_Int(data->gyro_y, GYROY_DISPLAY_PAGE, MOTION_DATA_COLSTART, true);
	display_String("Gz", GYROZ_DISPLAY_PAGE, DATA_LABELS_COLSTART, false);
	display_Int(data->gyro_z, GYROZ_DISPLAY_PAGE, MOTION_DATA_COLSTART, true);
	
}

/* This function acts as an accessor to the local motion data for external *
 * functions and files */
void display_New_Data(){
	display_Motion_Data(average_data);
}

void TIM15_IRQHandler(void) {
		mpu_readRegister(3, reg.raw_accel); /* Read the raw data registers */
		new_mpu_data = true;
		mpu_readRegister(3, reg.raw_gyro);
		while(!data_Packet_Ready()){}
		get_Data_Packet(average_data, NEWEST_DATA_PACKET); /* save the value */
		TIM15->SR &=  ~(TIM_SR_UIF |TIM_SR_CC2IF | TIM_SR_CC1IF); // reset status register
}


/* MPU-9150 external interrupt handler function */
void EXTI9_5_IRQHandler(void){
	uint16_t count;
	uint16_t  interrupt;
	if(num_data_interrupts >= MINIMUM_DATA_TRANSFER){
		if(!(MPU_I2C->ISR & I2C_ISR_BUSY) && !(mpuTxQueue->currentSize)){
		count = data_packet_size + num_data_interrupts;
		mpu_readRegister(data_packet_size, reg.fifo_r_w); // finally read all the data!
		num_data_interrupts = 0;
//		mpu_readRegister(1, reg.int_status);		/* clear mpu status register */
//		while(!(mpuRxQueue->currentSize)){}
//		interrupt = deQueue(mpuRxQueue);		/* read status bits */
//		if(num_data_interrupts >= MINIMUM_DATA_TRANSFER){
//			if(!interrupt || interrupt == BIT_FIFO_OVERFLOW){
//				/* ruh roh, sampling too slow */
//			}else{
//				mpu_readRegister(2, reg.fifo_count_h);
//				
//				while(!(mpuRxQueue->currentSize)){}
//				count = deQueue(mpuRxQueue) & 0x3;	// get high bits
//				while(!(mpuRxQueue->currentSize)){}
//				count <<= 8; 	// shift to high byte
//				count += deQueue(mpuRxQueue);	// get lower byte
//				if(count > MINIMUM_DATA_TRANSFER){	// If new data available
//					mpu_readRegister(count, reg.fifo_r_w); // finally read all the data!
//					num_data_interrupts = 0; /* Reset count */
//				}
//				is_initialized = true;									/* Reading from mpu, its init'd */
//				}
//			}
		}else{
			num_data_interrupts += data_packet_size;
		}
	}else{
		num_data_interrupts += data_packet_size;
	}
}

