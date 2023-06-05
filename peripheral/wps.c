/*************************************************************************
	> File Name: peripheral/wps.c
	> Author: 
	> Mail: 
	> Created Time: 2015年04月11日 星期六 15时52分12秒
 ************************************************************************/

#include <common.h>
#include <command.h>

#include <wps.h>
#define WPSKEY (*(unsigned long*)0xb0000604) // GPIO#18
#define WPSDATA (*(unsigned long *)0xb0000624) 
#define GPIO1_MODE_REG (*(volatile unsigned int *)0xb0000060)

#define LED_MODE_REG  0xb0000064
#define GPIO_MODE_REG 0xb0000060

#define GPIO_CTRL_0_REG 0xb0000600 // (GPIO#0--GPIO#31)
#define GPIO_CTRL_1_REG 0xb0000604 // (GPIO#32-GPIO#63)

#define GPIO_DATA_0_REG 0xb0000620
#define GPIO_DATA_1_REG 0xb0000624

#define GPIO_DATASET_0_REG 0xb0000630
#define GPIO_DATASET_1_REG 0xb0000634

#define GPIO_DATACLR_0_REG 0xb0000640
#define GPIO_DATACLR_1_REG 0xb0000644

#define SET_REG_VALUE(x) (*(volatile unsigned int *)x)

/* 

 SDA is pin 5
 SCL is pin 4

 */

#define I2C_ADDR 0x30

#define I2C_START 'S'
#define I2C_WRITE 'W'
#define I2C_READ  'R'
#define I2C_READ_NACK  'N'
#define I2C_STOP  'P'

void init_wps(void)
{
	GPIO1_MODE_REG |= (1<<14);
    WPSKEY &=(~(1<<6));	//GPIO#44
}
int readwps(void)//按下输出0,未按下输出1
{
    return (WPSDATA & (1<<6));
}

void bgn_init_gpio(void)
{
	/* First disable
		 -CS 1             (Bit 4)
		 -I2S              (Bit 6)
		 -I2C              (Bit 20)
		 -UART 1           (Bit 24)
		 -watchdog timeout (Bit 14) */
	SET_REG_VALUE(GPIO_MODE_REG) |=
		  (1 << 4)
		| (1 << 6)
		| (1 << 20)
		| (1 << 24)
		| (1 << 14);

	SET_REG_VALUE(LED_MODE_REG) &= 0xFFFFF000;
	SET_REG_VALUE(LED_MODE_REG) |= 0x00000555;

	/* The following LEDs are already disabled on powerup
	  -WLED
	  -ELED2
	  -ELED3
	  -ELED4 */
}

void bgn_set_gpio(int gpio_num, int value){
	/* Always force these gpio settings. */
	SET_REG_VALUE(LED_MODE_REG) &= 0xFFFFF000;
	SET_REG_VALUE(LED_MODE_REG) |= 0x00000555;

	SET_REG_VALUE(GPIO_MODE_REG) |=
		  (1 << 4)
		| (1 << 6)
		| (1 << 20)
		| (1 << 24)
		| (1 << 14);

	if(gpio_num <32){
		if(value == 0){
			SET_REG_VALUE(GPIO_CTRL_0_REG) &= ~(0x1 << gpio_num);
			//SET_REG_VALUE(GPIO_DATA_0_REG) &= ~(0x1 << gpio_num);
		}
		else if(1 == value){
			SET_REG_VALUE(GPIO_CTRL_0_REG) |= (0x1 << gpio_num);
			SET_REG_VALUE(GPIO_DATASET_0_REG) = 0x1 << gpio_num;
		}
		else if(2 == value){
			SET_REG_VALUE(GPIO_CTRL_0_REG) |= (0x1 << gpio_num);
			SET_REG_VALUE(GPIO_DATACLR_0_REG) = 0x1 << gpio_num;
		}
	}
	else{
		gpio_num -= 32;
		if(value == 0){
			SET_REG_VALUE(GPIO_CTRL_1_REG) &= ~(0x1 << gpio_num);
			//SET_REG_VALUE(GPIO_DATA_1_REG) &= ~(0x1 << gpio_num);
		}
		else if(1 == value){
			SET_REG_VALUE(GPIO_CTRL_1_REG) |= (0x1 << gpio_num);
			SET_REG_VALUE(GPIO_DATASET_1_REG) = 0x1 << gpio_num;
		}
		else if(2 == value){
			SET_REG_VALUE(GPIO_CTRL_1_REG) |= (0x1 << gpio_num);
			SET_REG_VALUE(GPIO_DATACLR_1_REG) = 0x1 << gpio_num;
		}
	}
}

unsigned bgn_read_gpio(int gpio_num){
	/* Force to input */
	bgn_set_gpio(gpio_num, 0);

	unsigned ret = 0;
	if(gpio_num <32){
		ret = SET_REG_VALUE(GPIO_DATA_0_REG);
		ret >>= gpio_num;
	}
	else{
		gpio_num -= 32;
		ret = SET_REG_VALUE(GPIO_DATA_1_REG);
		ret >>= gpio_num;
	}
	return ret & 0x1;
}



/* 10 Khz clock rate, 100 us */
void I2C_delay() {
	udelay(100);
}

unsigned read_SCL(void){
	return bgn_read_gpio(4);
}

unsigned read_SDA(void){
	return bgn_read_gpio(5);
}

void clear_SCL(void){
	bgn_set_gpio(4, 2);
}

void clear_SDA(void){
	bgn_set_gpio(5, 2);
}

unsigned clock_stretch(void){

	unsigned countdown = 400;
  while (read_SCL() == 0) {
		udelay(50);
		--countdown;
		if(0 == countdown)
			return 1;
  }

	return 0;
}

/* Write a byte to I2C bus.
Return 0 if ack by the slave.
Nonzero on NACK
*/
unsigned i2c_write_byte(unsigned char byte) {
  unsigned bit;

  for (bit = 0; bit < 8; bit++) {
		/* Set SDA and let it settle */
		if (byte & 0x80) {
			read_SDA();
		} else {
			clear_SDA();
		}
		udelay(15);

		/* Let SCL go high */
		read_SCL();
		udelay(15);

		/* Force SCL low again */
		clear_SCL();
		udelay(15);
		byte <<= 1;
  }

	read_SDA();
	udelay(15);

	if(clock_stretch()){
		/* Slave is stuck */
		return 2;
	}

	/* read SDA value and drop SCL back to low. */
	unsigned ret = read_SDA();
	clear_SCL();

	udelay(15);

  return ret;
}

// Read a byte from I2C bus
unsigned char i2c_read_byte(unsigned nack) {
  unsigned char byte = 0;
	unsigned i;
  for (i = 0; i < 8; i++) {
		/* Let the slave drive data */
		read_SDA();
		udelay(15);

		/* Release SCL */
		read_SCL();
		udelay(15);

		byte <<= 1;
		byte |= read_SDA();
		udelay(15);

		clear_SCL();
		udelay(15);
  }

	/*  */
	if(nack){
		read_SDA();
	} else {
		clear_SDA();
	}
	udelay(15);

	/* Let SCL go high */
	read_SCL();
	udelay(15);

	/* Force SCL low again */
	clear_SCL();

  return byte;
}

unsigned parse_byte(const char* cmd){
	unsigned ret = 0x300;
	if(cmd[0] >= '0' && cmd[0] <= '9'){
		ret &= 0x1FF;
		ret |= (cmd[0] - '0') << 4;
	}
	if(cmd[0] >= 'a' && cmd[0] <= 'f'){
		ret &= 0x1FF;
		ret |= (cmd[0] - 'a') << 4;
	}

	if(cmd[1] >= '0' && cmd[1] <= '9'){
		ret &= 0x2FF;
		ret |= cmd[1] - '0';
	}
	if(cmd[1] >= 'a' && cmd[1] <= 'f'){
		ret &= 0x2FF;
		ret |= cmd[1] - 'a';
	}

	return ret;
}

int i2ccmd(const char* cmd, uint8_t* rdbuff){
	const char* cur = cmd;
	uint8_t* output = rdbuff;
	unsigned started = 0;

	while(*cur){
		if(I2C_START == *cur){
			++cur;
			if (started) {
				printf("I2C: REP START\n");
				// if started, do a restart cond
				// set SDA to 1
				read_SDA();
				I2C_delay();
				if(clock_stretch()){
					printf("I2C: CLK.STRETCH\n");
					return 1;
				}

				// Repeated start setup time, minimum 4.7us
				I2C_delay();
			}
			else{
				unsigned sda = read_SDA();
				unsigned scl = read_SCL();
				if(!sda || !scl){
					printf("I2C: BAD START SDA %u SCL %u\n", sda, scl);
					return 1;
				}
				printf("I2C: START\n");
			}

			// SCL is high, set SDA from 1 to 0.
			clear_SDA();
			I2C_delay();
			clear_SCL();
			I2C_delay();

  		started = 1;
		}
		else if(I2C_WRITE == *cur){
			++cur;

			/* Send the address byte with ~WR == 0 */
			unsigned wret = i2c_write_byte(I2C_ADDR << 1);
			if(wret){
				printf("I2C: WR.NACK %u\n", wret);
				return 2;
			}

			while(1){
				unsigned byte = parse_byte(cur);
				if(byte & 0x300)
					break;
				cur += 2;

				printf("I2C: WR.%x\n", byte);
				if(i2c_write_byte(byte)){
					printf("I2C: WR.NACK\n");
					/* NACK or clock timeout */
					return 2;
				}
			}
		}
		else if(I2C_READ == *cur || I2C_READ_NACK == *cur){

			/* Send the address byte with WR == 1, to read data */

			unsigned wret = i2c_write_byte((I2C_ADDR << 1) | 0x1);
			if(wret){
				printf("I2C: RD.NACK %u\n", wret);
				return 2;
			}

			/* The first byte returned is the number of bytes to read. */
			unsigned count = i2c_read_byte(0);

			unsigned i;
			for(i = 0; i < count; ++i){
				if(i < count - 1 || I2C_READ == *cur)
					*output = i2c_read_byte(0);
				else
					*output = i2c_read_byte(1);
				printf("I2C: RD.%x\n", *output);
				++output;
			}
			++cur;
		}
		else if(I2C_STOP == *cur){
			printf("I2C: STOP\n");
			++cur;

			clear_SDA();
			I2C_delay();
			if(clock_stretch()){
				printf("I2C: CLK.STRETCH\n");
				return 4;
			}

			// Stop bit setup time, minimum 4us
			I2C_delay();
			started = 0;
			read_SDA();
			I2C_delay();
		}
	}

	return 0;
}


/* Process an i2c command string */
int do_i2ccmd(cmd_tbl_t *cmdtp, int flag, int argc, char *argv[]){
	if(argc < 2){
		printf ("Usage:\n%s\n", cmdtp->usage);
		return 0;
	}

	read_SCL();
	read_SDA();
	I2C_delay();

	uint8_t response[64];
	int ret = i2ccmd(argv[1], response);

	read_SCL();
	read_SDA();

	return ret;
}


int do_gpioset (cmd_tbl_t *cmdtp, int flag, int argc, char *argv[]){

	if(argc < 3){
		printf ("Usage:\n%s\n", cmdtp->usage);
		return 0;
	}

	unsigned gpio_num = simple_strtoul(argv[1], NULL, 10);
	unsigned value = simple_strtoul(argv[2], NULL, 10);

	if(gpio_num >= 64){
		printf("Bad gpio\n");
		return 0;
	}

	if(value > 2){
		printf("Bad value\n");
		return 0;
	}

	bgn_set_gpio(gpio_num, value);
	return 0;
}

U_BOOT_CMD(
	i2ccmd,	2,	1,	do_i2ccmd,
	"i2ccmd - perform i2c transactions\n",
	"\ni2ccmd CMDSTR\n    - \n"
);


U_BOOT_CMD(
	gpioset,	3,	1,	do_gpioset,
	"gpioset   - assert/deassert gpio pin\n",
	"\ngpioset GPIO_NUM VALUE\n    - \n"
);



