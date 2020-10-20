#include <stdio.h>
#include "adxl345.h"
#include "sys.h"
#include "spi.h"
#include "delay.h"

#define ADXL345_CMD_MULTB	(1 << 6)
#define ADXL345_CMD_READ	(1 << 7)
#define ADXL345_WRITECMD(reg)	(reg & 0x3F)
#define ADXL345_READCMD(reg)	(ADXL345_CMD_READ | (reg & 0x3F))
#define ADXL345_READMB_CMD(reg) (ADXL345_CMD_READ | ADXL345_CMD_MULTB | (reg & 0x3F))

/* accel Register Map */
#define DEVID		0x00	/* R   Device ID */
#define THRESH_TAP	0x1D	/* R/W Tap threshold */
#define DUR		0x21	/* R/W Tap duration */
#define TAP_AXES	0x2A	/* R/W Axis control for tap/double tap */
#define ACT_TAP_STATUS	0x2B	/* R   Source of tap/double tap */
#define BW_RATE		0x2C	/* R/W Data rate and power mode control */
#define POWER_CTL	0x2D	/* R/W Power saving features control */
#define INT_ENABLE	0x2E	/* R/W Interrupt enable control */
#define INT_MAP		0x2F	/* R/W Interrupt mapping control */
#define INT_SOURCE	0x30	/* R   Source of interrupts */
#define DATA_FORMAT	0x31	/* R/W Data format control */
#define DATAX0		0x32	/* R   X-Axis Data 0 */
#define DATAX1		0x33	/* R   X-Axis Data 1 */
#define DATAY0		0x34	/* R   Y-Axis Data 0 */
#define DATAY1		0x35	/* R   Y-Axis Data 1 */
#define DATAZ0		0x36	/* R   Z-Axis Data 0 */
#define DATAZ1		0x37	/* R   Z-Axis Data 1 */
#define FIFO_CTL	0x38	/* R/W FIFO control */

/* DEVIDs */
#define ID_ADXL345	0xE5

/* INT_ENABLE/INT_MAP/INT_SOURCE Bits */
#define SINGLE_TAP	(1 << 6)

/* TAP_AXES Bits */
#define TAP_X_EN	(1 << 2)
#define TAP_Y_EN	(1 << 1)
#define TAP_Z_EN	(1 << 0)

/* BW_RATE Bits */
#define LOW_POWER	(1 << 4)
#define RATE(x)		((x) & 0xF)

/* POWER_CTL Bits */
#define PCTL_MEASURE	(1 << 3)
#define PCTL_STANDBY	0X00

/* DATA_FORMAT Bits */
#define FULL_RES	(1 << 3)

/* 4 wire mode */
#define FOUR_WIRE	(1 << 6)

/* FIFO_CTL Bits */
#define FIFO_MODE(x)	(((x) & 0x3) << 6)
#define FIFO_BYPASS	0
#define FIFO_FIFO	1
#define FIFO_STREAM	2
#define SAMPLES(x)	((x) & 0x1F)

/* FIFO_STATUS Bits */
#define ADXL_X_AXIS			0
#define ADXL_Y_AXIS			1
#define ADXL_Z_AXIS			2

#define ADXL_TAP_X_EN	(1 << 2)
#define ADXL_TAP_Y_EN	(1 << 1)
#define ADXL_TAP_Z_EN	(1 << 0)

#define ADXL_FULL_RES		(1 << 3)
#define ADXL_RANGE_PM_2g	0
#define ADXL_RANGE_PM_4g	1
#define ADXL_RANGE_PM_8g	2
#define ADXL_RANGE_PM_16g	3

struct accel_bus_ops {
	int (*read)(unsigned char);
	void (*read_block)(unsigned char, int, void *);
	void (*write)(unsigned char, unsigned char);
};

//operations
static int accel_spi_read(unsigned char reg)
{
	u8 rev;

  SPI2_NSS_Enable();
  SPI2_SendByte(ADXL345_READCMD(reg));
	rev = SPI2_SendByte(0x00);
  SPI2_NSS_Disable();
  
  return rev;
}

static void accel_spi_write(unsigned char reg, unsigned char val)
{
	SPI2_NSS_Enable();
  SPI2_SendByte(ADXL345_WRITECMD(reg));
  SPI2_SendByte(val);
  SPI2_NSS_Disable();
}

static void accel_spi_read_block(unsigned char reg, int count, void *buf)
{
  char *pbuf = buf;
  
  SPI2_NSS_Enable();
  
  //write one
  SPI2_SendByte(ADXL345_READMB_CMD(reg));
  
  //read multi
	for(int i = 0; i < count; i++)
    pbuf[i] = SPI2_SendByte(0x00);
  
  SPI2_NSS_Disable();
}

static struct accel_bus_ops accel_spi_bops = {
	.write = accel_spi_write,
	.read = accel_spi_read,
	.read_block = accel_spi_read_block,
};

/* Macros to do SPI operations */
#define AC_READ(reg)	(accel_spi_bops.read(reg))
#define AC_WRITE(reg, val)	{accel_spi_bops.write(reg, val);delay_ms(1);}while(0);

static inline s32 sign_extend32(u32 value, int index)
{
	u8 shift = 31 - index;
	return (s32)(value << shift) >> shift;
}

#define le16_to_cpu(x) x

void accel_get_triple(struct axis_triple *axis)
{
	u16 buf[3];

	accel_spi_bops.read_block(DATAX0, DATAZ1 - DATAX0 + 1, buf);

	axis->x = sign_extend32(le16_to_cpu(buf[0]), 12);
	axis->y = sign_extend32(le16_to_cpu(buf[1]), 12);
	axis->z = sign_extend32(le16_to_cpu(buf[2]), 12);
}

int accel_probe(void)
{
	u8 revid;

  u8 low_power_mode = 0;
	u8 tap_threshold = 90;
	u8 tap_duration = 3;
	u8 tap_axis_control = ADXL_TAP_X_EN | ADXL_TAP_Y_EN | ADXL_TAP_Z_EN;
	u8 data_rate = 8;
	u8 data_range = ADXL_FULL_RES;
	u8 fifo_mode = FIFO_BYPASS;
	u8 watermark = 0;
  u8 int_mask = 0x40;

	//test spi communication
	revid = AC_READ(DEVID);
	if(revid == 0xE5){
		printf("ADXL345 is found\r\n");
	}else{
		printf("Failed to probe revid: %02x\r\n", revid);
    //AC_WRITE(POWER_CTL, PCTL_STANDBY);
    return -1;
	}

	/* Initialize the ADXL345 registers */

	/* Set the tap threshold and duration */
	AC_WRITE(THRESH_TAP, tap_threshold);
	AC_WRITE(DUR, tap_duration);

	/* set the axis where the tap will be detected */
	AC_WRITE(TAP_AXES, tap_axis_control);

	/* set the data rate and the axis reading power
	 * mode, less or higher noise reducing power
	 */
	AC_WRITE(BW_RATE, RATE(data_rate) | (low_power_mode ? LOW_POWER : 0));

	/* 13-bit full resolution right justified */
	AC_WRITE(DATA_FORMAT, data_range);

	/* Set the FIFO mode, no FIFO by default */
	AC_WRITE(FIFO_CTL, FIFO_MODE(fifo_mode) | SAMPLES(watermark));

	/* Map all INTs to INT1 pin */
	AC_WRITE(INT_MAP, 0);

	/* Enables interrupts */
	AC_WRITE(INT_ENABLE, int_mask);

	/* Set RUN mode */
	AC_WRITE(POWER_CTL, PCTL_MEASURE);

  printf("ADXL345 init finished.\r\n");
  
	return 0;
}
