#ifndef _ADXL345_H_
#define _ADXL345_H_

struct axis_triple {
	int x;
	int y;
	int z;
};

int accel_probe(void);
void accel_get_triple(struct axis_triple *axis);

#endif
