/*
 * DHT.h
 *
 */

#ifndef DHT_H_
#define DHT_H_

typedef struct
{
	float Temperature;
	float Humidity;
}DHT_DataTypedef;


void DHT_GetData (DHT_DataTypedef *DHT_Data);
void delay_us(uint16_t us);

#endif /* INC_DHT_H_ */
