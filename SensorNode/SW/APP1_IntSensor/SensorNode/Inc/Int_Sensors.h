
#ifndef INT_SENSORS_H_
#define INT_SENSORS_H_

typedef struct {
	/* °C */
	float temp;
	/* °C */
	float temp_hum;
	/* % */
	float hum;
	/* Pascal - target 101325 Pa */
	float pres;
	/* x - y - z mG */
	float magnetic_mG[3];
	/* x - y - z mG*/
	float acceleration[3];
	/* x - y -z mdps*/
	float angular[3];
	/* ADC0 V*/
	float adc0;
	/* ADC1 V*/
	float adc1;
	/* ADC2 V*/
	float adc2;
	/* ADC3 V*/
	float adc3;
}Sensors_data_t;

void IntSensors_Task (void const * argument);
void IIS2MDC_Callback( void );
void lsm6dsox_Callback( void );
void HDC2080_Callback( void );
void TMP117_Callback( void );
uint16_t Int_Sensors_Stream (uint8_t *out, uint16_t size);

#endif /* INT_SENSORS_H_ */
