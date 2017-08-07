
typedef struct
{
	uint16_t RPMcounter; // in amount of timer ticks
	uint16_t RPMvalue; // in RPM
	// rpm ticks multiplier or something
	uint16_t VSScounter; // in amount of timer ticks
	uint16_t VSSvalue; // in km/h
	// vss ticks multiplier or something
	uint16_t batteryValue; // in milliVolt

} carData_datastruct;
