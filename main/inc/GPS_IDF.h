
#include "driver/uart.h"

void init_uart_config_GPS(uart_port_t portNum, int BufSize, int GpsTXpin, int GpsRXpin);
void parse_gprmc(const char *sentence, const char *tag);
