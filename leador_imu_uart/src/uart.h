#ifndef __UART_H__
#define	__UART_H__

int uart_init(char *deviceName, int baudRate, int dataBits, int parity, int stopBits);

#endif

