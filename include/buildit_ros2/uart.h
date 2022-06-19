#include <string> 
#include <cstdint>

#define HEADER1 (0xAB)
#define HEADER2 (0xCC)
#define HEADER3 (0xBA)

int sendCmdUART(int fd, uint8_t cmd[], int len);
int recvDataUART(int fd, uint8_t recv_buf[]);
int initializeUART(int *fd, std::string port);
void closeUART(int fd);
uint8_t CRCCalculate(const uint8_t *buf, uint32_t size);
