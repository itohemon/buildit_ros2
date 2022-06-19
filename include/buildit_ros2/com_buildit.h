#include <unistd.h>
#include <string>

int initializeComBuildIt(int *fd, std::string port);
void closeComBuildIt(int fd);
int recvDataBuidIt(int fd, uint8_t recv_buf[]);
int sendCmdBuildIt(int fd, uint8_t cmd_data[], int length);
int sendClearFaultCmd(int fd, uint8_t devid);
int sendFault(int fd, uint8_t devid, uint16_t type);
int sendFreeCmd(int fd, uint8_t devid);
int sendGetLogCmd(int fd, uint8_t devid, uint16_t startIdx, uint16_t readSize);
int sendGetLogInfoCmd(int fd, uint8_t devid);
int sendGetRefCurrentCmd(int fd, uint8_t devid);
int sendGetRefPositionCmd(int fd, uint8_t devid);
int sendGetRefVelocityCmd(int fd, uint8_t devid);
int sendHoldCmd(int fd, uint8_t devid);
int sendProtectionStopCmd(int fd, uint8_t devid, uint16_t timeout);
int sendQueryServoStatusCmd(int fd, uint8_t devid);
int sendReadyCmd(int fd, uint8_t devid);
int sendResetRotationCmd(int fd, uint8_t devid, int16_t rot);
int sendSetParamCmd1B(int fd, uint8_t devid, uint8_t paramid, uint8_t param);
int sendSetParamCmd2B(int fd, uint8_t devid, uint8_t paramid, uint16_t param);
int sendSetParamCmd4B(int fd, uint8_t devid, uint8_t paramid, uint32_t param);
int sendSetRefCurrentCmd(int fd, uint8_t devid, int16_t cur);
int sendSetRefPositionCmd(int fd, uint8_t devid, int32_t pos);
int sendSetRefVelocityCmd(int fd, uint8_t devid, int16_t vel);

