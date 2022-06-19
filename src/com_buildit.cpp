#include <unistd.h>

#include "buildit_ros2/uart.h"

enum
{
  QUERY_SERVO_STATUS_CMD = 0x01, // サーボ関連のセンサ値を取得
  GET_LOG_INFO_CMD = 0x05,       // ログ情報を取得
  GET_LOG_CMD = 0x06,            // ログ情報を取得
  READY_CMD = 0x10,              // STATE_READY 状態へ遷移
  FREE_CMD = 0x11,               // STATE_FREE 状態へ遷移
  HOLD_CMD = 0x12,               // STATE_HOLD 状態へ遷移
  CLEAR_FAULT_CMD = 0x13,        // フォルト情報クリア
  PROTECTION_STOP_CMD = 0x14,    // 保護停止
  SET_REF_CURRENT_CMD = 0x20,    // 電流指令値設定
  GET_REF_CURRENT_CMD = 0x21,    // 電流指令値取得
  SET_REF_VELOCITY_CMD = 0x22,   // 速度指令値設定
  GET_REF_VELOCITY_CMD = 0x23,   // 速度指令値取得
  SET_REF_POSITION_CMD = 0x24,   // 位置指令値設定
  GET_REF_POSITION_CMD = 0x25,   // 位置指令値取得
  SET_PARAM_CMD = 0x30,          // パラメータ設定
  GET_PARAM_CMD = 0x31,          // パラメータ取得
  RESET_ROTATION_CMD = 0x32,     // 回転オフセットの初期化
  FAULT = 0x3d,
  NACK = 0xff,                   // 失敗応答
};

int initializeComBuildIt(int *fd, std::string port)
{
  if (initializeUART(fd, port) < 0)
  {
    fprintf(stderr, "Can't initizalize UART to communicate with BuildIt\n");
    return -1;
  }

  return 1;
}

void closeComBuildIt(int fd)
{
  closeUART(fd);
  close(fd);
}

int recvDataBuidIt(int fd, uint8_t recv_buf[])
{
  return recvDataUART(fd, recv_buf);
}

int sendCmdBuildIt(int fd, uint8_t cmd_data[], int length)
{
  int i;
  const int num_data = length + 4;
  uint8_t data[num_data];
  int checksum = 0;

  // fprintf(stdout, "sendCmdBuildIt, fd: %d, cmd: ", fd);

  data[0] = HEADER1;
  data[1] = HEADER2;
  data[2] = HEADER3;
  data[3] = CRCCalculate(cmd_data, length);

  for (i = 0; i < length; i++) {
    data[i + 4] = cmd_data[i];
    // fprintf(stdout, "0x%x ", cmd_data[i]);
  }

  if (sendCmdUART(fd, data, num_data) < 0) {
    fprintf(stderr, "Failed to send command\n");
    return -1;
  }
  
  return 1;
}

int sendClearFaultCmd(int fd, uint8_t devid)
{
  int num_cmd = 4;
  uint8_t cmd[num_cmd];

  cmd[0] = devid;
  cmd[1] = CLEAR_FAULT_CMD;
  cmd[2] = 0x00;
  cmd[3] = 0x00;

  return sendCmdBuildIt(fd, cmd, num_cmd);
}

// 0: コマンドトリガフォルト発生。STATE_FAULT_HOLD状態に遷移させる
// 1: システムフォルト発生（再起動が必要）
int sendFault(int fd, uint8_t devid, uint16_t type)
{
  int num_cmd = 6;
  uint8_t cmd[num_cmd];

  cmd[0] = devid;
  cmd[1] = FAULT;
  cmd[2] = 0x02;
  cmd[3] = 0x00;
  cmd[4] = type & 0xFF;
  cmd[5] = type >> 8;

  return sendCmdBuildIt(fd, cmd, num_cmd);
}

int sendFreeCmd(int fd, uint8_t devid)
{
  int num_cmd = 4;
  uint8_t cmd[num_cmd];

  cmd[0] = devid;
  cmd[1] = FREE_CMD;
  cmd[2] = 0x00;
  cmd[3] = 0x00;

  return sendCmdBuildIt(fd, cmd, num_cmd);
}

int sendGetLogCmd(int fd, uint8_t devid, uint16_t startIdx, uint16_t readSize)
{
  int num_cmd = 6;
  uint8_t cmd[num_cmd];

  cmd[0] = devid;
  cmd[1] = GET_LOG_CMD;
  cmd[2] = 0x04;
  cmd[3] = 0x00;
  cmd[4] = startIdx & 0xFF;
  cmd[5] = startIdx >> 8;
  cmd[6] = readSize & 0xFF;
  cmd[7] = readSize >> 8;

  return sendCmdBuildIt(fd, cmd, num_cmd);}

int sendGetLogInfoCmd(int fd, uint8_t devid)
{
  int num_cmd = 4;
  uint8_t cmd[num_cmd];

  cmd[0] = devid;
  cmd[1] = GET_LOG_INFO_CMD;
  cmd[2] = 0x00;
  cmd[3] = 0x00;

  return sendCmdBuildIt(fd, cmd, num_cmd);
}

int sendGetRefCurrentCmd(int fd, uint8_t devid)
{
  int num_cmd = 4;
  uint8_t cmd[num_cmd];

  cmd[0] = devid;
  cmd[1] = GET_REF_CURRENT_CMD;
  cmd[2] = 0x00;
  cmd[3] = 0x00;

  return sendCmdBuildIt(fd, cmd, num_cmd);
}

int sendGetRefPositionCmd(int fd, uint8_t devid)
{
  int num_cmd = 4;
  uint8_t cmd[num_cmd];

  cmd[0] = devid;
  cmd[1] = GET_REF_POSITION_CMD;
  cmd[2] = 0x00;
  cmd[3] = 0x00;

  return sendCmdBuildIt(fd, cmd, num_cmd);
}

int sendGetRefVelocityCmd(int fd, uint8_t devid)
{
  int num_cmd = 4;
  uint8_t cmd[num_cmd];

  cmd[0] = devid;
  cmd[1] = GET_REF_VELOCITY_CMD;
  cmd[2] = 0x00;
  cmd[3] = 0x00;

  return sendCmdBuildIt(fd, cmd, num_cmd);
}

int sendHoldCmd(int fd, uint8_t devid)
{
  int num_cmd = 4;
  uint8_t cmd[num_cmd];

  cmd[0] = devid;
  cmd[1] = HOLD_CMD;
  cmd[2] = 0x00;
  cmd[3] = 0x00;

  return sendCmdBuildIt(fd, cmd, num_cmd);
}

int sendProtectionStopCmd(int fd, uint8_t devid, uint16_t timeout)
{
  int num_cmd = 6;
  uint8_t cmd[num_cmd];

  cmd[0] = devid;
  cmd[1] = PROTECTION_STOP_CMD;
  cmd[2] = 0x02;
  cmd[3] = 0x00;
  cmd[4] = timeout & 0xFF;
  cmd[5] = timeout >> 8;

  return sendCmdBuildIt(fd, cmd, num_cmd);
}

int sendQueryServoStatusCmd(int fd, uint8_t devid)
{
  int num_cmd = 4;
  uint8_t cmd[num_cmd];

  cmd[0] = devid;
  cmd[1] = QUERY_SERVO_STATUS_CMD;
  cmd[2] = 0x00;
  cmd[3] = 0x00;

  return sendCmdBuildIt(fd, cmd, num_cmd);
}

int sendReadyCmd(int fd, uint8_t devid)
{
  int num_cmd = 4;
  uint8_t cmd[num_cmd];

  cmd[0] = devid;
  cmd[1] = READY_CMD;
  cmd[2] = 0x00;
  cmd[3] = 0x00;

  return sendCmdBuildIt(fd, cmd, num_cmd);
}

int sendResetRotationCmd(int fd, uint8_t devid, int16_t rot)
{
  int num_cmd = 6;
  uint8_t cmd[num_cmd];

  cmd[0] = devid;
  cmd[1] = RESET_ROTATION_CMD;
  cmd[2] = 0x02;
  cmd[3] = 0x00;
  cmd[4] = rot & 0xFF;
  cmd[5] = rot >> 8;

  return sendCmdBuildIt(fd, cmd, num_cmd);
}

int sendSetParamCmd1B(int fd, uint8_t devid, uint8_t paramid, uint8_t param)
{
  int num_cmd = 6;
  uint8_t cmd[num_cmd];

  cmd[0] = devid;
  cmd[1] = SET_PARAM_CMD;
  cmd[2] = 0x03;
  cmd[3] = 0x00;
  cmd[4] = paramid;
  cmd[5] = param;

  return sendCmdBuildIt(fd, cmd, num_cmd);
}

int sendSetParamCmd2B(int fd, uint8_t devid, uint8_t paramid, uint16_t param)
{
  int num_cmd = 7;
  uint8_t cmd[num_cmd];

  cmd[0] = devid;
  cmd[1] = SET_PARAM_CMD;
  cmd[2] = 0x03;
  cmd[3] = 0x00;
  cmd[4] = paramid;
  cmd[5] = param & 0xFF;
  cmd[6] = param >> 8;

  return sendCmdBuildIt(fd, cmd, num_cmd);
}

int sendSetParamCmd4B(int fd, uint8_t devid, uint8_t paramid, uint32_t param)
{
  int num_cmd = 9;
  uint8_t cmd[num_cmd];

  cmd[0] = devid;
  cmd[1] = SET_PARAM_CMD;
  cmd[2] = 0x03;
  cmd[3] = 0x00;
  cmd[4] = paramid;
  cmd[5] = param & 0xFF;
  cmd[6] = param >> 8;
  cmd[7] = param >> 16;
  cmd[8] = param >> 24;

  return sendCmdBuildIt(fd, cmd, num_cmd);
}

int sendSetRefCurrentCmd(int fd, uint8_t devid, int16_t cur)
{
  int num_cmd = 6;
  uint8_t cmd[num_cmd];

  cmd[0] = devid;
  cmd[1] = SET_REF_CURRENT_CMD;
  cmd[2] = 0x02;
  cmd[3] = 0x00;
  cmd[4] = cur & 0xFF;
  cmd[5] = cur >> 8;

  return sendCmdBuildIt(fd, cmd, num_cmd);
}

int sendSetRefPositionCmd(int fd, uint8_t devid, int32_t pos)
{
  int num_cmd = 8;
  uint8_t cmd[num_cmd];

  cmd[0] = devid;
  cmd[1] = SET_REF_POSITION_CMD;
  cmd[2] = 0x04;
  cmd[3] = 0x00;
  cmd[4] = pos & 0xFF;
  cmd[5] = pos >> 8;
  cmd[6] = pos >> 16;
  cmd[7] = pos >> 24;

  return sendCmdBuildIt(fd, cmd, num_cmd);}

int sendSetRefVelocityCmd(int fd, uint8_t devid, int16_t vel)
{
  int num_cmd = 6;
  uint8_t cmd[num_cmd];

  cmd[0] = devid;
  cmd[1] = SET_REF_VELOCITY_CMD;
  cmd[2] = 0x02;
  cmd[3] = 0x00;
  cmd[4] = vel & 0xFF;
  cmd[5] = vel >> 8;

  return sendCmdBuildIt(fd, cmd, num_cmd);
}
