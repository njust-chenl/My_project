#include "robot_control.h"

#define ArmMotorPulsePerTurn 10000
#define ArmReductionRatio ((float)72)
#define motor_F 0x03
#define motor_B 0x04
float ratioAngleToPulse = ArmMotorPulsePerTurn * ArmReductionRatio / (2 * Pi);
my_serial sp1;
// uint8_t 一个字节   uint16_t  2个字节  uint32_t 4个字节

// ptr->motor.tarPos=ptr->tarAngle*ratioAngleToPulse;将角度转化为脉冲，注意角度必须是弧度制。
//相对运动指令：04 29 00 00 27 10 23 39
//绝对运动指令：04 28 00 00 27 10  23 38    //10000个
//开始运动指令：04 32 00 00 00 00 03 32
//使能电机 设置距离 开始运动//通过z轴旋转的角度来刻画算角度   这个地方需要该
//找限位开关指令 PRIM_SeekLimit  04 47 00 00 00 00 04 47
//编码器读数归零 PRIM_EncRst 04 50 00 00 00 00 04 50
//电机回零 PRIM_Homming 04 48 00 00 00 00 04 48
//获取驱动器状态位信息 PRIM_GetStatus 04 64 00 00 00 00 04 64 //处理返回的信息
//读取电机位置实际值 PRIM_GetActualPos 04 05 00 00 00 00 04 05
//获取q轴电流（包括设定值和实际值） PRIM_GetAxisQCurrent 04 40 00 00 00 00 04 40
//获取d轴电流（包括设定值和实际值） PRIM_GetAxisQCurrent 04 40 00 00 00 00 04 40
//找电机编码器z脉冲   PRIM_SeekIndex 04 46 00 00 00 00 04 46



/*

TODO :  摆臂电机初始化
输入 ：
返回   成功1 
    失败 0

备注

*/
bool motorArm_init(uint8_t addr) {
  sp1.write(setCmd(PRIM_SeekLimit, addr, 0), 8);
  if (statusGet() != 0)
    return true;

  // sp.write(setCmd(PRIM_EncRst,addr,0),8);//编码器读数归零
  // 技术人员说不需要编码器归零
}
/*
TODO :   写摆臂的运动，这个需要根据传入的角度值来编写,   最好采用相对运动模式

输入：

返回:

备注:
*/

void motorArm_ctrl(uint8_t angle, uint8_t addr, uint8_t mode) {
  angleSet(angle, addr, mode);
  if (statusGet(addr) != 0)
    sp1.write(setCmd(PRIM_Go, addr, 0), 8);
}



//角度的设定需要确定是相对运动还是绝对运动
void angleSet(int tarAngle, uint8_t id, uint8_t mode) {
  int32_t tarPos = tarAngle * ratioAngleToPulse;
  sp1.write(setCmd(mode, id, tarPos), 8);
}

// sp.write(setCmd(PRIM_Homming,MOTOR_F,0),8);

bool statusGet( uint8_t addr ) {
  uint8_t buffer[8], bit[8];
  size_t n;
  while(){
  sp1.write(setCmd(PRIM_GetStatus, addr, 0), 8);
  n = sp.available();
  sp1.read(buffer, n);
  if (formatcheck(buffer, PRIM_GetStatus, addr) != 0) {
    bit[0] =
        buffer[6] & 0x0f; //取出字节的低四位 周一 问问bit0的数值是怎么分配的
    // bit[1]=((buffer[6]&0xf0)>>4);     //取出字节的高四位
  } else
    return false;
  if (bit[0] 怎么样的时候退出循环)
    return true;
  }
}



bool formatcheck(uint8_t *cmd, uint8_t cmdtype, uint8_t addr) {
  if (cmd[0] != addr || cmd[1] != cmdtype)
    return FALSE;
  if (cmd[0] ^ cmd[2] ^ cmd[4] != cmd[6] || cmd[1] ^ cmd[3] ^ cmd[5] != cmd[7])
    return FALSE;
  return true;
}

// bool servoMotorCtrl(servoMotor* ptr,uint8_t mode){
// 	if(mode==0){//速度模式
// 		return writeParam(PRIM_SetVelocity,ptr->ID,ptr->tarV*10);
// 	}else{
// 		if(mode==1){//位置模式(相对运动)
// 			writeParam(PRIM_MoveRel,ptr->ID,ptr->tarPos);
// 		}else if(mode==2){//位置模式(绝对运动)
// 			writeParam(PRIM_MoveAbs,ptr->ID,ptr->tarPos);
// 		}
// 		return writeParam(PRIM_Go,ptr->ID,0);//开始运动
// 	}
// 	return FALSE;
// }