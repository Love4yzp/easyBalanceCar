//
// Created by YZP on 2021/7/21.
//

#include "control.h"
#include "mpu6050.h"
#include  "math.h"

// Kalman structure
typedef struct {
  double Q_angle;
  double Q_bias;
  double R_measure;
  double angle;
  double bias;
  double P[2][2];
} Kalman_t;

//region Kalman Instance
Kalman_t KalmanX = {
    .Q_angle = 0.001f,
    .Q_bias = 0.003f,
    .R_measure = 0.03f
};

Kalman_t KalmanY = {
    .Q_angle = 0.001f,
    .Q_bias = 0.003f,
    .R_measure = 0.03f,
};
//endregion

uint32_t timer;

const double Accel_Z_corrector = 14418.0;
#define RAD_TO_DEG 57.295779513082320876798154814105
#define Banlance_Position -2.8; // 此处为 中值；

void MPU6050_ToHuman(TM_MPU6050_t *DataStruct);
double Kalman_getAngle(Kalman_t *Kalman, double newAngle, double newRate, double dt);

//region For Controlling Car speed
int Balance(float Angle, float Gyro);//roll(横滚角)（回复力）、角速度（阻尼力）
int Velocity(int encoder_left, int encoder_right);
void PWM_Limit(int *LeftPWM, int *RightPWM);
//endregion

extern TM_MPU6050_t MPU6050;
extern TM_MPU6050_Interrupt_t MPU6050_Interrupts;


int Encoder_Left = 0, Encoder_Right = 0;  // From Time Encoder.
int Balance_Pwm = 0, Velocity_Pwm = 0;           //直立环-速度环Pwm  For Calculate

//左右轮电机PWM变量
int speedL = 0; // PWML
int speedR = 0; //  PWMR



/**
 * @brief This is For MPU6050 INT Pin
 * @attention Pin is PA12
 * @param GPIO_Pin
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
  if (PAin(12) == 0) {
    //region Get the Speed of Car
    Encoder_Left = (int16_t) __HAL_TIM_GET_COUNTER(&htim4);
    Encoder_Right = (int16_t) __HAL_TIM_GET_COUNTER(&htim2); //读取脉冲数 ==>返回脉冲数
    __HAL_TIM_SetCounter(&htim4, 0);
    __HAL_TIM_SetCounter(&htim2, 0);
    //endregion
    //region Get MPU6050 Data(Angle), and To Human Read
    TM_MPU6050_ReadInterrupts(&MPU6050, &MPU6050_Interrupts);
    TM_MPU6050_ReadAll(&MPU6050);
    MPU6050_ToHuman(&MPU6050);
    //endregion
    //region calculate the PWM value
    Balance_Pwm = Balance((float)MPU6050.KalmanAngleY, (float)MPU6050.Gy);
    Velocity_Pwm = Velocity(Encoder_Left, Encoder_Right);
    speedL = -Balance_Pwm - Velocity_Pwm;
    speedR = -Balance_Pwm - Velocity_Pwm;
    //endregion
    //region Set PWM to Drive Car
    PWM_Limit(&speedL, &speedR);
    Set_Pwm(speedL, speedR);
    //endregion
  }
}

/**************************************************************************
 * @brief 赋值给PWM寄存器
 * @param motorLeft,motorRight  左轮速度  右轮速度
 * @retval None
**************************************************************************/
void Set_Pwm(int motorLeft, int motorRight) { //0~100
  if (motorLeft < 0) BIN1 = 0, BIN2 = 1;
  else
    BIN1 = 1, BIN2 = 0;
  PWMB = myabs(motorRight);
  if (motorRight < 0) AIN2 = 0, AIN1 = 1;
  else
    AIN2 = 1, AIN1 = 0;
  PWMA = myabs(motorLeft);
}

/**
 * @brief 对输出的PWM再次调整，
 * @param LeftPWM
 * @param RightPWM
 */
void PWM_Limit(int *LeftPWM, int *RightPWM) {
  int Amplitude = 900; //===PWM满幅ZZZ,限制在XXX
  if (*LeftPWM < -Amplitude)
    *LeftPWM = -Amplitude;
  if (*LeftPWM > Amplitude)
    *LeftPWM = Amplitude;
  if (*RightPWM < -Amplitude)
    *RightPWM = -Amplitude;
  if (*RightPWM > Amplitude)
    *RightPWM = Amplitude;
}
/**
 * @brief 绝对值函数 | 自用
 * @param arg
 * @return abs(arg)
 */
int myabs(int arg) {
  return arg > 0 ? arg : -arg;
}

/**
 * 将MPU6050的数据卡尔曼滤波后以人的阅读格式。
 * @param DataStruct
 */
void MPU6050_ToHuman(TM_MPU6050_t *DataStruct) {
  DataStruct->Ax = DataStruct->Accelerometer_X / 16384.0;
  DataStruct->Ay = DataStruct->Accelerometer_Y / 16384.0;
  DataStruct->Az = DataStruct->Accelerometer_Z / Accel_Z_corrector;
//  DataStruct->Temperature = (float) ((int16_t)DataStruct->Temperature / (float) 340.0 + (float) 36.53);
  DataStruct->Gx = DataStruct->Gyroscope_X / 131.0;
  DataStruct->Gy = DataStruct->Gyroscope_Y / 131.0;
  DataStruct->Gz = DataStruct->Gyroscope_Z / 131.0;

  double dt = (double) (HAL_GetTick() - timer) / 1000;
  timer = HAL_GetTick();
  double roll;
  double roll_sqrt = sqrt(
      DataStruct->Accelerometer_X * DataStruct->Accelerometer_X +
      DataStruct->Accelerometer_Z * DataStruct->Accelerometer_Z);
  if (roll_sqrt != 0.0) {
    roll = atan(DataStruct->Accelerometer_Y / roll_sqrt) * RAD_TO_DEG;
  } else {
    roll = 0.0;
  }
  double pitch = atan2(-DataStruct->Accelerometer_X, DataStruct->Accelerometer_Z) * RAD_TO_DEG;
  if ((pitch < -90 && DataStruct->KalmanAngleY > 90) || (pitch > 90 && DataStruct->KalmanAngleY < -90)) {
    KalmanY.angle = pitch;
    DataStruct->KalmanAngleY = pitch;
  } else {
    DataStruct->KalmanAngleY = Kalman_getAngle(&KalmanY, pitch, DataStruct->Gy, dt);
  }
  if (fabs(DataStruct->KalmanAngleY) > 90)
    DataStruct->Gx = -DataStruct->Gx;
  DataStruct->KalmanAngleX = Kalman_getAngle(&KalmanX, roll, DataStruct->Gy, dt);
}

double Kalman_getAngle(Kalman_t *Kalman, double newAngle, double newRate, double dt) {
  double rate = newRate - Kalman->bias;
  Kalman->angle += dt * rate;

  Kalman->P[0][0] += dt * (dt * Kalman->P[1][1] - Kalman->P[0][1] - Kalman->P[1][0] + Kalman->Q_angle);
  Kalman->P[0][1] -= dt * Kalman->P[1][1];
  Kalman->P[1][0] -= dt * Kalman->P[1][1];
  Kalman->P[1][1] += Kalman->Q_bias * dt;

  double S = Kalman->P[0][0] + Kalman->R_measure;
  double K[2];
  K[0] = Kalman->P[0][0] / S;
  K[1] = Kalman->P[1][0] / S;

  double y = newAngle - Kalman->angle;
  Kalman->angle += K[0] * y;
  Kalman->bias += K[1] * y;

  double P00_temp = Kalman->P[0][0];
  double P01_temp = Kalman->P[0][1];

  Kalman->P[0][0] -= K[0] * P00_temp;
  Kalman->P[0][1] -= K[0] * P01_temp;
  Kalman->P[1][0] -= K[1] * P00_temp;
  Kalman->P[1][1] -= K[1] * P01_temp;

  return Kalman->angle;
};

/**
 * @attention [Kp] decided by your PWM Setting and Bias
 * @param Angle
 * @param Gyro
 * @return Balance_Pwm
 */
int Balance(float Angle, float Gyro) {
  float Bias;
  int balance;
//  double Balance_Kp = -40, Balance_Kd = -8;//   kp(0 - 25) kd( 0 - 2 )
	float Balance_Kp=0,Balance_Kd=0;           //===调试用 |效果因为反应迅速

  Bias = Angle - Banlance_Position;                       //===求出平衡的角度中值 和机械相关
  balance = (int)(Balance_Kp * Bias + Gyro * Balance_Kd);   //===计算平衡控制的电机PWM  PD控制   kp是P系数 kd是D系数
  return balance;
}

/**
 * @brief 速度积分 | 为达到速度目标(0)，积分当前速度
 * @param encoder_left
 * @param encoder_right
 * @return  Velocity_PWM
 */
int Velocity(int encoder_left, int encoder_right) {
  static float Encoder_Least = 0, Encoder = 0;
  static float Encoder_Integral = 0;
  static int Velocity = 0;

  double Velocity_Kp = 28.4, Velocity_Ki = Velocity_Kp / 200.0;//-185 -0.925
//  float Velocity_Kp=0,   Velocity_Ki=Velocity_Kp/200.0;       //当调试其它参数时时，用此行         //===调试用

  //=============速度PI控制器=======================//
  Encoder_Least = (float)(encoder_left + encoder_right) - 0;                    //===获取最新速度偏差==测量速度（左右编码器之和）-目标速度（此处为零）
  Encoder *= (float)0.8;                                                    //===一阶低通滤波器
  Encoder += Encoder_Least * 0.2;                                      //===一阶低通滤波器
  Encoder_Integral += Encoder;                                         //===积分出位移 积分时间：5ms
  if (Encoder_Integral > 100) Encoder_Integral = 100;                 //===积分限幅决定速度的最大上限
  if (Encoder_Integral < -100) Encoder_Integral = -100;                //===积分限幅
  Velocity = (int)(Encoder * Velocity_Kp + Encoder_Integral * Velocity_Ki);        //===速度控制
  return Velocity;
}
