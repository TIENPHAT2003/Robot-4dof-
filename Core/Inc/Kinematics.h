/*
 * Kinematics.h
 *
 *  Created on: Oct 26, 2024
 *      Author: vutie
 */

#ifndef INC_KINEMATICS_H_
#define INC_KINEMATICS_H_
#include <math.h>
#include <stdint.h>
#define L1 		91
#define L2		122
#define L3		77
#define L4		78
#define d1		(62 + 176)
typedef struct{
	float inputTheta1;
	float inputTheta2;
	float inputTheta3;
	float inputTheta4;
}InputFK_;

typedef struct{
	float theta1_FK;
	float theta2_FK;
	float theta3_FK;
	float theta4_FK;
	float theta_Fk;
	float psi_FK;

	float theta1_FK_rad;
	float theta2_FK_rad;
	float theta3_FK_rad;
	float theta4_FK_rad;
	float theta_FK_rad;
	float psi_FK_rad;

	float Px_FK;
	float Py_FK;
	float Pz_FK;

}ForwardKinematics_;

int16_t t1, t2, t3, t4;

typedef struct{
	float Px_IK;
	float Py_IK;
	float Pz_IK;

	float Theta1_IK;
	float Theta2_IK;
	float Theta3_IK;
	float Theta4_IK;
	float Theta_IK;

	float theta1_IK_rad;
	float theta2_IK_rad;
	float theta3_IK_rad;
	float theta4_IK_rad;

	float alpha;
	float k;
	float E;
	float F;
	float a;
	float b;
	float d;
	float f;
	float var_temp;
	float c23;
	float s23;
	float t_rad;

}InverseKinematics_;


typedef struct{
	float inputPx;
	float inputPy;
	float inputPz;
	float inputTheta;
}InputIK_;
//--------------TRAJECTORY PLANNING----------------//
typedef struct{
	float setpoint1;
	float setpoint2;
	float setpoint3;
	float setpoint4;

	float preSetpoint1;
	float preSetpoint2;
	float preSetpoint3;
	float preSetpoint4;

	float p0_1;
	float p0_2;
	float p0_3;
	float p0_4;

	float theta1_Nha;
	float theta2_Nha;
	float theta3_Nha;
	float theta4_Nha;
}Setpoint_;
void calculate_FK(ForwardKinematics_ *FK, Setpoint_ *Setpoint, float theta1Value, float theta2Value, float theta3Value, float theta4Value);
float round_nearest(float value);
void calculate_IK_BN1(InverseKinematics_ *IK, Setpoint_ *Setpoint,float Px_value, float Py_value, float Pz_value, float Theta_value);
void calculate_IK_BN2(InverseKinematics_ *IK, Setpoint_ *Setpoint,float px_value, float py_value, float pz_value, float Theta_value) ;

#endif /* INC_KINEMATICS_H_ */
