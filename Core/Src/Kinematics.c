/*
 * Kinematics.c
 *
 *  Created on: Oct 26, 2024
 *      Author: vutie
 */
#include <Kinematics.h>
InputFK_ InputFK;
ForwardKinematics_ FK;
InverseKinematics_ IK;
InputIK_ InputIK;

void calculate_FK(ForwardKinematics_ *FK, Setpoint_ *Setpoint, float theta1Value, float theta2Value, float theta3Value, float theta4Value) {

	FK->theta1_FK = theta1Value;
	FK->theta1_FK_rad = (FK->theta1_FK * M_PI) / 180.0;
	Setpoint->setpoint1 = theta1Value;

	FK->theta2_FK = theta2Value;
	FK->theta2_FK_rad = (FK->theta2_FK * M_PI) / 180.0;
	Setpoint->setpoint2 = theta2Value;

	FK->theta3_FK = theta3Value;
	FK->theta3_FK_rad = (FK->theta3_FK * M_PI) / 180.0;
	Setpoint->setpoint3 = theta3Value;

	FK->theta4_FK = theta4Value;
	FK->theta4_FK_rad = (FK->theta4_FK * M_PI) / 180.0;
	Setpoint->setpoint4 = theta4Value;

	FK->psi_FK = FK->theta2_FK + FK->theta3_FK + FK->theta4_FK;
	FK->psi_FK_rad = FK->theta2_FK_rad + FK->theta3_FK_rad + FK->theta4_FK_rad;

	// Tính toán giá trị Px, Py, Pz
	FK->Px_FK = cos(FK->theta1_FK_rad) * (L1 + L2 * cos(FK->theta2_FK_rad) + L3 * cos(FK->theta2_FK_rad + FK->theta3_FK_rad) + L4 * cos(FK->psi_FK_rad));

	FK->Py_FK = sin(FK->theta1_FK_rad) * (L1 + L2 * cos(FK->theta2_FK_rad) + L3 * cos(FK->theta2_FK_rad + FK->theta3_FK_rad) + L4 * cos(FK->psi_FK_rad));

	FK->Pz_FK = d1 + L3 * sin(FK->theta2_FK_rad + FK->theta3_FK_rad) + L2 * sin(FK->theta2_FK_rad) + L4 * sin(FK->psi_FK_rad);

}
float round_nearest(float value) {
    return (value >= 0) ? floor(value + 0.5) : ceil(value - 0.5);
}

void calculate_IK_BN1(InverseKinematics_ *IK, Setpoint_ *Setpoint,float Px_value, float Py_value, float Pz_value, float Theta_value){

    IK->Px_IK = Px_value;
    IK->Py_IK = Py_value;
    IK->Pz_IK = Pz_value;
    IK->Theta_IK = Theta_value;

    IK->t_rad = IK->Theta_IK * (M_PI / 180);
    IK->k = sqrt(pow(IK->Px_IK, 2) + pow(IK->Py_IK, 2));
    IK->theta1_IK_rad = atan2((IK->Py_IK / IK->k), (IK->Px_IK / IK->k));
    IK->Theta1_IK = IK->theta1_IK_rad * (180 / M_PI);

    if (IK->Theta1_IK < -180) {
    	IK->Theta1_IK += 360;
    } else if (IK->Theta1_IK > 180) {
    	IK->Theta1_IK -= 360;
    }
    IK->Theta1_IK = round_nearest(IK->Theta1_IK);
    Setpoint->setpoint1 = IK->Theta1_IK;

    IK->E = IK->Px_IK * cos(IK->theta1_IK_rad) + IK->Py_IK * sin(IK->theta1_IK_rad) - L1 - L4 * cos(IK->t_rad);
    IK->F = IK->Pz_IK - d1 - L4 * sin(IK->t_rad);
    IK->a = -2 * L2 * IK->F;
    IK->b = -2 * L2 * IK->E;
    IK->d = pow(L3, 2) - pow(IK->E, 2) - pow(IK->F, 2) - pow(L2, 2);
    IK->f = sqrt(pow(IK->a, 2) + pow(IK->b, 2));
    IK->alpha = atan2(-2 * L2 * IK->F / IK->f, -2 * L2 * IK->E / IK->f);

    IK->var_temp = pow(IK->d, 2) / pow(IK->f, 2);
    if (IK->var_temp > 1) IK->var_temp = 1;

    IK->theta2_IK_rad = atan2(sqrt(1 - IK->var_temp), IK->d / IK->f) + IK->alpha;
    IK->Theta2_IK = IK->theta2_IK_rad * (180 / M_PI);

    if (IK->Theta2_IK < -180) {
    	IK->Theta2_IK += 360;
    } else if (IK->Theta2_IK > 180) {
    	IK->Theta2_IK -= 360;
    }
    IK->Theta2_IK = round_nearest(IK->Theta2_IK);
    Setpoint->setpoint2 = IK->Theta2_IK;

    IK->c23 = (IK->Px_IK * cos(IK->theta1_IK_rad) + IK->Py_IK * sin(IK->theta1_IK_rad) - L1 - L2 * cos(IK->theta2_IK_rad) - L4 * cos(IK->t_rad)) / L3;
    IK->s23 = (IK->Pz_IK - d1 - L2 * sin(IK->theta2_IK_rad) - L4 * sin(IK->t_rad)) / L3;
    IK->theta3_IK_rad = atan2(IK->s23, IK->c23) - IK->theta2_IK_rad;
    IK->Theta3_IK = IK->theta3_IK_rad * (180 / M_PI);

    if (IK->Theta3_IK < -180) {
    	IK->Theta3_IK += 360;
    } else if (IK->Theta3_IK > 180) {
    	IK->Theta3_IK -= 360;
    }
    IK->Theta3_IK = round_nearest(IK->Theta3_IK);
    Setpoint->setpoint3 = IK->Theta3_IK;

    IK->theta4_IK_rad = IK->t_rad - IK->theta2_IK_rad - IK->theta3_IK_rad;
    IK->Theta4_IK = IK->theta4_IK_rad * (180 / M_PI);
    IK->Theta4_IK = round_nearest(IK->Theta4_IK);
    Setpoint->setpoint4 = IK->Theta4_IK;
}

void calculate_IK_BN2(InverseKinematics_ *IK, Setpoint_ *Setpoint,float px_value, float py_value, float pz_value, float Theta_value) {

	IK->Px_IK = px_value;
	IK->Py_IK = py_value;
	IK->Pz_IK = pz_value;
	IK->Theta_IK = Theta_value;

	IK->t_rad = IK->Theta_IK * (M_PI / 180);
	IK->k = sqrt(pow(IK->Px_IK, 2) + pow(IK->Py_IK, 2));
	IK->theta1_IK_rad = atan2((IK->Py_IK / IK->k), (IK->Px_IK / IK->k));
	IK->Theta1_IK = IK->theta1_IK_rad * (180 / M_PI);

    if (IK->Theta1_IK < -180) {
    	IK->Theta1_IK += 360;
    } else if (IK->Theta1_IK > 180) {
    	IK->Theta1_IK -= 360;
    }
    IK->Theta1_IK = round_nearest(IK->Theta1_IK);
    Setpoint->setpoint1 = IK->Theta1_IK;

    IK->E = IK->Px_IK * cos(IK->theta1_IK_rad) + IK->Py_IK * sin(IK->theta1_IK_rad) - L1 - L4 * cos(IK->t_rad);
    IK->F = IK->Pz_IK - d1 - L4 * sin(IK->t_rad);

    IK->a = -2 * L2 * IK->F;
    IK->b = -2 * L2 * IK->E;
    IK->d = pow(L3, 2) - pow(IK->E, 2) - pow(IK->F, 2) - pow(L2, 2);
    IK->f = sqrt(pow(IK->a, 2) + pow(IK->b, 2));
    IK->alpha = atan2(-2 * L2 * IK->F / IK->f, -2 * L2 * IK->E / IK->f);

    IK->var_temp = pow(IK->d, 2) / pow(IK->f, 2);
    if (IK->var_temp > 1) IK->var_temp = 1;

    IK->theta2_IK_rad = atan2(-sqrt(1 - IK->var_temp), IK->d / IK->f) + IK->alpha;
    IK->Theta2_IK = IK->theta2_IK_rad * (180 / M_PI);

    if (IK->Theta2_IK < -180) {
    	IK->Theta2_IK += 360;
    } else if (IK->Theta2_IK > 180) {
    	IK->Theta2_IK -= 360;
    }
    IK->Theta2_IK = round_nearest(IK->Theta2_IK);
    Setpoint->setpoint2 = IK->Theta2_IK;

    IK->c23 = (IK->Px_IK * cos(IK->theta1_IK_rad) + IK->Py_IK * sin(IK->theta1_IK_rad) - L1 - L2 * cos(IK->theta2_IK_rad) - L4 * cos(IK->t_rad)) / L3;
    IK->s23 = (IK->Pz_IK - d1 - L2 * sin(IK->theta2_IK_rad) - L4 * sin(IK->t_rad)) / L3;
    IK->theta3_IK_rad = atan2(IK->s23, IK->c23) - IK->theta2_IK_rad;
    IK->Theta3_IK = IK->theta3_IK_rad * (180 / M_PI);

    if (IK->Theta3_IK < -180) {
    	IK->Theta3_IK += 360;
    } else if (IK->Theta3_IK > 180) {
    	IK->Theta3_IK -= 360;
    }
    IK->Theta3_IK = round_nearest(IK->Theta3_IK);
    Setpoint->setpoint3 = IK->Theta3_IK;

    IK->theta4_IK_rad = IK->t_rad - IK->theta2_IK_rad - IK->theta3_IK_rad;
    IK->Theta4_IK = IK->theta4_IK_rad * (180 / M_PI);
    IK->Theta4_IK = round_nearest(IK->Theta4_IK);
    Setpoint->setpoint4 = IK->Theta4_IK;
}
