

#ifdef EKF_FOC

float Rs=0;
float Ls=0;
float flux=0;

float Q_0_0=0;
//float Q_0_1;
//float Q_0_2;
//float Q_0_3;
//float Q_1_0;
float Q_1_1=0;
//float Q_1_2;
//float Q_1_3;
//float Q_2_0;
//float Q_2_1;
float Q_2_2=0;
//float Q_2_3;
//float Q_3_0;
//float Q_3_1;
//float Q_3_2;
float Q_3_3=0;

float R_0_0=0;
//float R_0_1;
//float R_1_0;
float R_1_1=0;
float T=0;

float vs_ab_0_0=0;
float vs_ab_1_0=0;
float is_ab_0_0=0;
float is_ab_1_0=0;

float P0_0_0=0;
float P0_0_1=0;
float P0_0_2=0;
float P0_0_3=0;
float P0_1_0=0;
float P0_1_1=0;
float P0_1_2=0;
float P0_1_3=0;
float P0_2_0=0;
float P0_2_1=0;
float P0_2_2=0;
float P0_2_3=0;
float P0_3_0=0;
float P0_3_1=0;
float P0_3_2=0;
float P0_3_3=0;

float H_0_0=0;
//float H_0_1;
//float H_0_2;
//float H_0_3;
//float H_1_0;
float H_1_1=0;
//float H_1_2;
//float H_1_3;

float B_0_0=0;
//float B_0_1;
//float B_1_0;
float B_1_1=0;
//float B_2_0;
//float B_2_1;
//float B_3_0;
//float B_3_1;

float F_0_0=0;
//float F_0_1;
float F_0_2=0;
float F_0_3=0;
//float F_1_0;
float F_1_1=0;
float F_1_2=0;
float F_1_3=0;
//float F_2_0;
//float F_2_1;
//float F_2_2;
//float F_2_3;
//float F_3_0;
//float F_3_1;
float F_3_2=0;
//float F_3_3;

float temp_0_0=0;
float temp_0_1=0;
float temp_0_2=0;
float temp_0_3=0;
float temp_1_0=0;
float temp_1_1=0;
float temp_1_2=0;
float temp_1_3=0;
float temp_2_0=0;
float temp_2_1=0;
float temp_2_2=0;
float temp_2_3=0;
float temp_3_0=0;
float temp_3_1=0;
float temp_3_2=0;
float temp_3_3=0;

float f1_0_0=0;
float f1_1_0=0;
float f1_2_0=0;
float f1_3_0=0;

float f2_0_0=0;
//float f2_0_1;
float f2_0_2=0;
float f2_0_3=0;
//float f2_1_0;
float f2_1_1=0;
float f2_1_2=0;
float f2_1_3=0;
//float f2_2_0;
//float f2_2_1;
float f2_2_2=0;
//float f2_2_3;
//float f2_3_0;
//float f2_3_1;
float f2_3_2=0;
float f2_3_3=0;

float X_pred_0_0=0;
float X_pred_1_0=0;
float X_pred_2_0=0;
float X_pred_3_0=0;

float Y_pred_0_0=0;
float Y_pred_1_0=0;

float Y_0_0=0;
float Y_1_0=0;

float P_pred_0_0=0;
float P_pred_0_1=0;
float P_pred_0_2=0;
float P_pred_0_3=0;    
float P_pred_1_0=0;
float P_pred_1_1=0;
float P_pred_1_2=0;
float P_pred_1_3=0;  
float P_pred_2_0=0;
float P_pred_2_1=0;
float P_pred_2_2=0;
float P_pred_2_3=0;
float P_pred_3_0=0;
float P_pred_3_1=0;
float P_pred_3_2=0;
float P_pred_3_3=0;

float temp_0_0_t=0;
float temp_0_1_t=0;
float temp_1_0_t=0;
float temp_1_1_t=0;
float temp=0;

float K_0_0=0;
float K_0_1=0;
float K_1_0=0;
float K_1_1=0;   
float K_2_0=0;
float K_2_1=0;   
float K_3_0=0;
float K_3_1=0;

float tempa_0_0=0;
float tempa_1_0=0;
float tempa_2_0=0;
float tempa_3_0=0;


void stm32_ekf_Start_wrapper(void)
{
	Rs =(float)MCPara[PAR_RESISTANCE]/R_coefficient;
	Ls = (float)MCPara[PAR_INDUCTANCE_LQ]/L_coefficient;
	flux =((((float)MCPara[PAR_BEMF_CONST]/BEMF_coefficient)/1000.0f)*60.0f)/((float)ANGLE_2PI*1.73205f*(float)MCPara[PAR_POLES]);

//	Rs = 90.0f/1000.0f;
//	Ls = 185.0f/1000000.0f;
//	flux =0.0059f;
//	Rs = 13.017f;//电阻
//	Ls = 0.04527;//电感
//	flux = 0.22f;//磁链

	Q_0_0 = 0.1f;
	//Q_0_1 = 0.0f;
	//Q_0_2 = 0.0f;
	//Q_0_3 = 0.0f;
	//Q_1_0 = 0.0f;
	Q_1_1 = 0.1f;
	//Q_1_2 = 0.0f;
	//Q_1_3 = 0.0f;
	//Q_2_0 = 0.0f;
	//Q_2_1 = 0.0f;
	Q_2_2 = 0.1f;
	//Q_2_3 = 0.0f;
	//Q_3_0 = 0.0f;
	//Q_3_1 = 0.0f;
	//Q_3_2 = 0.0f;
	Q_3_3 = 0.001f;

	R_0_0 = 0.2f;
	//R_0_1 = 0.0f;
	//R_1_0 = 0.0f;
	R_1_1 = 0.2f;
	
	T = 1.0f/(float)PWM_FREQ; ;//0.0001f;

	H_0_0 = 1.0f;
	//H_0_1 = 0.0f;
	//H_0_2 = 0.0f;
	//H_0_3 = 0.0f;
	//H_1_0 = 0.0f;
	H_1_1 = 1.0f;
	//H_1_2 = 0.0f;
	//H_1_3 = 0.0f;

	B_0_0 = 1.0f/Ls;
	//B_0_1 = 0.0f;
	//B_1_0 = 0.0f;
	B_1_1 = 1.0f/Ls;
	//B_2_0 = 0.0f;
	//B_2_1 = 0.0f;
	//B_3_0 = 0.0f;
	//B_3_1 = 0.0f;

	P0_0_0 = 0.0f;
	P0_0_1 = 0.0f;
	P0_0_2 = 0.0f;
	P0_0_3 = 0.0f;
	P0_1_0 = 0.0f;
	P0_1_1 = 0.0f;
	P0_1_2 = 0.0f;
	P0_1_3 = 0.0f;
	P0_2_0 = 0.0f;
	P0_2_1 = 0.0f;
	P0_2_2 = 0.0f;
	P0_2_3 = 0.0f;
	P0_3_0 = 0.0f;
	P0_3_1 = 0.0f;
	P0_3_2 = 0.0f;
	P0_3_3 = 0.0f;
}




void stm32_ekf_Update_wrapper(void)
{
	vs_ab_0_0 = mcApp_V_AlphaBetaParam.alpha*mcApp_focParam.MaxPhaseVoltage;//Voltage_Alpha_Beta.Valpha;
	vs_ab_1_0 = mcApp_V_AlphaBetaParam.beta*mcApp_focParam.MaxPhaseVoltage;//Voltage_Alpha_Beta.Vbeta;
	is_ab_0_0 = mcApp_I_AlphaBetaParam.alpha;//Current_Ialpha_beta.Ialpha;
	is_ab_1_0 = mcApp_I_AlphaBetaParam.beta;//Current_Ialpha_beta.Ibeta;
	
//	vs_ab_0_0 = 0.1f;//Voltage_Alpha_Beta.Valpha;
//	vs_ab_1_0 = 0.1f;//Voltage_Alpha_Beta.Vbeta;
//	is_ab_0_0 = 0.1f;//Current_Ialpha_beta.Ialpha;
//	is_ab_1_0 = 0.1f;//Current_Ialpha_beta.Ibeta;
	
	F_0_0 = -Rs/Ls;
	//F_0_1 = 0.0f;
	F_0_2 = flux/Ls*mcApp_SincosParam.Sin;
	F_0_3 = flux/Ls*tempa_2_0*mcApp_SincosParam.Cos;
	//F_1_0 = 0.0f;
	F_1_1 = -Rs/Ls;
	F_1_2 = -flux/Ls*mcApp_SincosParam.Cos;
	F_1_3 = flux/Ls*tempa_2_0*mcApp_SincosParam.Sin;
	//F_2_0 = 0.0f;
	//F_2_1 = 0.0f;
	//F_2_2 = 0.0f;
	//F_2_3 = 0.0f;
	//F_3_0 = 0.0f;
	//F_3_1 = 0.0f;
	F_3_2 = 1.0f;
	//F_3_3 = 0.0f;


	temp_0_0 = 1.0f;
	//temp_0_1 = 0.0f;
	//temp_0_2 = 0.0f;
	//temp_0_3 = 0.0f;
	//temp_1_0 = 0.0f;
	temp_1_1 = 1.0f;
	//temp_1_2 = 0.0f;
	//temp_1_3 = 0.0f;
	//temp_2_0 = 0.0f;
	//temp_2_1 = 0.0f;
	temp_2_2 = 1.0f;
	//temp_2_3 = 0.0f;
	//temp_3_0 = 0.0f;
	//temp_3_1 = 0.0f;
	//temp_3_2 = 0.0f;
	temp_3_3 = 1.0f;


	f1_0_0 = -Rs/Ls*tempa_0_0+flux/Ls*tempa_2_0*mcApp_SincosParam.Sin;
	f1_1_0 = -Rs/Ls*tempa_1_0-flux/Ls*tempa_2_0*mcApp_SincosParam.Cos;
	f1_2_0 = 0.0f;
	f1_3_0 = tempa_2_0;



	f2_0_0 = temp_0_0 + (T*F_0_0);
	//f2_0_1 = temp_0_1;
	f2_0_2 = (T*F_0_2);
	f2_0_3 = (T*F_0_3);
	//f2_1_0 = temp_1_0;
	f2_1_1 = temp_1_1 + (T*F_1_1);
	f2_1_2 = (T*F_1_2);
	f2_1_3 = (T*F_1_3);
	//f2_2_0 = temp_2_0;
	//f2_2_1 = temp_2_1;
	f2_2_2 = temp_2_2;
	//f2_2_3 = temp_2_3;
	//f2_3_0 = temp_3_0;
	//f2_3_1 = temp_3_1;
	f2_3_2 = (T*F_3_2);
	f2_3_3 = temp_3_3;

	X_pred_0_0 = tempa_0_0 + T*(f1_0_0 + B_0_0*vs_ab_0_0);
	X_pred_1_0 = tempa_1_0 + T*(f1_1_0 + B_1_1*vs_ab_1_0);
	X_pred_2_0 = tempa_2_0 + T*(f1_2_0);
	X_pred_3_0 = tempa_3_0 + T*(f1_3_0);

	Y_pred_0_0 = H_0_0*X_pred_0_0;
	Y_pred_1_0 = H_1_1*X_pred_1_0;


	Y_0_0 = is_ab_0_0;
	Y_1_0 = is_ab_1_0;

	P_pred_0_0 = f2_0_0*P0_0_0 + f2_0_2*P0_2_0 + f2_0_3*P0_3_0;
	P_pred_0_1 = f2_0_0*P0_0_1 + f2_0_2*P0_2_1 + f2_0_3*P0_3_1;
	P_pred_0_2 = f2_0_0*P0_0_2 + f2_0_2*P0_2_2 + f2_0_3*P0_3_2;
	P_pred_0_3 = f2_0_0*P0_0_3 + f2_0_2*P0_2_3 + f2_0_3*P0_3_3;    
	P_pred_1_0 = f2_1_1*P0_1_0 + f2_1_2*P0_2_0 + f2_1_3*P0_3_0;
	P_pred_1_1 = f2_1_1*P0_1_1 + f2_1_2*P0_2_1 + f2_1_3*P0_3_1;
	P_pred_1_2 = f2_1_1*P0_1_2 + f2_1_2*P0_2_2 + f2_1_3*P0_3_2;
	P_pred_1_3 = f2_1_1*P0_1_3 + f2_1_2*P0_2_3 + f2_1_3*P0_3_3;  
	P_pred_2_0 = f2_2_2*P0_2_0;
	P_pred_2_1 = f2_2_2*P0_2_1;
	P_pred_2_2 = f2_2_2*P0_2_2;
	P_pred_2_3 = f2_2_2*P0_2_3;
	P_pred_3_0 = f2_3_2*P0_2_0 + f2_3_3*P0_3_0;
	P_pred_3_1 = f2_3_2*P0_2_1 + f2_3_3*P0_3_1;
	P_pred_3_2 = f2_3_2*P0_2_2 + f2_3_3*P0_3_2;
	P_pred_3_3 = f2_3_2*P0_2_3 + f2_3_3*P0_3_3;

	P_pred_0_0 = P_pred_0_0*f2_0_0 + P_pred_0_2*f2_0_2 + P_pred_0_3*f2_0_3 + Q_0_0;
	P_pred_0_1 = P_pred_0_1*f2_1_1 + P_pred_0_2*f2_1_2 + P_pred_0_3*f2_1_3;
	P_pred_0_2 = P_pred_0_2*f2_2_2;
	P_pred_0_3 = P_pred_0_2*f2_3_2 + P_pred_0_3*f2_3_3;    
	P_pred_1_0 = P_pred_1_0*f2_0_0 + P_pred_1_2*f2_0_2 + P_pred_1_3*f2_0_3;
	P_pred_1_1 = P_pred_1_1*f2_1_1 + P_pred_1_2*f2_1_2 + P_pred_1_3*f2_1_3 + Q_1_1;
	P_pred_1_2 = P_pred_1_2*f2_2_2;
	P_pred_1_3 = P_pred_1_2*f2_3_2 + P_pred_1_3*f2_3_3;    
	P_pred_2_0 = P_pred_2_0*f2_0_0 + P_pred_2_2*f2_0_2 + P_pred_2_3*f2_0_3;
	P_pred_2_1 = P_pred_2_1*f2_1_1 + P_pred_2_2*f2_1_2 + P_pred_2_3*f2_1_3;
	P_pred_2_2 = P_pred_2_2*f2_2_2 + Q_2_2;
	P_pred_2_3 = P_pred_2_2*f2_3_2 + P_pred_2_3*f2_3_3; 
	P_pred_3_0 = P_pred_3_0*f2_0_0 + P_pred_3_2*f2_0_2 + P_pred_3_3*f2_0_3;
	P_pred_3_1 = P_pred_3_1*f2_1_1 + P_pred_3_2*f2_1_2 + P_pred_3_3*f2_1_3;
	P_pred_3_2 = P_pred_3_2*f2_2_2;
	P_pred_3_3 = P_pred_3_2*f2_3_2 + P_pred_3_3*f2_3_3 + Q_3_3;

			temp_0_0 = H_0_0*P_pred_0_0;
			temp_0_1 = H_0_0*P_pred_0_1;
			temp_0_2 = H_0_0*P_pred_0_2;
			temp_0_3 = H_0_0*P_pred_0_3;   
			temp_1_0 = H_1_1*P_pred_1_0;
			temp_1_1 = H_1_1*P_pred_1_1;
			temp_1_2 = H_1_1*P_pred_1_2;
			temp_1_3 = H_1_1*P_pred_1_3;


			temp_0_0 = temp_0_0*H_0_0+ R_0_0;
			temp_0_1 = temp_0_1*H_1_1;    
			temp_1_0 = temp_1_0*H_0_0;
			temp_1_1 = temp_1_1*H_1_1 + R_1_1;

	temp_0_0_t = temp_0_0;
	temp_0_1_t = temp_0_1;
	temp_1_0_t = temp_1_0;
	temp_1_1_t = temp_1_1;

	temp = temp_0_0*temp_1_1 - temp_0_1*temp_1_0;
	if(temp != 0)
	{
		 temp_0_0 = temp_1_1_t/temp;
		 temp_0_1 = -temp_0_1_t/temp;
		 temp_1_0 = -temp_1_0_t/temp;
		 temp_1_1 = temp_0_0_t/temp;
	}

	K_0_0 = P_pred_0_0*H_0_0;
	K_0_1 = P_pred_0_1*H_1_1;    
	K_1_0 = P_pred_1_0*H_0_0;
	K_1_1 = P_pred_1_1*H_1_1;   
	K_2_0 = P_pred_2_0*H_0_0;
	K_2_1 = P_pred_2_1*H_1_1;   
	K_3_0 = P_pred_3_0*H_0_0;
	K_3_1 = P_pred_3_1*H_1_1;



	K_0_0 = K_0_0*temp_0_0 + K_0_1*temp_1_0;
	K_0_1 = K_0_0*temp_0_1 + K_0_1*temp_1_1;  
	K_1_0 = K_1_0*temp_0_0 + K_1_1*temp_1_0;
	K_1_1 = K_1_0*temp_0_1 + K_1_1*temp_1_1;    
	K_2_0 = K_2_0*temp_0_0 + K_2_1*temp_1_0;
	K_2_1 = K_2_0*temp_0_1 + K_2_1*temp_1_1;   
	K_3_0 = K_3_0*temp_0_0 + K_3_1*temp_1_0;
	K_3_1 = K_3_0*temp_0_1 + K_3_1*temp_1_1;

	tempa_0_0 = X_pred_0_0 + K_0_0*(Y_0_0 - Y_pred_0_0) + K_0_1*(Y_1_0 - Y_pred_1_0);
	tempa_1_0 = X_pred_1_0 + K_1_0*(Y_0_0 - Y_pred_0_0) + K_1_1*(Y_1_0 - Y_pred_1_0);
	tempa_2_0 = X_pred_2_0 + K_2_0*(Y_0_0 - Y_pred_0_0) + K_2_1*(Y_1_0 - Y_pred_1_0);
	tempa_3_0 = X_pred_3_0 + K_3_0*(Y_0_0 - Y_pred_0_0) + K_3_1*(Y_1_0 - Y_pred_1_0);

			temp_0_0 = K_0_0*H_0_0;
			temp_0_1 = K_0_1*H_1_1;
			//temp_0_2 = 0.0f;
			//temp_0_3 = 0.0f;    
			temp_1_0 = K_1_0*H_0_0;
			temp_1_1 = K_1_1*H_1_1;
			//temp_1_2 = 0.0f;
			//temp_1_3 = 0.0f;    
			temp_2_0 = K_2_0*H_0_0;
			temp_2_1 = K_2_1*H_1_1;
			//temp_2_2 = 0.0f;
			//temp_2_3 = 0.0f;   
			temp_3_0 = K_3_0*H_0_0;
			temp_3_1 = K_3_1*H_1_1;
			//temp_3_2 = 0.0f;
			//temp_3_3 = 0.0f;


		 
		 
			P0_0_0 =   P_pred_0_0 - (temp_0_0*P_pred_0_0 + temp_0_1*P_pred_1_0);
			P0_0_1 =   P_pred_0_1 - (temp_0_0*P_pred_0_1 + temp_0_1*P_pred_1_1);
			P0_0_2 =   P_pred_0_2 - (temp_0_0*P_pred_0_2 + temp_0_1*P_pred_1_2);
			P0_0_3 =   P_pred_0_3 - (temp_0_0*P_pred_0_3 + temp_0_1*P_pred_1_3);  
			P0_1_0 =   P_pred_1_0 - (temp_1_0*P_pred_0_0 + temp_1_1*P_pred_1_0);
			P0_1_1 =   P_pred_1_1 - (temp_1_0*P_pred_0_1 + temp_1_1*P_pred_1_1);
			P0_1_2 =   P_pred_1_2 - (temp_1_0*P_pred_0_2 + temp_1_1*P_pred_1_2);
			P0_1_3 =   P_pred_1_3 - (temp_1_0*P_pred_0_3 + temp_1_1*P_pred_1_3);    
			P0_2_0 =   P_pred_2_0 - (temp_2_0*P_pred_0_0 + temp_2_1*P_pred_1_0);
			P0_2_1 =   P_pred_2_1 - (temp_2_0*P_pred_0_1 + temp_2_1*P_pred_1_1);
			P0_2_2 =   P_pred_2_2 - (temp_2_0*P_pred_0_2 + temp_2_1*P_pred_1_2);
			P0_2_3 =   P_pred_2_3 - (temp_2_0*P_pred_0_3 + temp_2_1*P_pred_1_3);  
			P0_3_0 =   P_pred_3_0 - (temp_3_0*P_pred_0_0 + temp_3_1*P_pred_1_0);
			P0_3_1 =   P_pred_3_1 - (temp_3_0*P_pred_0_1 + temp_3_1*P_pred_1_1);
			P0_3_2 =   P_pred_3_2 - (temp_3_0*P_pred_0_2 + temp_3_1*P_pred_1_2);
			P0_3_3 =   P_pred_3_3 - (temp_3_0*P_pred_0_3 + temp_3_1*P_pred_1_3);
			if(tempa_3_0>(6.2831853f)){
				 tempa_3_0 -= (6.2831853f);
			}else if(tempa_3_0<(0.0f)){
				 tempa_3_0 += (6.2831853f);
			}
//	tempa_0_0 = tempa_0_0;//Iap
//	tempa_1_0 = tempa_1_0;//Ibt
//	tempa_2_0 = tempa_2_0;//W
//	tempa_3_0 = tempa_3_0;//角度
			mcApp_EstimParam.qRho=tempa_3_0;
			mcApp_EstimParam.qVelEstim=tempa_2_0;
		
}
#endif

