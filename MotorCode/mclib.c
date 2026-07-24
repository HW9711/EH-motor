#include "mclib.h"
#include "math.h"
#include "mcctl.h"
//全部PID结构体定义，
mcParam_PIController     			mcApp_Q_PIParam;      // Parameters for Q axis Current PI Controller 
mcParam_PIController     			mcApp_D_PIParam;      // Parameters for D axis Current PI Controller 
mcParam_PIController     			mcApp_Speed_PIParam;  // Parameters for Speed PI Controller 
mcParam_PIController     			mcApp_Cur_PIParam;

mcParam_FOC						      	mcApp_focParam;       // Parameters related to Field Oriented Control
mcParam_SinCos					      mcApp_SincosParam;    // Parameters related to Sine/Cosine calculator
mcParam_SVPWM 						    mcApp_SVGenParam;     // Parameters related to Space Vector PWM
mcParam_ControlRef 					  mcApp_ControlParam;   // Parameters related to Current and Speed references
mcParam_PLLEstimator          mcApp_EstimParam;     // Parameters related to PLL Estimator
mcParam_AlphaBeta             mcApp_I_AlphaBetaParam; // Alpha and Beta (2 Phase Stationary Frame) axis Current values
mcParam_DQ                    mcApp_I_DQParam;// D and Q axis (2 Phase Rotating Frame) current values
mcParam_ABC                   mcApp_I_ABCParam; // A,B,C axis (3 Phase Stationary Frame) current values
mcParam_AlphaBeta             mcApp_V_AlphaBetaParam; // Alpha and Beta (2 Phase Stationary Frame) axis voltage values
mcParam_DQ                    mcApp_V_DQParam;// D and Q axis (2 Phase Rotating Frame) voltage values


mcParam_PIController     			mcApp_Q2_PIParam;      // Parameters for Q axis Current PI Controller 
mcParam_PIController     			mcApp_D2_PIParam;      // Parameters for D axis Current PI Controller 
mcParam_PIController     			mcApp_Speed2_PIParam;  // Parameters for Speed PI Controller 
mcParam_PIController     			mcApp_Cur2_PIParam;		


mcParam_PIController				mcApp_VoltageP_PIParam;		//Buckd电路的PI参数
mcParam_PIController				mcApp_VoltageP_PIParam2;	//Buckd电路的PI参数



mcParam_FOC						      	mcApp_focParam2;       // Parameters related to Field Oriented Control
mcParam_SinCos					      mcApp_SincosParam2;    // Parameters related to Sine/Cosine calculator
mcParam_SVPWM 						    mcApp_SVGenParam2;     // Parameters related to Space Vector PWM
mcParam_ControlRef 					  mcApp_ControlParam2;   // Parameters related to Current and Speed references
mcParam_PLLEstimator          mcApp_EstimParam2;     // Parameters related to PLL Estimator
mcParam_AlphaBeta             mcApp_I2_AlphaBetaParam; // Alpha and Beta (2 Phase Stationary Frame) axis Current values
mcParam_DQ                    mcApp_I2_DQParam;// D and Q axis (2 Phase Rotating Frame) current values
mcParam_ABC                   mcApp_I2_ABCParam; // A,B,C axis (3 Phase Stationary Frame) current values
mcParam_AlphaBeta             mcApp_V2_AlphaBetaParam; // Alpha and Beta (2 Phase Stationary Frame) axis voltage values
mcParam_DQ                    mcApp_V2_DQParam;// D and Q axis (2 Phase Rotating Frame) voltage values

//0-2PI的sin值；sin(3.1415926*2*x/256);x=0,1,2.....255
const float sineTable[TABLE_SIZE] = 
{
	0,
	0.024541,
	0.049068,
	0.073565,
	0.098017,
	0.122411,
	0.14673,
	0.170962,
	0.19509,
	0.219101,
	0.24298,
	0.266713,
	0.290285,
	0.313682,
	0.33689,
	0.359895,
	0.382683,
	0.405241,
	0.427555,
	0.449611,
	0.471397,
	0.492898,
	0.514103,
	0.534998,
	0.55557,
	0.575808,
	0.595699,
	0.615232,
	0.634393,
	0.653173,
	0.671559,
	0.689541,
	0.707107,
	0.724247,
	0.740951,
	0.757209,
	0.77301,
	0.788346,
	0.803208,
	0.817585,
	0.83147,
	0.844854,
	0.857729,
	0.870087,
	0.881921,
	0.893224,
	0.903989,
	0.91421,
	0.92388,
	0.932993,
	0.941544,
	0.949528,
	0.95694,
	0.963776,
	0.970031,
	0.975702,
	0.980785,
	0.985278,
	0.989177,
	0.99248,
	0.995185,
	0.99729,
	0.998795,
	0.999699,
	1,
	0.999699,
	0.998795,
	0.99729,
	0.995185,
	0.99248,
	0.989177,
	0.985278,
	0.980785,
	0.975702,
	0.970031,
	0.963776,
	0.95694,
	0.949528,
	0.941544,
	0.932993,
	0.92388,
	0.91421,
	0.903989,
	0.893224,
	0.881921,
	0.870087,
	0.857729,
	0.844854,
	0.83147,
	0.817585,
	0.803208,
	0.788346,
	0.77301,
	0.757209,
	0.740951,
	0.724247,
	0.707107,
	0.689541,
	0.671559,
	0.653173,
	0.634393,
	0.615232,
	0.595699,
	0.575808,
	0.55557,
	0.534998,
	0.514103,
	0.492898,
	0.471397,
	0.449611,
	0.427555,
	0.405241,
	0.382683,
	0.359895,
	0.33689,
	0.313682,
	0.290285,
	0.266713,
	0.24298,
	0.219101,
	0.19509,
	0.170962,
	0.14673,
	0.122411,
	0.098017,
	0.073565,
	0.049068,
	0.024541,
	0,
	-0.024541,
	-0.049068,
	-0.073565,
	-0.098017,
	-0.122411,
	-0.14673,
	-0.170962,
	-0.19509,
	-0.219101,
	-0.24298,
	-0.266713,
	-0.290285,
	-0.313682,
	-0.33689,
	-0.359895,
	-0.382683,
	-0.405241,
	-0.427555,
	-0.449611,
	-0.471397,
	-0.492898,
	-0.514103,
	-0.534998,
	-0.55557,
	-0.575808,
	-0.595699,
	-0.615232,
	-0.634393,
	-0.653173,
	-0.671559,
	-0.689541,
	-0.707107,
	-0.724247,
	-0.740951,
	-0.757209,
	-0.77301,
	-0.788346,
	-0.803208,
	-0.817585,
	-0.83147,
	-0.844854,
	-0.857729,
	-0.870087,
	-0.881921,
	-0.893224,
	-0.903989,
	-0.91421,
	-0.92388,
	-0.932993,
	-0.941544,
	-0.949528,
	-0.95694,
	-0.963776,
	-0.970031,
	-0.975702,
	-0.980785,
	-0.985278,
	-0.989177,
	-0.99248,
	-0.995185,
	-0.99729,
	-0.998795,
	-0.999699,
	-1,
	-0.999699,
	-0.998795,
	-0.99729,
	-0.995185,
	-0.99248,
	-0.989177,
	-0.985278,
	-0.980785,
	-0.975702,
	-0.970031,
	-0.963776,
	-0.95694,
	-0.949528,
	-0.941544,
	-0.932993,
	-0.92388,
	-0.91421,
	-0.903989,
	-0.893224,
	-0.881921,
	-0.870087,
	-0.857729,
	-0.844854,
	-0.83147,
	-0.817585,
	-0.803208,
	-0.788346,
	-0.77301,
	-0.757209,
	-0.740951,
	-0.724247,
	-0.707107,
	-0.689541,
	-0.671559,
	-0.653173,
	-0.634393,
	-0.615232,
	-0.595699,
	-0.575808,
	-0.55557,
	-0.534998,
	-0.514103,
	-0.492898,
	-0.471397,
	-0.449611,
	-0.427555,
	-0.405241,
	-0.382683,
	-0.359895,
	-0.33689,
	-0.313682,
	-0.290285,
	-0.266713,
	-0.24298,
	-0.219101,
	-0.19509,
	-0.170962,
	-0.14673,
	-0.122411,
	-0.098017,
	-0.073565,
	-0.049068,
	-0.024541
};

//0-2PI的cos值；cos(3.1415926*2*x/256);
const float cosineTable[TABLE_SIZE] = 
{
	1,
	0.999699,
	0.998795,
	0.99729,
	0.995185,
	0.99248,
	0.989177,
	0.985278,
	0.980785,
	0.975702,
	0.970031,
	0.963776,
	0.95694,
	0.949528,
	0.941544,
	0.932993,
	0.92388,
	0.91421,
	0.903989,
	0.893224,
	0.881921,
	0.870087,
	0.857729,
	0.844854,
	0.83147,
	0.817585,
	0.803208,
	0.788346,
	0.77301,
	0.757209,
	0.740951,
	0.724247,
	0.707107,
	0.689541,
	0.671559,
	0.653173,
	0.634393,
	0.615232,
	0.595699,
	0.575808,
	0.55557,
	0.534998,
	0.514103,
	0.492898,
	0.471397,
	0.449611,
	0.427555,
	0.405241,
	0.382683,
	0.359895,
	0.33689,
	0.313682,
	0.290285,
	0.266713,
	0.24298,
	0.219101,
	0.19509,
	0.170962,
	0.14673,
	0.122411,
	0.098017,
	0.073565,
	0.049068,
	0.024541,
	0,
	-0.024541,
	-0.049068,
	-0.073565,
	-0.098017,
	-0.122411,
	-0.14673,
	-0.170962,
	-0.19509,
	-0.219101,
	-0.24298,
	-0.266713,
	-0.290285,
	-0.313682,
	-0.33689,
	-0.359895,
	-0.382683,
	-0.405241,
	-0.427555,
	-0.449611,
	-0.471397,
	-0.492898,
	-0.514103,
	-0.534998,
	-0.55557,
	-0.575808,
	-0.595699,
	-0.615232,
	-0.634393,
	-0.653173,
	-0.671559,
	-0.689541,
	-0.707107,
	-0.724247,
	-0.740951,
	-0.757209,
	-0.77301,
	-0.788346,
	-0.803208,
	-0.817585,
	-0.83147,
	-0.844854,
	-0.857729,
	-0.870087,
	-0.881921,
	-0.893224,
	-0.903989,
	-0.91421,
	-0.92388,
	-0.932993,
	-0.941544,
	-0.949528,
	-0.95694,
	-0.963776,
	-0.970031,
	-0.975702,
	-0.980785,
	-0.985278,
	-0.989177,
	-0.99248,
	-0.995185,
	-0.99729,
	-0.998795,
	-0.999699,
	-1,
	-0.999699,
	-0.998795,
	-0.99729,
	-0.995185,
	-0.99248,
	-0.989177,
	-0.985278,
	-0.980785,
	-0.975702,
	-0.970031,
	-0.963776,
	-0.95694,
	-0.949528,
	-0.941544,
	-0.932993,
	-0.92388,
	-0.91421,
	-0.903989,
	-0.893224,
	-0.881921,
	-0.870087,
	-0.857729,
	-0.844854,
	-0.83147,
	-0.817585,
	-0.803208,
	-0.788346,
	-0.77301,
	-0.757209,
	-0.740951,
	-0.724247,
	-0.707107,
	-0.689541,
	-0.671559,
	-0.653173,
	-0.634393,
	-0.615232,
	-0.595699,
	-0.575808,
	-0.55557,
	-0.534998,
	-0.514103,
	-0.492898,
	-0.471397,
	-0.449611,
	-0.427555,
	-0.405241,
	-0.382683,
	-0.359895,
	-0.33689,
	-0.313682,
	-0.290285,
	-0.266713,
	-0.24298,
	-0.219101,
	-0.19509,
	-0.170962,
	-0.14673,
	-0.122411,
	-0.098017,
	-0.073565,
	-0.049068,
	-0.024541,
	0,
	0.024541,
	0.049068,
	0.073565,
	0.098017,
	0.122411,
	0.14673,
	0.170962,
	0.19509,
	0.219101,
	0.24298,
	0.266713,
	0.290285,
	0.313682,
	0.33689,
	0.359895,
	0.382683,
	0.405241,
	0.427555,
	0.449611,
	0.471397,
	0.492898,
	0.514103,
	0.534998,
	0.55557,
	0.575808,
	0.595699,
	0.615232,
	0.634393,
	0.653173,
	0.671559,
	0.689541,
	0.707107,
	0.724247,
	0.740951,
	0.757209,
	0.77301,
	0.788346,
	0.803208,
	0.817585,
	0.83147,
	0.844854,
	0.857729,
	0.870087,
	0.881921,
	0.893224,
	0.903989,
	0.91421,
	0.92388,
	0.932993,
	0.941544,
	0.949528,
	0.95694,
	0.963776,
	0.970031,
	0.975702,
	0.980785,
	0.985278,
	0.989177,
	0.99248,
	0.995185,
	0.99729,
	0.998795,
	0.999699
};

/*****************************************************************************
Clacke变换：3相坐标——>直角坐标abc->alpha,beta
输入：a 、b 、c
输出：alpha 、beta   这个是等幅值变换
*****************************************************************************/
void mcLib_ClarkeTransform(mcParam_ABC *abcParam, mcParam_AlphaBeta *alphabetaParam)
{
    alphabetaParam->alpha = abcParam->a;
    alphabetaParam->beta = (abcParam->a * ONE_BY_SQRT3) + (abcParam->b * TWO_BY_SQRT3);
}
/*****************************************************************************
Park变换：直角坐标转换到旋转直角坐标
输入：角度 、alpha 、beta
输出：d 、q
*****************************************************************************/
void mcLib_ParkTransform(mcParam_AlphaBeta *alphabetaParam , mcParam_SinCos *scParam, mcParam_DQ *dqParam)
{
    dqParam->d =  alphabetaParam->alpha*scParam->Cos + alphabetaParam->beta*scParam->Sin;
    dqParam->q = -alphabetaParam->alpha*scParam->Sin + alphabetaParam->beta*scParam->Cos;
}


/*****************************************************************************
Park逆变换：旋转直角坐标到直角坐标转换
输入：角度 、alpha 、beta
输出：d 、q
*****************************************************************************/

void mcLib_InvParkTransform(mcParam_DQ *dqParam, mcParam_SinCos *scParam,mcParam_AlphaBeta *alphabetaParam)
{
    alphabetaParam->alpha =  dqParam->d*scParam->Cos - dqParam->q*scParam->Sin;
    alphabetaParam->beta  =  dqParam->d*scParam->Sin + dqParam->q*scParam->Cos;       
}


// *****************************************************************************
// *****************************************************************************
// PLL速度估算模块，根据电机参数，和ialpha,ibeta，及vialpa,vbeta;估算反电动势esq，esq，
// 根据esq和反电动势参数进行估算速度，然后对速度进行积分，获得角度；该估算方法对反电动势参数敏感
// *****************************************************************************
// *****************************************************************************
void  mcLib_PLLEstimator(mcParam_PLLEstimator *pllestimatorParam,
                        mcParam_SinCos *scParam, mcParam_FOC *focParam, 
                        mcParam_ControlRef *ctrlParam, 
                        mcParam_AlphaBeta *I_alphabetaParam,
                        mcParam_AlphaBeta *V_alphabetaParam )
{
  	float tempqVelEstim;
    mcParam_DQ BEMF_DQParam;
    mcParam_AlphaBeta BEMF_AlphaBetaParam;
    

	//转子估计转速的绝对值
	if(pllestimatorParam->qVelEstim < 0)
    {
        tempqVelEstim = pllestimatorParam->qVelEstim * (-1);    
    }
    else
    {
        tempqVelEstim = pllestimatorParam->qVelEstim;
    }
    //计算α轴的电流的差值
	pllestimatorParam->qDIalpha	=	(I_alphabetaParam->alpha-pllestimatorParam->qLastIalpha);
	//α轴反电动势感应电压 Vα=  电感L * 电流Iα
    pllestimatorParam->qVIndalpha = (pllestimatorParam->qLsDtq * pllestimatorParam->qDIalpha);
	//计算β轴的电流的差值
	pllestimatorParam->qDIbeta	=	(I_alphabetaParam->beta-pllestimatorParam->qLastIbeta);
	//β轴反电动势感应电压 Vβ=  电感L * 电流Iβ
    pllestimatorParam->qVIndbeta= (pllestimatorParam->qLsDtd * pllestimatorParam->qDIbeta);
    
    // Update LastIalpha and LastIbeta
	// 更新上一次电流α和电流β的的值
    pllestimatorParam->qLastIalpha	=	I_alphabetaParam->alpha;
    pllestimatorParam->qLastIbeta 	=	I_alphabetaParam->beta;
    
    // Stator voltage equations
	//反电动势电压方程：反电动势α = 上一次Vα -（R*Iα）- 本次α轴感应电压（ia*L） 
 	pllestimatorParam->qEsa = BEMF_AlphaBetaParam.alpha = pllestimatorParam->qLastValpha -
							((pllestimatorParam->qRs  * I_alphabetaParam->alpha))
							- pllestimatorParam->qVIndalpha;
	//反电动势电压方程：反电动势β = 上一次Vβ -（R*Iβ）- 本次β轴感应电压（iβ*L） 						
 	pllestimatorParam->qEsb = BEMF_AlphaBetaParam.beta	= 	pllestimatorParam->qLastVbeta -
							((pllestimatorParam->qRs  * I_alphabetaParam->beta ))
							- pllestimatorParam->qVIndbeta;
    
    // Update LastValpha and LastVbeta
	//更新感应α/β轴感应电压的值
	pllestimatorParam->qLastValpha = (focParam->MaxPhaseVoltage * V_alphabetaParam->alpha);
	pllestimatorParam->qLastVbeta = (focParam->MaxPhaseVoltage * V_alphabetaParam->beta);

    // Calculate Sin(Rho) and Cos(Rho)
	//转子估算的角度+转子原始偏移量
    scParam->Angle 	=	pllestimatorParam->qRho + pllestimatorParam->RhoOffset; 
	//计算
	mcLib_SinCosGen(scParam);

   // Translate Back EMF (Alpha,beta)  ESA, ESB to Back EMF(D,Q) ESD, ESQ using Park Transform. 

	//park变换，把得到的Vα和Vβ通过park变换变为 反电动势d轴和q轴的反馈值
    mcLib_ParkTransform(&BEMF_AlphaBetaParam , scParam, &BEMF_DQParam);
    //取出
     pllestimatorParam->qEsd = BEMF_DQParam.d;
     pllestimatorParam->qEsq = BEMF_DQParam.q;
    // Filter first order for Esd and Esq
	pllestimatorParam->qEsdf	= pllestimatorParam->qEsdf+
							  ((pllestimatorParam->qEsd - pllestimatorParam->qEsdf) * pllestimatorParam->qKfilterEsdq) ;

	pllestimatorParam->qEsqf			= pllestimatorParam->qEsqf+
							  ((pllestimatorParam->qEsq - pllestimatorParam->qEsqf) * pllestimatorParam->qKfilterEsdq) ;


    if(ctrlParam->VelRef > pllestimatorParam->qNominal_Speed)
        {
    
            //Using airgap flux as KFi, i.e. KFi = BackEmF_Constant + (Ls*id/(2*pi)).
            //This factors in the air gap flux variation during Field Weakening. 
            //Ls.id is divided by 2*pi to match the units of BackEmF_Constant given in v-sec/rad.

            
            pllestimatorParam->qInvKFi  = (float)(1.0f/( pllestimatorParam->qKFi 
                                       + (pllestimatorParam->qLs_DIV_2_PI*ctrlParam->IdRef)));  
        }
    else
    {
        pllestimatorParam->qInvKFi = pllestimatorParam->qInvKFi_Below_Nominal_Speed;
    }
    
						  
      
	if (tempqVelEstim > pllestimatorParam->qDecimate_Nominal_Speed)
    {
    	if(pllestimatorParam->qEsqf>0)
    	{
    		pllestimatorParam->qOmegaMr	=	((pllestimatorParam->qInvKFi*(pllestimatorParam->qEsqf- pllestimatorParam->qEsdf))) ;
    	} 
		else
    	{
    		pllestimatorParam->qOmegaMr	=	((pllestimatorParam->qInvKFi*(pllestimatorParam->qEsqf + pllestimatorParam->qEsdf)));
    	}
    } 
	else // if est speed<10% => condition VelRef<>0
    {
    	if(pllestimatorParam->qVelEstim>0)
    	{
    		pllestimatorParam->qOmegaMr	=	((pllestimatorParam->qInvKFi*(pllestimatorParam->qEsqf- pllestimatorParam->qEsdf))) ;
    	} 
		else
    	{
    		pllestimatorParam->qOmegaMr	=	((pllestimatorParam->qInvKFi*(pllestimatorParam->qEsqf+ pllestimatorParam->qEsdf))) ;
    	}
    }
    	
	// The integral of the angle is the estimated angle */
	pllestimatorParam->qRho	= 	pllestimatorParam->qRho+
							(pllestimatorParam->qOmegaMr)*(pllestimatorParam->qDeltaT);
    
    if(pllestimatorParam->qRho >= ANGLE_2PI){
			#if _FG == 1

			#endif
				
        pllestimatorParam->qRho = pllestimatorParam->qRho - ANGLE_2PI; 
		}			

     if(pllestimatorParam->qRho <= 0){
			 #if _FG == 1

			 #endif
			 pllestimatorParam->qRho = pllestimatorParam->qRho + ANGLE_2PI; 
		 }
    

    // The estimated speed is a filter value of the above calculated OmegaMr. The filter implementation
    // is the same as for BEMF d-q components filtering
	pllestimatorParam->qVelEstim = (pllestimatorParam->qVelEstim+
						( (pllestimatorParam->qOmegaMr-pllestimatorParam->qVelEstim)*pllestimatorParam->qVelEstimFilterK ));
}



void mcLib_InitPI( mcParam_PIController *pParam)
{
    pParam->qdSum = 0;
    pParam->qOut = 0;
}

void mcLib_CalcPI( mcParam_PIController *pParam)
{
    float U;
    float Exc;
    //误差 = 目标 - 测量
    pParam->qErr  = pParam->qInRef - pParam->qInMeas; 
		
    U  = pParam->qdSum + pParam->qKp * pParam->qErr;
   
    if( U > pParam->qOutMax )
    {
        pParam->qOut = pParam->qOutMax;
    }    
    else if( U < pParam->qOutMin )
    {
        pParam->qOut = pParam->qOutMin;
    }
    else        
    {
        pParam->qOut = U;  
    }
     
    Exc = U - pParam->qOut;
    pParam->qdSum = pParam->qdSum + pParam->qKi * pParam->qErr - pParam->qKc * Exc;
	
	if(pParam->qdSum > pParam->qOutMax )
	{
		pParam->qdSum =  pParam->qOutMax ;
	}
	else if(pParam->qdSum < 0){
		pParam->qdSum = 0 ;
	}
	
	
		//积分限幅
		if(pParam->qdSum > pParam->qdSumMax){
			pParam->qdSum = pParam->qdSumMax;
		}else if(pParam->qdSum < pParam->qdSumMin){
			pParam->qdSum = pParam->qdSumMin;
		}
	
	
}

void mcLib_CalcPI1( mcParam_PIController *pParam)
{
    float U;
    float Exc;
    
    pParam->qErr  = pParam->qInRef - pParam->qInMeas; 
		
    U  = pParam->qdSum + pParam->qKp * pParam->qErr;
   
    if( U > pParam->qOutMax )
    {
        pParam->qOut = pParam->qOutMax;
    }    
    else if( U < pParam->qOutMin )
    {
        pParam->qOut = pParam->qOutMin;
    }
    else        
    {
        pParam->qOut = U;  
    }
     
    Exc = U - pParam->qOut;
    pParam->qdSum = pParam->qdSum + pParam->qKi * pParam->qErr - pParam->qKc * Exc;
}


void mcLib_CalcPI_Pos( mcParam_PIController *pParam)
{
    float U;
    float Exc;
    
    pParam->qErr  = pParam->qInRef - pParam->qInMeas; 
		if(pParam->qErr > 300){
			pParam->qErr = 300;
		}else if(pParam->qErr < -300){
			pParam->qErr = -300;
		}
		
    U  = pParam->qdSum + pParam->qKp * pParam->qErr;
   
    if( U > pParam->qOutMax )
    {
        pParam->qOut = pParam->qOutMax;
    }    
    else if( U < pParam->qOutMin )
    {
        pParam->qOut = pParam->qOutMin;
    }
    else        
    {
        pParam->qOut = U;  
    }
     
    Exc = U - pParam->qOut;
    pParam->qdSum = pParam->qdSum + pParam->qKi * pParam->qErr - pParam->qKc * Exc;
}

// *****************************************************************************
// *****************************************************************************
// Section: MC Sine Cosine Functions
// *****************************************************************************
// *****************************************************************************

void mcLib_SinCosGen(mcParam_SinCos *scParam)
{
   
    // Since we are using "float", it is not possible to get an index of array
    // directly. Almost every time, we will need to do interpolation, as per
    // following equation: -
    // y = y0 + (y1 - y0)*((x - x0)/(x1 - x0))
    
    uint32_t y0_Index;
    uint32_t y0_IndexNext;
    float x0, x1, y0, y1, temp;
    
    // Software check to ensure  0 <= Angle < 2*PI
     if(scParam->Angle <  0) 
        scParam->Angle = scParam->Angle + ANGLE_2PI; 
    if(scParam->Angle >= ANGLE_2PI)
        scParam->Angle = scParam->Angle - ANGLE_2PI; 
    
    y0_Index = (uint32_t)(scParam->Angle/ANGLE_STEP);
    
	  //Added this condition which detects if y0_Index is >=256.
    //Earlier the only check was for y0_IndexNext. 
    //We observed y0_Index > = 256 when the code to reverse the direction of the motor was added
    if(y0_Index>=TABLE_SIZE)
    {
        y0_Index = 0;
        y0_IndexNext = 1;
        x0 = ANGLE_2PI;
        x1 = ANGLE_STEP;
        temp = 0;
    }
    else
    {
        y0_IndexNext = y0_Index + 1;
        if(y0_IndexNext >= TABLE_SIZE )
        {
            y0_IndexNext = 0;
            x1 = ANGLE_2PI;
        }
        else
        {
            x1 = ((y0_IndexNext) * ANGLE_STEP);
        }

        x0 = (y0_Index * ANGLE_STEP);  
    
    
    // Since below calculation is same for sin & cosine, we can do it once and reuse
    
	temp = ((scParam->Angle - x0)/(x1 - x0));
    }
    
	// Find Sine now
    y0 = sineTable[y0_Index];
    y1 = sineTable[y0_IndexNext];     
   scParam->Sin = y0 + ((y1 - y0)*temp);
	
    // Find Cosine now
    y0 = cosineTable[y0_Index];
    y1 = cosineTable[y0_IndexNext];
    scParam->Cos = y0 + ((y1 - y0)*temp);
}

// *****************************************************************************
// *****************************************************************************
// Section: MC Space Vector Modulation Routines
// *****************************************************************************
// *****************************************************************************

void mcLib_SVPWMGen(mcParam_AlphaBeta *alphabetaParam, mcParam_SVPWM *svParam)
{

    //Modified inverse clarke transform which allows using instantaneous phase 
    // value to be used directly to calculate vector times.
    svParam->Vr1 = alphabetaParam->beta;
    svParam->Vr2 = (-alphabetaParam->beta/2 + SQRT3_BY2 * alphabetaParam->alpha);
    svParam->Vr3 = (-alphabetaParam->beta/2 - SQRT3_BY2 * alphabetaParam->alpha);     
    
    if( svParam->Vr1 >= 0 )
    {       
		// (xx1)
        if( svParam->Vr2 >= 0 )
        {
            // (x11)
            // Must be Sector 3 since Sector 7 not allowed
            // Sector 3: (0,1,1)  0-60 degrees
            svParam->Tv1 = svParam->Vr2;
            svParam->Tv2 = svParam->Vr1;
            mcLib_CalcTimes(svParam);

            svParam->dPWM_A = svParam->Ta;
            svParam->dPWM_B = svParam->Tb;
            svParam->dPWM_C = svParam->Tc;


        }
        else
        {            
            // (x01)
            if( svParam->Vr3 >= 0 )
            {
                // Sector 5: (1,0,1)  120-180 degrees
               svParam->Tv1 = svParam->Vr1;
               svParam->Tv2 = svParam->Vr3;
                mcLib_CalcTimes(svParam);

                svParam->dPWM_A = svParam->Tc;
                svParam->dPWM_B = svParam->Ta;
                svParam->dPWM_C = svParam->Tb;


            }
            else
            {
                // Sector 1: (0,0,1)  60-120 degrees
                svParam->Tv1 = -svParam->Vr2;
                svParam->Tv2 = -svParam->Vr3;
                mcLib_CalcTimes(svParam);

                svParam->dPWM_A = svParam->Tb;
                svParam->dPWM_B = svParam->Ta;
                svParam->dPWM_C = svParam->Tc;


            }
        }
    }
    else
    {
        // (xx0)
        if( svParam->Vr2 >= 0 )
        {
			// (x10)
            if( svParam->Vr3 >= 0 )
            {
                // Sector 6: (1,1,0)  240-300 degrees
                svParam->Tv1 = svParam->Vr3;
                svParam->Tv2 = svParam->Vr2;
                mcLib_CalcTimes(svParam);
                svParam->dPWM_A = svParam->Tb;
                svParam->dPWM_B = svParam->Tc;
                svParam->dPWM_C = svParam->Ta;



            }
            else
            {
                // Sector 2: (0,1,0)  300-0 degrees
                svParam->Tv1 = -svParam->Vr3;
                svParam->Tv2 = -svParam->Vr1;
                mcLib_CalcTimes(svParam);

                svParam->dPWM_A = svParam->Ta;
                svParam->dPWM_B = svParam->Tc;
                svParam->dPWM_C = svParam->Tb;

            }
        }
        else
        {            
            // (x00)
            // Must be Sector 4 since Sector 0 not allowed
            // Sector 4: (1,0,0)  180-240 degrees
            svParam->Tv1 = -svParam->Vr1;
            svParam->Tv2 = -svParam->Vr2;
            mcLib_CalcTimes(svParam);

            svParam->dPWM_A = svParam->Tc;
            svParam->dPWM_B = svParam->Tb;
            svParam->dPWM_C = svParam->Ta;


        }
    }
}

// *****************************************************************************
// *****************************************************************************
// Section: 计算Ta Tb Tc时间，PWM T0,T1,T2;
// *****************************************************************************
// *****************************************************************************


void mcLib_CalcTimes(mcParam_SVPWM *svParam)
{
    // Length of Vector T1
    // Length of Vector T2
    // Ta = To/2 + T1 + T2
    // Tb = To/2 + T2
    // Tc = To/2
    svParam->T1 = svParam->PWMPeriod * svParam->Tv1;
    svParam->T2 = svParam->PWMPeriod * svParam->Tv2;
    svParam->Tc = (svParam->PWMPeriod- svParam->T1 - svParam->T2)/2;
    svParam->Tb = svParam->Tc + svParam->T2;
    svParam->Ta = svParam->Tb + svParam->T1;    
    if(svParam->PWMPeriod>0)
     { svParam->Duty=(svParam->T1+svParam->T2)/svParam->PWMPeriod;   //Duty 计算
    }else{
      svParam->Duty=0;
     }
}  

