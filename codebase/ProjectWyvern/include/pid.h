// -------------------------------
// Wyvern Quadrotor
// PID
// version: 1.0.0
// date: April 14, 2010
// authors: William Etter, Paul Martin, Uriah Baalke
// -------------------------------

#define MIN_THRUST 1106
#define MAX_THRUST 1600
#define MOT1 1
#define MOT2 2
#define MOT3 3
#define MOT4 4
#define MOT1_BAL 0
#define MOT2_BAL 0
#define MOT3_BAL 0
#define MOT4_BAL 0
#define GAIN_DIVIDE 10000

// ----------------GLOBAL VARIABLES----------------
int PID_BAND = 120;

//	PID: yaw,pitch,roll
int P_yaw = 0;
int I_yaw = 0;
int D_yaw = 3500;

int P_pitch = 30;
int I_pitch = 0;
int D_pitch = 250;

int P_roll = 46;
int I_roll = 0;
int D_roll = 270;

//	PID: x,y,z
int P_acc = 0;
int I_acc = 0;
int D_acc = 0;

//	PID: altitude
int P_alt = 0;
int I_alt = 0;
int D_alt = 0;

// 	current orientation, acceleration
int yaw = 0;int pitch = 0;int roll = 0;
int xAcc = 0;int yAcc = 0;int zAcc = 0;int alt = 0;

//	desired yaw,pitch,roll & x,y,z & altitude
int yaw_des = 0;int pitch_des = 0;int roll_des = 0;
int x_des = 0;int y_des = 0;int z_des = 0;
int alt_des = 0;

//	current errors (desired - actual)
int error_yaw = 0;int error_pitch = 0;int error_roll = 0;
int error_x = 0;int error_y = 0;int error_z = 0;
int error_alt = 0;

//	past errors (one cycle ago)
int error_yaw_past = 0;int error_pitch_past = 0;int error_roll_past = 0;
int error_x_past = 0;int error_y_past = 0;int error_z_past = 0;
int error_alt_past = 0;

//	yaw,pitch,roll PID calculations
int yaw_P = 0;int yaw_I = 0;int yaw_D = 0;int yaw_sum = 0;
int pitch_P = 0;int pitch_I = 0;int pitch_D = 0;int pitch_sum = 0;
int roll_P = 0;int roll_I = 0;int roll_D = 0;int roll_sum = 0;

int pitch_rate = 0;

//	x,y,z PID calculations
int x_P = 0;int x_I = 0;int x_D = 0;int x_sum = 0;
int y_P = 0;int y_I = 0;int y_D = 0;int y_sum = 0;
int z_P = 0;int z_I = 0;int z_D = 0;int z_sum = 0;

//	altitude PID calculations
int alt_P = 0;int alt_I = 0;int alt_D = 0;int alt_sum = 0;

//	thrust floor of all motors 
int thrustFloor = 0;
int userThrust = 0;

// current PWM / Duty Cycle numbers
int PWM_MOT1 = 0;int PWM_MOT2 = 0;int PWM_MOT3 = 0;int PWM_MOT4 = 0;

// motor enable
int motorEnable = 0;

// have we started? do not allow quick starts
char isStarted = 0;
char activated = 0;

// ----------------FUNCTION HEADERS----------------



/*************************************************************************
Function: PID_setGyro()
Purpose:  changes PID gain values
Input:    PID gains
Returns:  None
**************************************************************************/
void PID_motorEnable(int status){
	motorEnable = status; // 0 or 1
}


/*************************************************************************
Function: PID_setAlt()
Purpose:  changes PID gain values
Input:    PID gains
Returns:  None
**************************************************************************/
void PID_setAlt(int P, int I, int D){
	P_alt = P;
	I_alt = I;
	D_alt = D;
}

/*************************************************************************
Function: PID_setThrust()
Purpose:  stabilize PWM using PID
Input:    IMU values: yaw,pitch,roll,altitude
Returns:  None
**************************************************************************/
void PID_setThrust(int verticalThrust){
	if(isStarted == 1){
		thrustFloor = (verticalThrust*6)/20 + 1200;
	}
}

/*************************************************************************
Function: PID_setThrust()
Purpose:  stabilize PWM using PID
Input:    IMU values: yaw,pitch,roll,altitude
Returns:  None
**************************************************************************/
void PID_command(char command){
	switch(command){
		case 's':
			// idle throttle
			if(activated == 1){
				thrustFloor = 1106;
				motorEnable = 1;
				set_duty(1,1106);
				set_duty(2,1106);
				set_duty(3,1106);
				set_duty(4,1106);	
				isStarted = 1;
			}
			break;
		case ' ':
			// emergency dead stop
			//TransmitString("SPACE\n\r");
			activated = 1;
			motorEnable = 0;
			thrustFloor = 1000;
			set_duty(1,1000);
			set_duty(2,1000);
			set_duty(3,1000);
			set_duty(4,1000);
			isStarted = 0;			
			break;
		case '-':
			userThrust -= 10;
			break;
		case '=':
			if(isStarted == 1)
				userThrust += 10;
			break;
		case '1':	// Start Motor 1
			set_duty(1,1250);
			break;
		case '2':	// Start Motor 2
			set_duty(2,1250);
			break;
		case '3':	// Start Motor 3
			set_duty(3,1250);
			break;
		case '4':	// Start Motor 4
			set_duty(4,1250);
		case 'p':
			P_pitch += 1;
			break;
		case 'o':
			if(P_pitch >= 1)
				P_pitch -= 1;
			break;
		case 'l':
			D_pitch += 2;
			break;
		case 'k':
			if(D_pitch >= 2)
				D_pitch -= 2;
			break;
		case 'i':
			PID_BAND += 1;
			break;
		case 'u':
			if(PID_BAND >= 1)
				PID_BAND -= 1;
			break;
		default:
			break;
	}
}

/*************************************************************************
Function: PID_setPosition()
Purpose:  changes PID gain values
Input:    desired yaw,pitch,roll,alt
Returns:  None
**************************************************************************/
void PID_setPosition(packet_com_t* data){
	yaw_des 	= (data->yaw)/10;
	pitch_des 	= (data->pitch)/70;
	roll_des 	= (data->roll)/70;
}

/*************************************************************************
Function: PID_updatePWM()
Purpose:  stabilize PWM using PID
Input:    IMU values: yaw,pitch,roll,altitude
Returns:  None
**************************************************************************/
void PID_updatePWM(packet_razordata_t* data){
	if(motorEnable == 1){
	
		//	YAW
		error_yaw = 0 - ((int)((data->yaw)-20000));
		yaw_P = ((double)(P_yaw)*(double)(error_yaw))/GAIN_DIVIDE;
		if(yaw_P > PID_BAND){
			yaw_P = PID_BAND;
		}else if(yaw_P < -1*PID_BAND){
			yaw_P = -1*PID_BAND;
		}
		yaw_I += error_yaw;
		yaw_D = ((int)((data->omegatwo)-20000));
		if(yaw_D < 20 && yaw_D > -20){
			yaw_D = 0;
		}
		yaw_D = 0-((double)(D_yaw)*(double)(yaw_D))/GAIN_DIVIDE;
		if(yaw_D > 100){
			yaw_D = 100;
		}else if(yaw_D < -100){
			yaw_D = -100;
		}
		yaw_sum = yaw_P + ((double)(I_yaw)*(double)(yaw_I))/GAIN_DIVIDE + yaw_D + yaw_des;
	

		//	PITCH
		//error_pitch_past = error_pitch;
		error_pitch = 0 - ((int)((data->pitch)-20000));
		pitch_P = ((double)(P_pitch)*(double)(error_pitch))/GAIN_DIVIDE;
		if(pitch_P > PID_BAND){
			pitch_P = PID_BAND;
		}else if(pitch_P < -1*PID_BAND){
			pitch_P = -1*PID_BAND;
		}
		pitch_I += error_pitch;
		pitch_D = 0-((int)((data->omegaone)-20000));
		if(pitch_D < 20 && pitch_D > -20){
			pitch_D = 0;
		}
		pitch_D = ((double)(D_pitch)*(double)(pitch_D))/GAIN_DIVIDE + pitch_des;
		pitch_sum = pitch_P + ((double)(I_pitch)*(double)(pitch_I))/GAIN_DIVIDE + pitch_D;
	
		//	ROLL
		error_roll = 0 - ((int)((data->roll)-20000));
		roll_P = ((double)(P_roll)*(double)(error_roll))/GAIN_DIVIDE;
		if(roll_P > PID_BAND){
			roll_P = PID_BAND;
		}else if(roll_P < -1*PID_BAND){
			roll_P = -1*PID_BAND;
		}
		roll_I += error_roll;
		roll_D = ((int)((data->omegazero)-20000));
		if(roll_D < 20 && roll_D > -20){
			roll_D = 0;
		}
		roll_sum = roll_P + ((double)(I_roll)*(double)(roll_I))/GAIN_DIVIDE + ((double)(D_roll)*(double)(roll_D))/GAIN_DIVIDE + roll_des;

		// 	Calculate motor responses

		PWM_MOT1 = thrustFloor + userThrust - roll_sum + yaw_sum;
		PWM_MOT2 = thrustFloor + userThrust + pitch_sum - yaw_sum;
		PWM_MOT3 = thrustFloor + userThrust + roll_sum + yaw_sum;
		PWM_MOT4 = thrustFloor + userThrust - pitch_sum - yaw_sum;

		//TransmitString("z_sum: "); TransmitInt(z_sum); TransmitString("\n\r");

		if(PWM_MOT1 > MAX_THRUST){
			PWM_MOT1 = MAX_THRUST;
		}else if(PWM_MOT1 < MIN_THRUST){
			PWM_MOT1 = MIN_THRUST;
		}
	
		if(PWM_MOT2 > MAX_THRUST){
			PWM_MOT2 = MAX_THRUST;
		}else if(PWM_MOT2 < MIN_THRUST){
			PWM_MOT2 = MIN_THRUST;
		}

		if(PWM_MOT3 > MAX_THRUST){
			PWM_MOT3 = MAX_THRUST;
		}else if(PWM_MOT3 < MIN_THRUST){
			PWM_MOT3 = MIN_THRUST;
		}
	
		if(PWM_MOT4 > MAX_THRUST){
			PWM_MOT4 = MAX_THRUST;
		}else if(PWM_MOT4 < MIN_THRUST){
			PWM_MOT4 = MIN_THRUST;
		}
	
		set_duty(MOT1,PWM_MOT1);
		set_duty(MOT2,PWM_MOT2+10);
		set_duty(MOT3,PWM_MOT3+8);
		set_duty(MOT4,PWM_MOT4);
	}
	/*
	TransmitInt(pitch_P);
	TransmitString(" ");
	TransmitInt(error_pitch);
	TransmitString("\n\r");
	*/
	
}

