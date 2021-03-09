/*
 *  Created on: Dec 11, 2013
 *  Author: yury
 *  Modification : Walid REMMAS
 *  remmas.walid@gmail.com
 */

#include "tut_arrows/WrenchDriver.h"
#include <map>
#include <cmath>

namespace
{
	const int NUM_OF_MOTORS = 4;

	enum
	{
		FRONT_RIGHT = 0,
		BACK_RIGHT = 1,
		BACK_LEFT = 2,
		FRONT_LEFT = 3
	};

	// max amplitude applies to amplitude modulation only
	// amplitude in radians
	const float MIN_AMPLITUDE = 0.1;
	const float MAX_AMPLITUDE = 1.2;

	// max frequencies applies to frequency modulation only
	// frequency in hertz
	const float MIN_FREQUENCY = 0.1;
	const float MAX_FREQUENCY = 2.0;

	const float DISTANCE_MULTIPLIER = 1.0 / 0.2; // inverse-proportional to distance between center and fin

	// motor coordinate frame difference in radians, only around Z axis
	// motors are at 30 degree angles
	const float SIN_MOTOR_ANGLES[] = { 0.5, -0.5, 0.5, -0.5 };

	const float COS_MOTOR_ANGLES = 0.86602540378;

	// roll, pitch and yaw as 0, 1, and 2, respectively
	const int ANGULAR_MULTIPLIER[NUM_OF_MOTORS][3] =
	{ -1, -1,  1,
	  -1,  1,  1,
	   1,  1, -1,
	   1, -1, -1 };

	const int REVERSED_ROTATION[NUM_OF_MOTORS] = { 1, 1, -1, -1 };

	const float MAX_DIR_CHANGE = 3.5;

	const float ALL_IN_PHASE_OFFSETS[NUM_OF_MOTORS] = { 0.0, 0, -M_PI, -M_PI };

	const float DIAGONAL_PHASE_OFFSETS[NUM_OF_MOTORS] = { M_PI, 0, 0, -M_PI };

	/// Normalize angle to range [-PI ... PI].
	//
	float normalizeAngle(float angle)
	{
		float x = fmod(angle + M_PI, 2.0 * M_PI);
		if (x < 0)
		{
			x += 2.0 * M_PI;
		}
		return x - M_PI;
	}

	float getMagnitude(const geometry_msgs::Wrench& wrench)
	{
		const geometry_msgs::Vector3& f = wrench.force;
		const geometry_msgs::Vector3& tq = wrench.torque;

		return std::sqrt((f.x * f.x + f.y * f.y + f.z * f.z) + (tq.x * tq.x + tq.y * tq.y + tq.z * tq.z));
	}
}

WrenchDriver::WrenchDriver()
	: nh_()
	, useFastV2_(false)
	, lastModeChangeTime_()
	, modeChangeTime_(3.0)
{
	flippersMsg_ = tut_arrows_msgs::FlippersPtr(new tut_arrows_msgs::Flippers());
	flippersMsg_->flippers = std::vector<tut_arrows_msgs::Flipper>(NUM_OF_MOTORS);

	flippersMsg_->flippers[FRONT_RIGHT].motorNumber = FRONT_RIGHT;
	flippersMsg_->flippers[BACK_RIGHT].motorNumber = BACK_RIGHT;
	flippersMsg_->flippers[BACK_LEFT].motorNumber = BACK_LEFT;
	flippersMsg_->flippers[FRONT_LEFT].motorNumber = FRONT_LEFT;

	ros::NodeHandle nhPrivate("~");

	pub_flippers = nh_.advertise<tut_arrows_msgs::Flippers>("hw/flippers_cmd", 1, true);

	modeCmd_.header.stamp = ros::Time::now();
	modeCmd_.mode = tut_arrows_msgs::FlippersModeCmd::MODE_SLOW;
	nhPrivate.getParam("mode", modeCmd_.mode);
	setMode(modeCmd_);

	nhPrivate.getParam("use_fast_v2", useFastV2_);

	nhPrivate.getParam("mode_change_time", modeChangeTime_);

	sub_force_req = nh_.subscribe("force_req", 1, &WrenchDriver::ProcessWrenchStamped, this);
	sub_mode = nh_.subscribe("force_mode", 1, &WrenchDriver::ProcessModeCmd, this);
    std::cout<<"SUB MODE : "<<sub_mode<<std::endl;
	acousticModemSub_ = nh_.subscribe("APPLICON_INBOX", 10, &WrenchDriver::acousticModemCallback, this);
}

void WrenchDriver::ProcessModeCmd(const tut_arrows_msgs::FlippersModeCmdConstPtr data)
{
	if (data->mode != modeCmd_.mode)
	{
		setMode(*data);
	}
}

void WrenchDriver::setMode(const tut_arrows_msgs::FlippersModeCmd& modeCmd)
{
	modeCmd_ = modeCmd;
	setFrequency(0.5f);
	
	lastModeChangeTime_ = ros::Time::now();
	if (modeCmd.mode == tut_arrows_msgs::FlippersModeCmd::MODE_FAST)
	{
		
		
		if (useFastV2_)
		{
			controlByAmplitude(0, 0, 0, 0, 0, 0);
		}
		
		else
		{
			moveFast(0, 0, 0, 0, 0);
		}

		setFrequency(2.0f);
	}
	else if (modeCmd.mode == tut_arrows_msgs::FlippersModeCmd::MODE_SLOW)
	{	
		setFrequency(1.1f);
		moveSlow(0, 0, 0, 0, 0, 0);
	}

	modeChangeTime_ = modeCmd.mode_change_time;
	lastModeChangeTime_ = ros::Time::now();
}

void WrenchDriver::setFrequency(float frequency)
{
	for (auto& flipper : flippersMsg_->flippers)
	{
		flipper.frequency = frequency;
	}
}

void WrenchDriver::ProcessWrenchStamped(const geometry_msgs::WrenchStampedConstPtr wrenchStamped)
{
	applyWrench(wrenchStamped->wrench);
}

/// Converts force applied to flipper (in Newtons) to flipper amplitude (in radians).
///
float WrenchDriver::forceToAmplitude(float force) const
{
    const float rc = 0.1 ; //distance between the axis of rotation
                           // and the center of gravity of the fin

    const float At = 0.02 ; //Area of the fin
    const float density = 1000.0 ;
    float Cdrot = 0.31 ; //rotation drag coefficient
    
    float f = 1.1 ; //frequency
 
    
   if (fabs(force) < 0.01f)
	{
		return 0.0;
	}

	
	if (modeCmd_.mode == tut_arrows_msgs::FlippersModeCmd::MODE_FAST)
	{	
		
		f=2.1;
		Cdrot = 0.15 ;
		force = std::min(fabs(force), fabs(5.0f));
		//std::cout<<"abs(Force) = "<<force<<std::endl;
		//std::cout<<"returned Amplitude = "<<(180/M_PI) * acos(-force/(8*density*Cdrot*At*rc*rc*M_PI*M_PI*f*f)+1)<<std::endl;		
		return acos(-fabs(force)/(8*density*Cdrot*At*rc*rc*M_PI*M_PI*f*f)+1) ;
		
	}
	
	else if (modeCmd_.mode == tut_arrows_msgs::FlippersModeCmd::MODE_SLOW)
	{
		f=1.1;
		Cdrot = 0.15;
		force = std::min(fabs(force), fabs(5.0f));
		/*
		std::cout<<"Force = "<<force<<std::endl;
		std::cout<<"abs(Force) = "<<force<<std::endl;
		std::cout<<"Without acos = "<<-force/(8.0*density*At*rc*rc*M_PI*M_PI*f*f)<<std::endl;
				
		std::cout<<"returned Amplitude = "<<acos(-force/(8*density*Cdrot*At*rc*rc*M_PI*M_PI*f*f)+1)<<std::endl;
		*/
		return acos(-fabs(force)/(8*density*Cdrot*At*rc*rc*M_PI*M_PI*f*f)+1) ;
		
	}
	
	else
	{
		ROS_WARN("Invalid wrench mode");
		return 0;
	}
	
}

float WrenchDriver::getMaxForce() const
{
	if (modeCmd_.mode == tut_arrows_msgs::FlippersModeCmd::MODE_FAST)
	{
		return 4.0 * 3.5f;
	}
	else if (modeCmd_.mode == tut_arrows_msgs::FlippersModeCmd::MODE_SLOW)
	{
		return 4.0 * 1.8f;
	}
	else
	{
		ROS_WARN("Invalid wrench mode");
		return 0;
	}
}

void WrenchDriver::applyWrench(const geometry_msgs::Wrench& wrench)
{
	if ((ros::Time::now() - lastModeChangeTime_).toSec() < modeChangeTime_)
	{
		return; // prevent new commands from being executed when mode change is still in progress
	}

	flippersMsg_->header.stamp = ros::Time::now();

	if (modeCmd_.mode == tut_arrows_msgs::FlippersModeCmd::MODE_FAST)
	{
		if (useFastV2_)
		{
			controlByAmplitude(wrench.force.x, wrench.force.y, wrench.force.z, wrench.torque.x, wrench.torque.y, wrench.torque.z);
		}
		else
		{
			moveFast(wrench.force.x, wrench.force.z, wrench.torque.x, wrench.torque.y, wrench.torque.z);
		}
	}
	else if (modeCmd_.mode == tut_arrows_msgs::FlippersModeCmd::MODE_SLOW)
	{
		moveSlow(wrench.force.x, wrench.force.y, wrench.force.z, wrench.torque.x, wrench.torque.y, wrench.torque.z);
	}
	else
	{
		ROS_ERROR("Invalid wrench mode");
	}
}



float WrenchDriver::moveSlowVertical(double fz, double fRoll, double fPitch)
{
	// Direction of movement is such that all fins move towards inside zero angle as magnitude
	// of Z force gets smaller.
	static const float MovementDirectionZ[NUM_OF_MOTORS] = { 1.0, -1.0, 1.0, -1.0 };

	static const float MovementDirectionRoll[NUM_OF_MOTORS] = { -1.0, -1.0, 1.0, 1.0 };

	static const float MovementDirectionPitch[NUM_OF_MOTORS] = { -1.0, 1.0, 1.0, -1.0 };

	const float verticalForce = std::sqrt(fz * fz + fRoll * fRoll + fPitch * fPitch);

	const float verticalForcePct = verticalForce / getMaxForce();

	const float direction = fz >= 0 ? 1.0 : -1.0;

	const float verticalForcePerFlipper = verticalForce / NUM_OF_MOTORS;

	auto& flippers = flippersMsg_->flippers;
	for (int motornum = 0; motornum < NUM_OF_MOTORS; ++motornum)
	{
		//if (abs(verticalForce) > 0.3)
		{
		flippers[motornum].amplitude += verticalForcePerFlipper;
		
		flippers[motornum].zeroDirection += direction * MovementDirectionZ[motornum] * verticalForcePct * M_PI_2;
	    }
	}
    
	const float rollForcePerFlipper = fRoll / NUM_OF_MOTORS;
	const float pitchForcePerFlipper = fPitch / NUM_OF_MOTORS;
    
    /*
	for (int motornum = 0; motornum < NUM_OF_MOTORS; ++motornum)
	{
		flippers[motornum].amplitude += rollForcePerFlipper * direction * MovementDirectionRoll[motornum];
		flippers[motornum].amplitude += pitchForcePerFlipper * direction * MovementDirectionPitch[motornum];
	}
	*/

	return verticalForcePct * M_PI_2;
}

float* fuzzify(float Fx, float* UniverseOfDiscourse)  //UniverseOfDiscourse=[VN,N,Z,P,VP]
{   float *MU = new float[5];
    if(Fx <= UniverseOfDiscourse[0])
    {MU[0] = 1;
     MU[1] = 0;
     MU[2] = 0;
     MU[3] = 0;
     MU[4] = 0;
    }
    else if(Fx > UniverseOfDiscourse[0] && Fx <= UniverseOfDiscourse[1])
    {
     MU[0] = (Fx-UniverseOfDiscourse[1])/(UniverseOfDiscourse[0]-UniverseOfDiscourse[1]);
     MU[1] = (Fx-UniverseOfDiscourse[0])/(UniverseOfDiscourse[1]-UniverseOfDiscourse[0]);
     MU[2] = 0;
     MU[3] = 0;
     MU[4] = 0;   
    }
    else if(Fx > UniverseOfDiscourse[1] && Fx <= UniverseOfDiscourse[2])
    {
     MU[0] = 0;
     MU[1] = (Fx)/(UniverseOfDiscourse[1]);
     MU[2] = (Fx-UniverseOfDiscourse[1])/(-UniverseOfDiscourse[1]);
     MU[3] = 0;
     MU[4] = 0;
    }
    else if(Fx > UniverseOfDiscourse[2] && Fx <= UniverseOfDiscourse[3])
    {
     MU[0] = 0;
     MU[1] = 0;
     MU[2] = (Fx-UniverseOfDiscourse[3])/(-UniverseOfDiscourse[3]);
     MU[3] = (Fx)/(UniverseOfDiscourse[3]);
     MU[4] = 0;
    }
    else if(Fx > UniverseOfDiscourse[3] && Fx <= UniverseOfDiscourse[4])
    {
     MU[0] = 0;
     MU[1] = 0;
     MU[2] = 0;
     MU[3] = (Fx-UniverseOfDiscourse[4])/(UniverseOfDiscourse[3]-UniverseOfDiscourse[4]);
     MU[4] = (Fx-UniverseOfDiscourse[3])/(UniverseOfDiscourse[4]-UniverseOfDiscourse[3]);
    }
    else if(Fx > UniverseOfDiscourse[4])
    {
     MU[0] = 0;
     MU[1] = 0;
     MU[2] = 0;
     MU[3] = 0;
     MU[4] = 1;
    }
    return MU;
}

float* computeRulesSlow(float* MU_Fx,float* MU_Fz)
{ 
  float *MU_R = new float[25];
  float *MU_CONFIG = new float[11];
    

  //RULES COMPUTING  
  MU_R[0] = fmin(MU_Fx[0],MU_Fz[0]);  MU_R[1]=fmin(MU_Fx[1],MU_Fz[0]);  MU_R[2]=fmin(MU_Fx[2],MU_Fz[0]);
  MU_R[3] = fmin(MU_Fx[3],MU_Fz[0]);  MU_R[4]=fmin(MU_Fx[4],MU_Fz[0]);  MU_R[5]=fmin(MU_Fx[0],MU_Fz[1]);
  MU_R[6] = fmin(MU_Fx[1],MU_Fz[1]);  MU_R[7]=fmin(MU_Fx[2],MU_Fz[1]);  MU_R[8]=fmin(MU_Fx[3],MU_Fz[1]);
  MU_R[9] = fmin(MU_Fx[4],MU_Fz[1]);  MU_R[10]=fmin(MU_Fx[0],MU_Fz[2]);  MU_R[11]=fmin(MU_Fx[1],MU_Fz[2]);
  MU_R[12] = fmin(MU_Fx[2],MU_Fz[2]);  MU_R[13]=fmin(MU_Fx[3],MU_Fz[2]);  MU_R[14]=fmin(MU_Fx[4],MU_Fz[2]);
  MU_R[15] = fmin(MU_Fx[0],MU_Fz[3]);  MU_R[16]=fmin(MU_Fx[1],MU_Fz[3]);  MU_R[17]=fmin(MU_Fx[2],MU_Fz[3]);
  MU_R[18] = fmin(MU_Fx[3],MU_Fz[3]);  MU_R[19]=fmin(MU_Fx[4],MU_Fz[3]);  MU_R[20]=fmin(MU_Fx[0],MU_Fz[4]);
  MU_R[21] = fmin(MU_Fx[1],MU_Fz[4]);  MU_R[22]=fmin(MU_Fx[2],MU_Fz[4]);  MU_R[23]=fmin(MU_Fx[3],MU_Fz[4]);
  MU_R[24] = fmin(MU_Fx[4],MU_Fz[4]);
  
  //INFERENCE MECHANISM
  MU_CONFIG[0] = MU_R[2];
  MU_CONFIG[1] = fmax(MU_R[1],MU_R[3]);
  MU_CONFIG[2] = fmax(fmax(MU_R[0],MU_R[4]),MU_R[7]);
  MU_CONFIG[3] = fmax(MU_R[6],MU_R[8]);
  MU_CONFIG[4] = fmax(MU_R[5],MU_R[9]);
  MU_CONFIG[5] = fmax(fmax(fmax(MU_R[10],MU_R[11]),fmax(MU_R[12],MU_R[13])),MU_R[14]);
  MU_CONFIG[6] = fmax(MU_R[15],MU_R[19]);
  MU_CONFIG[7] = fmax(MU_R[16],MU_R[18]);
  MU_CONFIG[8] = fmax(fmax(MU_R[17],MU_R[20]),MU_R[24]);
  MU_CONFIG[9] = fmax(MU_R[21],MU_R[23]);
  MU_CONFIG[10] = MU_R[22];
  
  
  return MU_CONFIG;   
  
}




float* computeRulesFast(float* MU_Fx,float* MU_Fz)
{ 
  float *MU_R = new float[25];
  float *MU_CONFIG = new float[20];
    

  //RULES COMPUTING  
  MU_R[0] = fmin(MU_Fx[0],MU_Fz[0]);  MU_R[1]=fmin(MU_Fx[1],MU_Fz[0]);  MU_R[2]=fmin(MU_Fx[2],MU_Fz[0]);
  MU_R[3] = fmin(MU_Fx[3],MU_Fz[0]);  MU_R[4]=fmin(MU_Fx[4],MU_Fz[0]);  MU_R[5]=fmin(MU_Fx[0],MU_Fz[1]);
  MU_R[6] = fmin(MU_Fx[1],MU_Fz[1]);  MU_R[7]=fmin(MU_Fx[2],MU_Fz[1]);  MU_R[8]=fmin(MU_Fx[3],MU_Fz[1]);
  MU_R[9] = fmin(MU_Fx[4],MU_Fz[1]);  MU_R[10]=fmin(MU_Fx[0],MU_Fz[2]);  MU_R[11]=fmin(MU_Fx[1],MU_Fz[2]);
  MU_R[12] = fmin(MU_Fx[2],MU_Fz[2]);  MU_R[13]=fmin(MU_Fx[3],MU_Fz[2]);  MU_R[14]=fmin(MU_Fx[4],MU_Fz[2]);
  MU_R[15] = fmin(MU_Fx[0],MU_Fz[3]);  MU_R[16]=fmin(MU_Fx[1],MU_Fz[3]);  MU_R[17]=fmin(MU_Fx[2],MU_Fz[3]);
  MU_R[18] = fmin(MU_Fx[3],MU_Fz[3]);  MU_R[19]=fmin(MU_Fx[4],MU_Fz[3]);  MU_R[20]=fmin(MU_Fx[0],MU_Fz[4]);
  MU_R[21] = fmin(MU_Fx[1],MU_Fz[4]);  MU_R[22]=fmin(MU_Fx[2],MU_Fz[4]);  MU_R[23]=fmin(MU_Fx[3],MU_Fz[4]);
  MU_R[24] = fmin(MU_Fx[4],MU_Fz[4]);
  
  //INFERENCE MECHANISM
  MU_CONFIG[0] = MU_R[1];
  MU_CONFIG[1] = MU_R[0];
  MU_CONFIG[2] = MU_R[6];
  MU_CONFIG[3] = MU_R[5];
  MU_CONFIG[4] = fmax(MU_R[10],MU_R[11]);
  MU_CONFIG[5] = MU_R[15];
  MU_CONFIG[6] = MU_R[16];
  MU_CONFIG[7] = MU_R[20];
  MU_CONFIG[8] = MU_R[21];
  MU_CONFIG[9] = MU_R[2];
  MU_CONFIG[10] = MU_R[3];
  MU_CONFIG[11] = fmax(MU_R[7],MU_R[4]);
  MU_CONFIG[12] = MU_R[8];
  MU_CONFIG[13] = MU_R[9];
  MU_CONFIG[14] = fmax(MU_R[12], fmax(MU_R[13],MU_R[14]));
  MU_CONFIG[15] = MU_R[19];
  MU_CONFIG[16] = MU_R[18];
  MU_CONFIG[17] = fmax(MU_R[17],MU_R[24]);
  MU_CONFIG[18] = MU_R[23];
  MU_CONFIG[19] = MU_R[22];

  
  
  return MU_CONFIG;   
  
}

float defuzzify(float* MU_CONFIG, float* config, int n)
{
 float sum1=0;
 float sum2=0;
 for (int i=0;i<=n;i++)
 {sum1 += MU_CONFIG[i];
  sum2 += MU_CONFIG[i]*config[i];
 }
 if(sum1 == 0) return 0;
 else return sum2 / sum1;
}


int sign(double f){
	if(f >= 0) return 1;
	else return -1;
}


	


void WrenchDriver::moveSlow(double fx, double fy, double fz, double fRoll, double fPitch, double fYaw)
{

	setFrequency(1.1f);
	// Initial position is such where both front fins and back fins are directed inside.
	// For front fins this is offset 0.0 and for back fins this is offset -PI.
	static const float InitialOffset[NUM_OF_MOTORS] = { 0.0, -M_PI, -M_PI, 0.0 };
	
	float front_right, front_left, back_right, back_left;
	front_right = 0;
	front_left = 0;
	back_right = 0;
	back_left = 0;
	/*
	if (abs(fx) < 0.05)
	{
		fx = 0;
	}
	*/
	
// SURGE AMPLITUDE ADDITION	
//----------------------------------------
	if (fx > 0)
	{
		front_right += fabs(fx/2.0);
		front_left += fabs(fx/2.0);
	}
	else if (fx < 0)
	{
		back_right += fabs(fx/2.0);
		back_left += fabs(fx/2.0);
	}
//----------------------------------------	
// SWAY AMPLITUDE ADDITION	
//----------------------------------------
	if (fy > 0)
	{
		front_right += fabs(fy/2.0);
		back_right += fabs(fy/2.0);
	}
	else if (fy < 0)
	{
		back_left += fabs(fy/2.0);
		front_left += fabs(fy/2.0);
	}
//----------------------------------------	
// HEAVE AMPLITUDE ADDITION	
//----------------------------------------
	front_right += fabs(fz/4.0);
	back_right += fabs(fz/4.0);
	back_left += fabs(fz/4.0);
	front_left += fabs(fz/4.0);	

//----------------------------------------
// YAW AMPLITUDE ADDITION
//----------------------------------------
    if (fYaw >= 0.05)
	{
		front_right += fabs(fYaw/2.0);
		back_left += fabs(fYaw/2.0);
	}
    else if (fYaw <= -0.05)
	{
		front_left += fabs(fYaw/2.0);
		back_right += fabs(fYaw/2.0);
	}
//----------------------------------------	

	// On one extreme all fins front and back fins are maximally apart and at other extreme
	// they are as close as possible inside. Both front and back fins are pairwise in sync.
	static const float PhaseOffsets[NUM_OF_MOTORS] = { 0.0, M_PI, 0.0, -M_PI };

	// Initialisation of Universes of Discourse for Fx and Fz
	const float VN_Fx = -4;
	const float N_Fx = -2;
        const float VP_Fx = -VN_Fx;
	const float P_Fx = -N_Fx;
	float *UOD_Fx = new float[5]{VN_Fx, N_Fx, 0, P_Fx, VP_Fx};
    
    
        const float VN_Fz = -4;
	const float N_Fz = -2;
        const float VP_Fz = -VN_Fz;
	const float P_Fz = -N_Fz;
	float *UOD_Fz = new float[5]{VN_Fz, N_Fz, 0, P_Fz, VP_Fz};

    
    // Initialisation of Universes of Discourse for Fx and Fz
    float *config = new float[11]{-M_PI/2, -M_PI/3, -M_PI/6, -M_PI/6, -M_PI/6, 0, M_PI/6, M_PI/6, M_PI/6, M_PI/3, M_PI/2};                                                                                    																					
    
    float finsdirection = defuzzify(computeRulesSlow(fuzzify(fx, UOD_Fx), fuzzify(fz, UOD_Fz)), config, 10);

	auto& flippers = flippersMsg_->flippers;
	
	static const float MovementDirectionZ[NUM_OF_MOTORS] = { 1.0, -1.0, 1.0, -1.0 };
	const float direction = fz >= 0 ? 1.0 : -1.0;

	for (int motornum = 0; motornum < NUM_OF_MOTORS; motornum++)
	{
		flippers[motornum].amplitude = 0;
		flippers[motornum].phaseOffset = PhaseOffsets[motornum];
		flippers[motornum].zeroDirection = InitialOffset[motornum];
	}
	
        flippers[0].zeroDirection +=  finsdirection * MovementDirectionZ[0];
        flippers[1].zeroDirection +=  finsdirection * MovementDirectionZ[1];
        flippers[2].zeroDirection +=  finsdirection * MovementDirectionZ[2];
        flippers[3].zeroDirection +=  finsdirection * MovementDirectionZ[3];
    	
	flippers[0].amplitude += front_right;
	flippers[1].amplitude += back_right;
	flippers[2].amplitude += back_left;
	flippers[3].amplitude += front_left;
	
	/*
	std::cout<<"Force flipper 0 = "<<flippers[0].amplitude<<std::endl;
	std::cout<<"Force flipper 1 = "<<flippers[1].amplitude<<std::endl;
	std::cout<<"Force flipper 2 = "<<flippers[2].amplitude<<std::endl;
	std::cout<<"Force flipper 3 = "<<flippers[3].amplitude<<std::endl;
	std::cout<<"------------------------------------------"<<std::endl;
	*/
	convertForcesToAmplitudes();
	/*
	std::cout<<"Amplitude flipper 0 = "<<flippers[0].amplitude<<std::endl;
	std::cout<<"Amplitude flipper 1 = "<<flippers[1].amplitude<<std::endl;
	std::cout<<"Amplitude flipper 2 = "<<flippers[2].amplitude<<std::endl;
	std::cout<<"Amplitude flipper 3 = "<<flippers[3].amplitude<<std::endl;
	*/
	limitAmplitudes();

	pub_flippers.publish(flippersMsg_);
}




void WrenchDriver::moveFastYaw(double fx, double fYaw)
{
	static const float YawPosDirection[NUM_OF_MOTORS] = { 1, 1, -1, -1 };
	
	const float fYawPerFlipper = fYaw / NUM_OF_MOTORS;
	const float fYawDiff = std::abs(fYaw);

	//ROS_INFO("Yaw diff: %g", fYawDiff);

	const float fxDirection = fx >= 0 ? 1.0 : -1.0;

	auto& flippers = flippersMsg_->flippers;

	for (int motornum = 0; motornum < NUM_OF_MOTORS; ++motornum)
	{
		flippers[motornum].amplitude += fYawPerFlipper * YawPosDirection[motornum] * fxDirection;
        /*
		if (fxDirection * fYaw * YawPosDirection[motornum] < 0)
		{
			flippers[motornum].amplitude -= fYawDiff;
		}
		*/
	}
}

void WrenchDriver::moveFast(double fx, double fz, double fRoll, double fPitch, double fYaw)
{
	//fx = fx / COS_MOTOR_ANGLES;
	setFrequency(2.1f);
	float totalForce = std::sqrt(fx * fx + fz * fz + fRoll * fRoll + fPitch * fPitch + fYaw * fYaw);
    /*
	if (totalForce > getMaxForce())
	{
		const float normalizer = totalForce / getMaxForce();

		fx /= normalizer;
		fz /= normalizer;
		fRoll /= normalizer;
		fPitch /= normalizer;
		fYaw /= normalizer;
	}
    */
	auto& flippers = flippersMsg_->flippers;
	
	// ====================================================================================++++++
	// Initialisation of Universes of Discourse for Fx and Fz
	const float VN_Fx = -4;
	const float N_Fx = -2;
        const float VP_Fx = -VN_Fx;
	const float P_Fx = -N_Fx;
	float *UOD_Fx = new float[5]{VN_Fx, N_Fx, 0, P_Fx, VP_Fx};
    
    
        const float VN_Fz = -1.8;
	const float N_Fz = -1;
        const float VP_Fz = -VN_Fz;
	const float P_Fz = -N_Fz;
	float *UOD_Fz = new float[5]{VN_Fz, N_Fz, 0, P_Fz, VP_Fz};

    
   	// Initialisation of Universes of Discourse for Fx and Fz

	float *config = new float[20]{-(4*M_PI/3), -(7*M_PI/6), -(8*M_PI/7), -(9*M_PI/8), -M_PI, -(7*M_PI/8), -(6*M_PI/7), -(5*M_PI/6), -(2*M_PI/3), -(M_PI/2), -(M_PI/3), -(M_PI/3), -(M_PI/7), -(M_PI/8), 0, (M_PI/8), (M_PI/7), (M_PI/3), (M_PI/3), (M_PI/2)};                                                                                    																					
    
	float finsdirection = defuzzify(computeRulesFast(fuzzify(fx, UOD_Fx), fuzzify(fz, UOD_Fz)), config, 19);
	// ====================================================================================++++++
	// yaw force adds to x force in same direction i.e. fYaw never decreases horizontal force
	const float fHorizontal = fx + std::abs(fYaw) * (fx >= 0 ? 1.0 : -1.0);

	for (int motornum = 0; motornum < NUM_OF_MOTORS; ++motornum)
	{
		const float fVertical = fz + fRoll * ANGULAR_MULTIPLIER[motornum][0] + fPitch * ANGULAR_MULTIPLIER[motornum][1];
		const float direction = atan2(fVertical, fHorizontal);

		const float flipperForce = (std::sqrt(fHorizontal * fHorizontal + fVertical * fVertical) / NUM_OF_MOTORS);

		flippers[motornum].amplitude = flipperForce;
		flippers[motornum].phaseOffset = DIAGONAL_PHASE_OFFSETS[motornum];
		flippers[motornum].zeroDirection = REVERSED_ROTATION[motornum] * finsdirection * sign(fHorizontal);
		//flippers[motornum].zeroDirection = REVERSED_ROTATION[motornum] * direction;
	}

	moveFastYaw(fx, fYaw);

	convertForcesToAmplitudes();

	limitAmplitudes();

	pub_flippers.publish(flippersMsg_);
}

/// Calculate motor command where frequency is constant and amplitude varies by required force/torque.
///
void WrenchDriver::controlByAmplitude(double fx, double fy, double fz, double fRoll, double fPitch, double fYaw)
{
	auto& flippers = flippersMsg_->flippers;

	float newdirections[NUM_OF_MOTORS];

	for (int motornum = 0; motornum < NUM_OF_MOTORS; motornum++)
	{
		float new_x = fx * COS_MOTOR_ANGLES; // straight movement
		float new_y = fy * SIN_MOTOR_ANGLES[motornum]; // side movement
		float new_yaw = fYaw * DISTANCE_MULTIPLIER * ANGULAR_MULTIPLIER[motornum][2]; // turning

		float new_flat = new_x + new_y + new_yaw;
		float new_z = fz +
				fRoll * DISTANCE_MULTIPLIER * ANGULAR_MULTIPLIER[motornum][0] +
				fPitch * DISTANCE_MULTIPLIER * ANGULAR_MULTIPLIER[motornum][1]; // straight+turning

		const float flipperForce = (sqrt(new_flat * new_flat + new_z * new_z)/NUM_OF_MOTORS);

		flippers[motornum].amplitude = forceToAmplitude(flipperForce);
		flippers[motornum].phaseOffset = (REVERSED_ROTATION[motornum] - 1) * M_PI_2;
		newdirections[motornum] = REVERSED_ROTATION[motornum] * atan2(new_z, new_flat); // to polar coordinates
	}

	limitAmplitudes();

	// slow down the change of direction to make stabilization easier
	for (int motornum = 0; motornum < NUM_OF_MOTORS; motornum++)
	{
		float shortest_path = normalizeAngle(newdirections[motornum] - flippers[motornum].zeroDirection);

		if (shortest_path > MAX_DIR_CHANGE)
		{
			shortest_path = MAX_DIR_CHANGE;
		}
		else if (shortest_path < -MAX_DIR_CHANGE)
		{
			shortest_path = -MAX_DIR_CHANGE;
		}

		flippers[motornum].zeroDirection += shortest_path;
	}

	pub_flippers.publish(flippersMsg_);
}

void WrenchDriver::limitAmplitudes()
{
	auto& flippers = flippersMsg_->flippers;
    
	// find the fastest motor
	
	float maxAmplitude = 0;
	for (int motornum = 0; motornum < NUM_OF_MOTORS; ++motornum)
	{
		maxAmplitude = std::max(maxAmplitude, flippers[motornum].amplitude);
	}

	// if fastest motor is faster than physically possible, scale all motor speeds down
	if (maxAmplitude > MAX_AMPLITUDE)
	{
		const float normalizer = MAX_AMPLITUDE / maxAmplitude;

		for (int motornum = 0; motornum < NUM_OF_MOTORS; ++motornum)
		{
			flippers[motornum].amplitude *= normalizer;
		}
	}

	for (int motornum = 0; motornum < NUM_OF_MOTORS; ++motornum)
	{
		if (flippers[motornum].amplitude < MIN_AMPLITUDE)
		{
			flippers[motornum].amplitude = 0;
		}
	}
	
	
}

/// amplitudes contain force values at the moment, now the summed up forces for each flipper
/// can be converted to amplitudes (NB! this cannot be done earlier as force to amplitude
/// conversion is not linear)
///
void WrenchDriver::convertForcesToAmplitudes()
{
	auto& flippers = flippersMsg_->flippers;

	for (int motornum = 0; motornum < NUM_OF_MOTORS; ++motornum)
	{
		flippers[motornum].amplitude = forceToAmplitude(std::abs(flippers[motornum].amplitude));
	}
}

void WrenchDriver::acousticModemCallback(const auv_msgs::AcousticModemData::ConstPtr& data)
{
	std::istringstream msgStream(std::string(data->payload.begin(), data->payload.end()));

	std::string cmd;
	msgStream >> cmd;

	if (cmd == "SET_FLIPPERS_FREQUENCY")
	{
		try
		{
			float newFrequency = 0.0f;

			if ( !(msgStream >> newFrequency) )
			{
				ROS_WARN("Frequency is required as first argument for SET_FLIPPERS_FREQUENCY");
				return;
			}

			setFrequency(newFrequency);

			ROS_INFO("Flippers frequency set to %g", newFrequency);
		}
		catch (const std::exception& ex)
		{
			ROS_WARN("Error when changing flippers frequency: %s", ex.what());
		}
	}
	else if (cmd == "SET_FLIPPERS_AMPLITUDE")
	{
		try
		{
			float  newAmplitude = 0.0f;

			if ( !(msgStream >> newAmplitude) )
			{
				ROS_WARN("Amplitude is required as first argument for SET_FLIPPERS_AMPLITUDE");
				return;
			}

			auto& flippers = flippersMsg_->flippers;

			int motornum = -1;
			if ( (msgStream >> motornum) )
			{
				for (int i = 0; i < NUM_OF_MOTORS && msgStream; ++i)
				{
					if (motornum >= 0 && motornum < NUM_OF_MOTORS)
					{
						flippers[motornum].amplitude = newAmplitude;
						ROS_INFO("Motor %d amplitude set to %g", motornum, newAmplitude);
					}

					msgStream >> motornum;
				}
			}
			else
			{
				for (int motornum = 0; motornum < NUM_OF_MOTORS; ++motornum)
				{
					flippers[motornum].amplitude = newAmplitude;
				}

				ROS_INFO("Motor amplitudes set to %g", newAmplitude);
			}

			pub_flippers.publish(flippersMsg_);
		}
		catch (const std::exception& ex)
		{
			ROS_WARN("Error when changing flippers amplitude: %s", ex.what());
		}
	}
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "WrenchDriver");
	WrenchDriver wrenchDriver;
	ros::spin();
	return 0;
}
