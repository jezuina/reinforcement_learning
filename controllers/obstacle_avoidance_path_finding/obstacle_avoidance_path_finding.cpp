/*python*/
#include </usr/include/python2.7/Python.h>
/* Include the controller definition */
#include "obstacle_avoidance_path_finding.h"
/* Function definitions for XML parsing */
#include <argos3/core/utility/configuration/argos_configuration.h>
/* 2D vector definition */
#include <argos3/core/utility/math/vector2.h>

/****************************************/
/****************************************/



CObstacleAvoidance::CObstacleAvoidance() :
   m_pcLEDs(NULL),
   m_pcCamera(NULL),
   m_pcWheels(NULL),
   m_pcProximity(NULL),
   m_pcLight(NULL),
   m_cAlpha(10.0f),
   m_fDelta(0.5f),
   m_fWheelVelocity(2.5f),
   m_cGoStraightAngleRange(-ToRadians(m_cAlpha),
                           ToRadians(m_cAlpha)) {


	numberOfSteps = 0;
	otnn = new ObjectTrackingNeuralNetwork;
	previous_state[0] = 0;
	previous_state[1] = 0;
	previous_current[0] = 0;
	previous_current[1] = 0;
	turn = 1;
	hasTurn = false;

}

/****************************************/
/****************************************/

void CObstacleAvoidance::Init(TConfigurationNode& t_node) {



   /*
    * Get sensor/actuator handles
    *
    * The passed string (ex. "differential_steering") corresponds to the
    * XML tag of the device whose handle we want to have. For a list of
    * allowed values, type at the command prompt:
    *
    * $ argos3 -q actuators
    *
    * to have a list of all the possible actuators, or
    *
    * $ argos3 -q sensors
    *
    * to have a list of all the possible sensors.
    *
    * NOTE: ARGoS creates and initializes actuators and sensors
    * internally, on the basis of the lists provided the configuration
    * file at the <controllers><footbot_diffusion><actuators> and
    * <controllers><footbot_diffusion><sensors> sections. If you forgot to
    * list a device in the XML and then you request it here, an error
    * occurs.
    */


   m_pcLEDs   = GetActuator<CCI_LEDsActuator                          >("leds");
   m_pcCamera = GetSensor  <CCI_ColoredBlobOmnidirectionalCameraSensor>("colored_blob_omnidirectional_camera");


   m_pcWheels    = GetActuator<CCI_DifferentialSteeringActuator		  >("differential_steering");
   m_pcProximity = GetSensor  <CCI_FootBotProximitySensor             >("footbot_proximity"    );
   m_pcPosSens   = GetSensor  <CCI_PositioningSensor                  >("positioning"       );
   m_pcLight  	 = GetSensor  <CCI_FootBotLightSensor                 >("footbot_light");

   /*
    * Parse the configuration file
    *
    * The user defines this part. Here, the algorithm accepts three
    * parameters and it's nice to put them in the config file so we don't
    * have to recompile if we want to try other settings.
    */
   GetNodeAttributeOrDefault(t_node, "alpha", m_cAlpha, m_cAlpha);
   m_cGoStraightAngleRange.Set(-ToRadians(m_cAlpha), ToRadians(m_cAlpha));
   GetNodeAttributeOrDefault(t_node, "delta", m_fDelta, m_fDelta);
   GetNodeAttributeOrDefault(t_node, "velocity", m_fWheelVelocity, m_fWheelVelocity);

   /* Switch the camera on */
   m_pcCamera->Enable();


   //python


   /*Py_Initialize();
   char *args[1] = {"/home/jkoroveshi/eclipse-workspace/PythonEmbedding/Debug/PythonEmbedding"};
   PySys_SetArgv(1,args); // must call this to get sys.argv and relative imports


   PyRun_SimpleString("import os, sys\n"
      				  "print sys.argv\n"
      				  "print sys.argv, \"\\n\".join(sys.path)\n"
      			      "print os.getcwd()\n"
      			      "import numpy\n" // import a relative module
      			     );*/


   //pName = PyString_FromString("clasifyNumbers");
   //pModuleTensorFlow = PyImport_Import(pName);
   //Py_DECREF(pName);
}

/****************************************/
/****************************************/
void CObstacleAvoidance::setTurn(int t)
{
	turn = t;
}
void CObstacleAvoidance::setHasTurn(bool t)
{
	hasTurn = t;
}
int CObstacleAvoidance::getTurn()
{
	return turn;
}
void CObstacleAvoidance::ControlStep() {

	//lexohen te dhenat e sensorit te drites
	/* Get light readings */
	//const CCI_FootBotLightSensor::TReadings& tReadings = m_pcLight->GetReadings();
	/* Calculate a normalized vector that points to the closest light */
	//CVector2 cAccum;
	//for(size_t i = 0; i < tReadings.size(); ++i) {
	  // cAccum += CVector2(tReadings[i].Value, tReadings[i].Angle);
	//}
	//CRadians cAngle = cAccum.Angle();
	//Real cValue = cAccum.Length();
	//std::cout << "gjetesia rezultat "<<cValue<<std::endl;
	//std::cout << "kendi rezultat "<<cAngle<<std::endl;

	//lexojme nga sensoret gjendjen aktuale
	double current_state[2];
	GetCurrentState(current_state);
	//std::cout<<"current ne controller"<<current_state[0]<<" "<<current_state[1]<<std::endl;

	if(hasTurn == false)//hera e pare e episodit
	{
		previous_state[0] = 0;
		previous_state[1] = 0;
		previous_current[0] = current_state[0];
		previous_current[1] = current_state[1];
		hasTurn = true;
		std::cout<<"nuk eshhte radha previous state"<<previous_state[0]<<" "<<previous_state[1]<<std::endl;
		std::cout<<"nuk eshhte radha previous current state"<<previous_current[0]<<" "<<previous_current[1]<<std::endl;

		return;
	}
	else//fillon nga hera e dyte
	{
		if((turn % 2) == 1)//nqs nuk eshte radha e tij, resetojme vlerat
		{
			previous_state[0] = previous_current[0];
			previous_state[1] = previous_current[1];
			previous_current[0] = current_state[0];
			previous_current[1] = current_state[1];
			std::cout<<"eshhte radha previous state"<<previous_state[0]<<" "<<previous_state[1]<<std::endl;
			std::cout<<"eshhte radha previous current state"<<previous_current[0]<<" "<<previous_current[1]<<std::endl;

		}
		else//radha e tij, do zgjedhe veprimin
		{
			double previous_state1[2];
			    GetPreviousState(previous_state1);
			    //std::cout<<"previous ne controller"<<previous_state1[0]<<" "<<previous_state1[1]<<std::endl;

			    double state[4];
			    state[0] = previous_state1[0];
			    state[1] = previous_state1[1];
			    state[2] = previous_current[0];
			    state[3] = previous_current[1];


			    //std::cout<<"exec f2"<<std::endl;
			    int action = otnn->callMethodAct(state,4);
			    last_action = action;

			    //0 majtas
			    //1 djathtas
			    //2 drejt
			    //4 no action

			    if(action == 0)
			    	m_pcWheels->SetLinearVelocity(0, m_fWheelVelocity);
			    else if(action == 1)
			    	m_pcWheels->SetLinearVelocity(m_fWheelVelocity, 0);
			    else if(action == 2)
			    	m_pcWheels->SetLinearVelocity(m_fWheelVelocity, m_fWheelVelocity);
			    else m_pcWheels->SetLinearVelocity(0, 0);

			   numberOfSteps ++;



		}

	}



}

void CObstacleAvoidance::Reset() {
 //std::cout<<"reseting"<<std::endl;

}

bool CObstacleAvoidance::IsEpisodeFinished()
{
	//std::cout<<"controlling is episode finished numberOfSteps "<<numberOfSteps<<std::endl;
	if(numberOfSteps == 0)
		return false;

	//proximity sensor
	const CCI_FootBotProximitySensor::TReadings& tProxReads = m_pcProximity->GetReadings();
	/* Sum them together */
	CVector2 cAccumulator;
	for(size_t i = 0; i < tProxReads.size(); ++i)
	{
	    cAccumulator += CVector2(tProxReads[i].Value, tProxReads[i].Angle);
	}
	cAccumulator /= tProxReads.size();
	double cAngle = cAccumulator.Angle().GetValue();
	Real cValue = cAccumulator.Length();


	//camera sensor
	/* Get led color of nearby robots */
	const CCI_ColoredBlobOmnidirectionalCameraSensor::SReadings& sBlobs = m_pcCamera->GetReadings();
	/*
	* Check whether someone sent a 1, which means 'flash'
	*/
	double distanca =0;
	bool bSomeoneFlashed = false;
	CVector2 cAccum;
	for(size_t i = 0; i < sBlobs.BlobList.size(); ++i) {
		bSomeoneFlashed = (sBlobs.BlobList[i]->Color == CColor::RED);
		cAccum += CVector2(sBlobs.BlobList[i]->Distance,sBlobs.BlobList[i]->Angle);
	}
	cAccum /= sBlobs.BlobList.size();

	if(bSomeoneFlashed == false)
		return false;

	distanca = cAccum.Length();

	double distanca_previous = previous_current[0];
	//std::cout << "distanca  "<<distanca<<std::endl;
	//std::cout << "distanca previous "<<distanca_previous<<std::endl;

	double kendi = cAccum.Angle().GetValue();
	double previous_angle = previous_current[1];
	//std::cout << "kendi  "<<kendi<<std::endl;
	//std::cout << "kendi previous "<<previous_angle<<std::endl;


	//if(cAccum.Length() > 35 || cAccum.Length() < 18)
	//return true;
	//else return false;

	//double dif_distanca = distanca - distanca_previous;
	//if(dif_distanca < 0) dif_distanca = (-1)* dif_distanca;

	//double dif_kendi = kendi - previous_angle;
	//if(dif_kendi < 0) dif_kendi = (-1) * dif_kendi;
	//std::cout << "kendi  "<<dif_kendi<<std::endl;
	//std::cout << "distanca  "<<dif_distanca<<std::endl;

	if(cValue > distanca_previous || cAngle > previous_angle)
		return true;
	else return false;

	if(distanca >= 2* distanca_previous && kendi >= 2*previous_angle)
		return true;
	else return false;

	if(cAccum.Length() > 35 || cAccum.Length() < 18)
			return true;
	else return false;


	if(kendi < 0 && previous_angle < 0 )
	{

		kendi = kendi * -1;
		previous_angle = previous_angle * -1;
		//std::cout << "kendi negativ  "<<kendi<<std::endl;
		//std::cout << "kendi previous negativ "<<previous_angle<<std::endl;
		if(kendi > 2 * previous_angle)
			{
			std::cout << "mbaron eisodiiiidd "<<std::endl;
			return true;
			}
		else
		{
			return false;
		}
	}
	else if(kendi < 0 && previous_angle > 0)
	{
		std::cout << "2 "<<std::endl;
		return false;
	}
	else if(kendi > 0 && previous_angle < 0)
	{
		std::cout << "3 "<<std::endl;
		return false;
	}
	else if (kendi > 0 && previous_angle > 0)
	{
		std::cout << "3 "<<std::endl;
		if(kendi > 2 * previous_angle)
			return true;
		else return false;
	}
	else return false;

	std::cout << "4 "<<std::endl;
	if(cAccum.Length() < 35 && cAccum.Length() > 18 && bSomeoneFlashed)
		return false;
	else return true;

}

int CObstacleAvoidance::GetReward()
{
	//lexojme nga proximity sensor
		/* Get readings from proximity sensor */
		   const CCI_FootBotProximitySensor::TReadings& tProxReads = m_pcProximity->GetReadings();
		   /* Sum them together */
		   CVector2 cAccumulator;
		   for(size_t i = 0; i < tProxReads.size(); ++i) {
		      cAccumulator += CVector2(tProxReads[i].Value, tProxReads[i].Angle);
		   }
		   cAccumulator /= tProxReads.size();





	//lexohen te dhenat nga sensori i kameras

	/* Get led color of nearby robots */
	const CCI_ColoredBlobOmnidirectionalCameraSensor::SReadings& sBlobs = m_pcCamera->GetReadings();
	/*
	* Check whether someone sent a 1, which means 'flash'
	*/
	Real distanca =0;
	bool bSomeoneFlashed = false;
	CVector2 cAccum;
	for(size_t i = 0; i < sBlobs.BlobList.size(); ++i) {
		bSomeoneFlashed = (sBlobs.BlobList[i]->Color == CColor::RED);
		//std::cout << "distanca "<<sBlobs.BlobList[i]->Distance<<std::endl;
		//std::cout << "kendi "<<sBlobs.BlobList[i]->Angle<<std::endl;
		//distanca = sBlobs.BlobList[i]->Distance;
		cAccum += CVector2(sBlobs.BlobList[i]->Distance,sBlobs.BlobList[i]->Angle);
	}

	cAccum /= sBlobs.BlobList.size();

	distanca = cAccumulator.Length();//gjatesia cm
	double kendi = cAccumulator.Angle().GetValue();

	//distanca paraardhese
	double previous_distance = previous_current[0];
	double previous_angle = previous_current[1];

	//std::cout << "get reward gjatesia "<<distanca<<std::endl;
	//if(distanca > 35 || distanca < 18)
	//return -100;

	//double dif_distanca = distanca - previous_distance;
	//if(dif_distanca < 0) dif_distanca = (-1)* dif_distanca;

	//double dif_kendi = kendi - previous_angle;
	//if(dif_kendi < 0) dif_kendi = (-1) * dif_kendi;

	if(distanca > previous_distance || kendi > previous_angle)
		return -10;
	else return 1;

	//std::cout << "get reward gjatesia "<<distanca<<std::endl;
	if(distanca > 35 || distanca < 18)
		return -100;





	    if(kendi < 0 && previous_angle < 0 )
		{
			kendi = kendi * -1;
			previous_angle = previous_angle * -1;

			if(kendi >2 * previous_angle) return -10;
			else return 1;
		}
		else if(kendi < 0 && previous_angle > 0)
		{
			return -10;
		}
		else if(kendi > 0 && previous_angle < 0)
		{
			return -10;
		}
		else if (kendi > 0 && previous_angle > 0)
		{
			if(kendi > 2* previous_angle)
				return -10;
			else return 1;
		}
		else return 1;




	if(kendi <= previous_angle) return 1;
	else return -10;

}


void CObstacleAvoidance::Remember(double current_state[], int action, double new_state[], int reward, bool done)
{
   otnn->callMethodRemember(current_state,action,reward,new_state,done,4);
}

void CObstacleAvoidance::WriteToFile(int episodeLength[], int dimension)
{
   otnn->callMethodSaveTeFile(episodeLength, dimension);
}

void CObstacleAvoidance::SaveModel()
{
   otnn->callMethodSaveModel("file");
}

void CObstacleAvoidance::GetCurrentState(double state[])
{
	//lexojme nga proximity sensor
	/* Get readings from proximity sensor */
	   //const CCI_FootBotProximitySensor::TReadings& tProxReads = m_pcProximity->GetReadings();
	   /* Sum them together */
	   CVector2 cAccumulator;
	   //for(size_t i = 0; i < tProxReads.size(); ++i) {
	   //   cAccumulator += CVector2(tProxReads[i].Value, tProxReads[i].Angle);
	   //}
	   //cAccumulator /= tProxReads.size();
	   //state[0] = cAccumulator.Length();//gjatesia cm
	   //state[1] = cAccumulator.Angle().GetValue();//kendi radian




	//camera sensor
	/* Get led color of nearby robots */

	//sBlobs ka 11 lexime, te perbera nga gjatesia dhe kendi
	//qe tregojne pozicionin ku eshte pare drita
	const CCI_ColoredBlobOmnidirectionalCameraSensor::SReadings& sBlobs = m_pcCamera->GetReadings();
	/*
	* Check whether someone sent a 1, which means 'flash'
	*/
	Real distanca =0;
	bool bSomeoneFlashed = false;
	CVector2 cAccum;
	//std::cout << "getting current state "<<std::endl;
	//std::cout << "bloblist size  "<<sBlobs.BlobList.size()<<std::endl;

	for(size_t i = 0; i < sBlobs.BlobList.size(); ++i) {
		bSomeoneFlashed = (sBlobs.BlobList[i]->Color == CColor::RED);
		//std::cout << "ngjyra  "<<sBlobs.BlobList[i]->Color<<std::endl;
		//std::cout << "flashed  "<<bSomeoneFlashed<<std::endl;
		//std::cout << "distanca  "<<sBlobs.BlobList[i]->Distance<<std::endl;
		//std::cout << "kendi  "<<sBlobs.BlobList[i]->Angle<<std::endl;
		cAccum += CVector2(sBlobs.BlobList[i]->Distance,sBlobs.BlobList[i]->Angle);
	}
	//std::cout << "pas forit "<<std::endl;
	//std::cout << "pas forit gjatesia  "<<cAccum.Length()<<std::endl;
	//std::cout << "pas forit kendi  "<<cAccum.Angle().GetValue()<<std::endl;

	cAccum /= sBlobs.BlobList.size();

	//std::cout << "pas pjesetimi "<<std::endl;
	//std::cout << "pas pjesetimit gjatesia  "<<cAccum.Length()<<std::endl;
	//std::cout << "pas pjesetimit kendi  "<<cAccum.Angle().GetValue()<<std::endl;

	state[0] = cAccum.Length();//gjatesia cm
	state[1] = cAccum.Angle().GetValue();//kendi radian

}

void CObstacleAvoidance::GetPreviousState(double state[])
{
	state[0] = previous_state[0];//gjatesia
	state[1] = previous_state[1];//kendi
}

void CObstacleAvoidance::GetPreviousCurrent(double state[])
{
	state[0] = previous_current[0];//gjatesia
	state[1] = previous_current[1];//kendi
}

void CObstacleAvoidance::SetPreviousState(double state[])
{
    previous_state[0] = state[0];//gjatesia
	previous_state[1] = state[1];//kendi
}

void CObstacleAvoidance::Replay()
{
   otnn->callMethodReplay();
}

void CObstacleAvoidance::TargetTrain()
{
   otnn->callMethodTargetTrain();
}

int CObstacleAvoidance::GetLastAction()
{
	return last_action;
}
/****************************************/
/****************************************/


/*
 * This statement notifies ARGoS of the existence of the controller.
 * It binds the class passed as first argument to the string passed as
 * second argument.
 * The string is then usable in the configuration file to refer to this
 * controller.
 * When ARGoS reads that string in the configuration file, it knows which
 * controller class to instantiate.
 * See also the configuration files for an example of how this is used.
 */
REGISTER_CONTROLLER(CObstacleAvoidance, "obstacle_avoidance_controller")
