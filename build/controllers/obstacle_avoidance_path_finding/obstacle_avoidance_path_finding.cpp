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


   Py_Initialize();
   char *args[1] = {"/home/jkoroveshi/eclipse-workspace/PythonEmbedding/Debug/PythonEmbedding"};
   PySys_SetArgv(1,args); // must call this to get sys.argv and relative imports


   PyRun_SimpleString("import os, sys\n"
      				  "print sys.argv\n"
      				  "print sys.argv, \"\\n\".join(sys.path)\n"
      			      "print os.getcwd()\n"
      			      "import numpy\n" // import a relative module
      			     );


   //pName = PyString_FromString("clasifyNumbers");
   //pModuleTensorFlow = PyImport_Import(pName);
   //Py_DECREF(pName);
}

/****************************************/
/****************************************/

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


	/* Get led color of nearby robots */
	const CCI_ColoredBlobOmnidirectionalCameraSensor::SReadings& sBlobs = m_pcCamera->GetReadings();
	/*
	* Check whether someone sent a 1, which means 'flash'
	*/
	//std::cout<<"size "<<sBlobs.BlobList.size()<<std::endl;
	//std::cout<<"color red "<<CColor::RED<<std::endl;
	bool bSomeoneFlashed = false;
	CVector2 cAccum;
	for(size_t i = 0; ! bSomeoneFlashed && i < sBlobs.BlobList.size(); ++i) {
		//std::cout<<"color "<<sBlobs.BlobList[i]->Color<<std::endl;
	    bSomeoneFlashed = (sBlobs.BlobList[i]->Color == CColor::RED);
	    //std::cout << "distanca "<<sBlobs.BlobList[i]->Distance<<std::endl;
	    //std::cout << "kendi "<<sBlobs.BlobList[i]->Angle<<std::endl;
		cAccum = CVector2(sBlobs.BlobList[i]->Distance,sBlobs.BlobList[i]->Angle);

		if(bSomeoneFlashed) break;
	}
    if(bSomeoneFlashed)
    {
    	std::cout<<"flashed "<<std::endl;

    }
    CRadians cAngle = cAccum.Angle();
    Real cValue = cAccum.Length();
    std::cout << "gjetesia rezultat camera7777"<<cValue<<std::endl;
    std::cout << "kendi rezultat camera"<<cAngle<<std::endl;



    //Py_Finalize();

	std::string robotId = this->GetId();
	//std::cout<<"id "<<robotId<<std::endl;
   /* Get readings from proximity sensor */
   const CCI_FootBotProximitySensor::TReadings& tProxReads = m_pcProximity->GetReadings();
   /* Sum them together */
   CVector2 cAccumulatorProximity;
   for(size_t i = 0; i < tProxReads.size(); ++i) {
	 cAccumulatorProximity += CVector2(tProxReads[i].Value, tProxReads[i].Angle);
     if(tProxReads[i].Value != 0)
     {
      //std::cout<<"i "<<i<<std::endl;
      //std::cout << "vlera "<<tProxReads[i].Value<<std::endl;
      //std::cout << "kendi "<<tProxReads[i].Angle<<std::endl;
     }
   }
   cAccumulatorProximity /= tProxReads.size();
   /* If the angle of the vector is small enough and the closest obstacle
    * is far enough, continue going straight, otherwise curve a little
    */
   CRadians cAngleProximity = cAccumulatorProximity.Angle();
   Real cValueProximity = cAccumulatorProximity.Length();
   std::cout << "gjetesia rezultat proximity1111"<<cValueProximity<<std::endl;


   //if(cValue != 0)
   //{
   //std::cout << "kendi rezultat"<<cAngle<<std::endl;
   //std::cout << "gjetesia rezultat "<<cValue<<std::endl;
   //}
   m_pcWheels->SetLinearVelocity(m_fWheelVelocity, m_fWheelVelocity);
   //m_pcWheels->SetLinearVelocity(m_fWheelVelocity, 0.0f);
   //if(m_cGoStraightAngleRange.WithinMinBoundIncludedMaxBoundIncluded(cAngle) &&
     // cAccumulator.Length() < m_fDelta ) {
      /* Go straight */
      //m_pcWheels->SetLinearVelocity(m_fWheelVelocity, m_fWheelVelocity);
   //}
   //else {
      /* Turn, depending on the sign of the angle */
     // if(cAngle.GetValue() > 0.0f) {
       //  m_pcWheels->SetLinearVelocity(m_fWheelVelocity, 0.0f);
      //}
      //else {
        // m_pcWheels->SetLinearVelocity(0.0f, m_fWheelVelocity);
     // }
   //}

   numberOfSteps ++;

}

void CObstacleAvoidance::Reset() {
 std::cout<<"reseting"<<std::endl;

}

bool CObstacleAvoidance::IsEpisodeFinished()
{

	return false;

	//std::cout<<"number of steps "<<numberOfSteps<<std::endl;
	if(numberOfSteps == 0)
		return false;
	const CCI_FootBotProximitySensor::TReadings& tProxReads = m_pcProximity->GetReadings();
	/* Sum them together */
	CVector2 cAccumulator;
	for(size_t i = 0; i < tProxReads.size(); ++i) {
	    cAccumulator += CVector2(tProxReads[i].Value, tProxReads[i].Angle);
	 }
	 cAccumulator /= tProxReads.size();
	 /* If the angle of the vector is small enough and the closest obstacle
	 * is far enough, continue going straight, otherwise curve a little
	 */
	 CRadians cAngle = cAccumulator.Angle();
	 Real cValue = cAccumulator.Length();
     //std::cout<<"ne controller vlera eshte"<<cValue<<std::endl;

	 if(cValue == 0) return true;
	 else return false;
}

int CObstacleAvoidance::GetReward()
{
	//lexohen te dhenat nga sensori i afersise


	    /* Get led color of nearby robots */
		const CCI_ColoredBlobOmnidirectionalCameraSensor::SReadings& sBlobs = m_pcCamera->GetReadings();
		/*
		* Check whether someone sent a 1, which means 'flash'
		*/
		Real distanca =0;
		bool bSomeoneFlashed = false;
		CVector2 cAccum;
		for(size_t i = 0; ! bSomeoneFlashed && i < sBlobs.BlobList.size(); ++i) {
		    bSomeoneFlashed = (sBlobs.BlobList[i]->Color == CColor::RED);
		    std::cout << "distanca "<<sBlobs.BlobList[i]->Distance<<std::endl;
		    std::cout << "kendi "<<sBlobs.BlobList[i]->Angle<<std::endl;
		    distanca = sBlobs.BlobList[i]->Distance;
			cAccum = CVector2(sBlobs.BlobList[i]->Distance,sBlobs.BlobList[i]->Angle);
		}

		if(distanca < 20 && distanca != 0)
			return 1;
		else return 0;
}

void CObstacleAvoidance::GetState(Real state[])
{

//test
//test2
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
