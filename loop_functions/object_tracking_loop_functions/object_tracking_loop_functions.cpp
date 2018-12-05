/*
 * object_tracking_loop_functions.cpp
 *
 *  Created on: Sep 1, 2018
 *      Author: jkoroveshi
 */

#include <argos3/core/simulator/loop_functions.h>
#include "object_tracking_loop_functions.h"
#include <argos3/core/simulator/simulator.h>
#include <argos3/core/utility/configuration/argos_configuration.h>
#include <argos3/plugins/robots/foot-bot/simulator/footbot_entity.h>
#include <controllers/obstacle_avoidance_path_finding/obstacle_avoidance_path_finding.h>
#include <controllers/footbot_diffusion/footbot_diffusion.h>
/****************************************/
/****************************************/

/*
 * To reduce the number of waypoints stored in memory,
 * consider two robot positions distinct if they are
 * at least MIN_DISTANCE away from each other
 * This constant is expressed in meters
 */
static const Real MIN_DISTANCE = 0.05f;
/* Convenience constant to avoid calculating the square root in PostStep() */
static const Real MIN_DISTANCE_SQUARED = MIN_DISTANCE * MIN_DISTANCE;

/****************************************/
/****************************************/

CObjectTrackingLoopFunctions::CObjectTrackingLoopFunctions() :
   m_pcFootBot1(NULL),
   m_pcController1(NULL),
   m_pcFootBot2(NULL),
   m_pcController2(NULL){}




void CObjectTrackingLoopFunctions::Init(TConfigurationNode& t_tree) {

	//krijohet roboti1
	m_pcFootBot1 = new CFootBotEntity(
	      "fb_0",    // entity id
	      "fdc1"    // controller id as set in the XML
	      );
	   AddEntity(*m_pcFootBot1);

	 //krijohet roboti2
	 m_pcFootBot2 = new CFootBotEntity(
	   	   "fb_1",    // entity id
	   	   "oac"    // controller id as set in the XML
	   	      );
	   	AddEntity(*m_pcFootBot2);



	   /* Create a RNG (it is automatically disposed of by ARGoS) */
	   CRandom::CRNG* pcRNG = CRandom::CreateRNG("argos");
	   CVector3 cFBPos;
	   CQuaternion cFBRot;

	   /* Choose a random position */
	   cFBPos.Set(0, 0.2, 0.0f);
	   CRadians r = CRadians(0);
	   cFBRot.FromAngleAxis(r, CVector3::Z);
	   bool bDone = MoveEntity(m_pcFootBot1->GetEmbodiedEntity(), cFBPos, cFBRot);

	   cFBPos.Set(0, 0, 0.0f);
	   bDone = MoveEntity(m_pcFootBot2->GetEmbodiedEntity(), cFBPos, cFBRot);

}

/****************************************/
/****************************************/

void CObjectTrackingLoopFunctions::Reset() {


}

/****************************************/
/****************************************/

void CObjectTrackingLoopFunctions::PostStep() {

	//duhen mare te dhenat mbi gjendjen e re, dhe shperblimi
	//keto duhet te perdoren per te perditesuar vleren e state-value

	bool isEpisodeFInished = false;
	std::cout<<"post step"<<std::endl;

	    //merret lista e te gjithe entiteteve te tipit foot-bot
		CSpace::TMapPerType& m_cFootbots = GetSpace().GetEntitiesByType("foot-bot");
	    //std::cout<<"robots number "<<m_cFootbots.size()<<std::endl;
			 for(CSpace::TMapPerType::iterator it = m_cFootbots.begin();it != m_cFootbots.end(); ++it)
			 {

				 /* Get handle to foot-bot entity and controller */
				  CFootBotEntity& cFootBot = *any_cast<CFootBotEntity*>(it->second);

				  std::string id = cFootBot.GetId();
				  if(id == "fb_1")//ky eshte foot-boti qe po trajnohet
				  {
					  CObstacleAvoidance& cController = dynamic_cast<CObstacleAvoidance&>(cFootBot.GetControllableEntity().GetController());

					  if(cController.IsEpisodeFinished() == true)
					  {

						  /* Create a RNG (it is automatically disposed of by ARGoS) */
						 	   CRandom::CRNG* pcRNG = CRandom::CreateRNG("argos");
						 	   CVector3 cFBPos;
						 	   CQuaternion cFBRot;

						 	   /* Choose a random position */
						 	   cFBPos.Set(0, 0.2, 0.0f);
						 	   CRadians r = CRadians(0);
						 	   cFBRot.FromAngleAxis(r, CVector3::Z);
						 	   bool bDone = MoveEntity(m_pcFootBot1->GetEmbodiedEntity(), cFBPos, cFBRot);

						 	   cFBPos.Set(0, 0, 0.0f);
						 	   bDone = MoveEntity(m_pcFootBot2->GetEmbodiedEntity(), cFBPos, cFBRot);
					  }

				  }
				 // std::cout<<"robot id "<<id<<std::endl;

			 }




}

bool CObjectTrackingLoopFunctions::IsExperimentFinished()
{
	return false;
}
/****************************************/
/****************************************/

REGISTER_LOOP_FUNCTIONS(CObjectTrackingLoopFunctions, "object_tracking_loop_functions")


