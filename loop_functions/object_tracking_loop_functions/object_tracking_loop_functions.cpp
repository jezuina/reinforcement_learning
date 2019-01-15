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
   m_pcController2(NULL){

	previous_state[0] = 0;
	previous_state[1] = 0;
	previous_state[2] = 0;
	previous_state[3] = 0;

}




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
	   cFBPos.Set(0,0.2, 0.0f);
	   CRadians r = CRadians(0);
	   cFBRot.FromAngleAxis(r, CVector3::Z);
	   bool bDone = MoveEntity(m_pcFootBot1->GetEmbodiedEntity(), cFBPos, cFBRot);

	   cFBPos.Set(0, -0.1, 0.0f);
	   bDone = MoveEntity(m_pcFootBot2->GetEmbodiedEntity(), cFBPos, cFBRot);

}

/****************************************/
/****************************************/

void CObjectTrackingLoopFunctions::Reset() {

   std::cout<<"inside reseting "<<std::endl;
   //merret lista e te gjithe entiteteve te tipit foot-bot
   bool exists = false;
   		CSpace::TMapPerType& m_cFootbots = GetSpace().GetEntitiesByType("foot-bot");
   	    std::cout<<"robots number "<<m_cFootbots.size()<<std::endl;
   			 for(CSpace::TMapPerType::iterator it = m_cFootbots.begin();it != m_cFootbots.end(); ++it)
   			 {
   				/* Get handle to foot-bot entity and controller */
   				CFootBotEntity& cFootBot = *any_cast<CFootBotEntity*>(it->second);
   				std::string id = cFootBot.GetId();

   				if(id == "fb_0")
   				{
   					exists = true;
   					break;
   				}

   			 }


   if(exists)
   {
	   std::cout<<"remove entity "<<std::endl;
	   RemoveEntity(*m_pcFootBot1);
   }

   /* Create a RNG (it is automatically disposed of by ARGoS) */
   CRandom::CRNG* pcRNG = CRandom::CreateRNG("argos");
   CVector3 cFBPos;
   CQuaternion cFBRot;

   /* Choose a random position */
   cFBPos.Set(0, 0.2, 0.0f);
   CRadians r = CRadians(0);
   cFBRot.FromAngleAxis(r, CVector3::Z);
   std::cout<<"adding entity2 "<<std::endl;
   if(exists == false)
   {

	   //krijohet roboti1
	   	m_pcFootBot1 = new CFootBotEntity(
	   	      "fb_0",    // entity id
	   	      "fdc1"    // controller id as set in the XML
	   	      );
	   	   AddEntity(*m_pcFootBot1);
	   	 bool bDone = MoveEntity(m_pcFootBot1->GetEmbodiedEntity(), cFBPos, cFBRot);
   }

   cFBPos.Set(0, -0.1, 0.0f);
   bool bDone = MoveEntity(m_pcFootBot2->GetEmbodiedEntity(), cFBPos, cFBRot);


   trials_done ++;
   trial_steps_done = 0;
}

/****************************************/
/****************************************/

void CObjectTrackingLoopFunctions::PostStep() {

	 argos::LOG << "steps "<< trial_steps_done << std::endl;

	//duhen mare te dhenat mbi gjendjen e re, dhe shperblimi
	//keto duhet te perdoren per te perditesuar vleren e state-value

	bool isEpisodeFInished = false;
	std::cout<<"post step"<<std::endl;

	    //merret lista e te gjithe entiteteve te tipit foot-bot
		CSpace::TMapPerType& m_cFootbots = GetSpace().GetEntitiesByType("foot-bot");
		if(m_cFootbots.size() == 1) Reset();
	    //std::cout<<"robots number "<<m_cFootbots.size()<<std::endl;
			 for(CSpace::TMapPerType::iterator it = m_cFootbots.begin();it != m_cFootbots.end(); ++it)
			 {

				 /* Get handle to foot-bot entity and controller */
				  CFootBotEntity& cFootBot = *any_cast<CFootBotEntity*>(it->second);

				  std::string id = cFootBot.GetId();
				  std::cout<<"id "<<id<<std::endl;
				  if(id == "fb_1")//ky eshte foot-boti qe po trajnohet
				  {
					  CObstacleAvoidance& cController = dynamic_cast<CObstacleAvoidance&>(cFootBot.GetControllableEntity().GetController());
					  //merret new state, reward dhe done
					   int reward = cController.GetReward();
					   bool done = cController.IsEpisodeFinished();
					   int action = cController.GetLastAction();

					   //nga foot boti merren leximet per dy gjendjen e fundit
					   double prev_state_bot[2];
					   double curr_state_bot[2];

					   cController.GetPreviousState(prev_state_bot);
					   cController.GetCurrentState(curr_state_bot);

					   double new_experiment_state[4];
					   new_experiment_state[0] = prev_state_bot[0];
					   new_experiment_state[1] = prev_state_bot[1];
					   new_experiment_state[2] = curr_state_bot[0];
					   new_experiment_state[3] = curr_state_bot[1];

					  cController.Remember(previous_state, action, new_experiment_state, reward, done);
					  cController.Replay();
					  cController.TargetTrain();

					  cController.SetPreviousState(curr_state_bot);

					  previous_state[0] = new_experiment_state[0];
					  previous_state[1] = new_experiment_state[1];
					  previous_state[2] = new_experiment_state[2];
					  previous_state[3] = new_experiment_state[3];

					  if(cController.IsEpisodeFinished() == true)
					  {

						 	   std::cout<<"reseting "<<id<<std::endl;
						 	   Reset();
						 	   break;
					  }

				  }
				 // std::cout<<"robot id "<<id<<std::endl;

			 }


	 //trial_steps_done: bejme reset nqs arrihet numri i hereve qe zhvillohet nje episod
	 trial_steps_done ++;

	 if(trial_steps_done == trial_length)
	 {
		 Reset();
	 }
}

bool CObjectTrackingLoopFunctions::IsExperimentFinished()
{
	if(trials_done == number_of_trials) return true;
	return false;
}
/****************************************/
/****************************************/

REGISTER_LOOP_FUNCTIONS(CObjectTrackingLoopFunctions, "object_tracking_loop_functions")


