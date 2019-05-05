/*
 * object_tracking_loop_functions.cpp
 *
 *  Created on: Sep 1, 2018
 *      Author: jkoroveshi
 */
#include <fstream>
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

	accumulatedReward = 0;
	turn = 1;


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
	   cFBPos.Set(0.2,0.2, 0.0f);
	   CRadians r = CRadians(0);
	   cFBRot.FromAngleAxis(r, CVector3::Z);
	   bool bDone = MoveEntity(m_pcFootBot1->GetEmbodiedEntity(), cFBPos, cFBRot);

	   cFBPos.Set(0, 0.2, 0.0f);
	   bDone = MoveEntity(m_pcFootBot2->GetEmbodiedEntity(), cFBPos, cFBRot);

}

/****************************************/
/****************************************/

void CObjectTrackingLoopFunctions::Reset() {

   //std::cout<<"inside reseting "<<std::endl;
   //merret lista e te gjithe entiteteve te tipit foot-bot
   bool exists = false;
   		CSpace::TMapPerType& m_cFootbots = GetSpace().GetEntitiesByType("foot-bot");
   	    //std::cout<<"robots number "<<m_cFootbots.size()<<std::endl;
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
	  // std::cout<<"remove entity "<<std::endl;
	  // RemoveEntity(*m_pcFootBot1);
   }




   /* Create a RNG (it is automatically disposed of by ARGoS) */
   CRandom::CRNG* pcRNG = CRandom::CreateRNG("argos");
   CVector3 cFBPos;
   CQuaternion cFBRot;

   /* Choose a random position */
   cFBPos.Set(0, -0.1, 0.0f);
   CRadians r = CRadians(0);
   cFBRot.FromAngleAxis(r, CVector3::Z);
   //std::cout<<"adding entity2 "<<std::endl;
   if(exists == false)
   {

	   //krijohet roboti1
	   //	m_pcFootBot1 = new CFootBotEntity(
	   //	      "fb_0",    // entity id
	   //	      "fdc1"    // controller id as set in the XML
	   //	      );
	   //	   AddEntity(*m_pcFootBot1);
	   //	 bool bDone = MoveEntity(m_pcFootBot1->GetEmbodiedEntity(), cFBPos, cFBRot);
   }



   float x =0;
   float y = 0;

   if((trials_done % 4) == 0) //0 0.2     0  -0.1
   {
	   x = 0;
	   y = 0;
   }
   else if((trials_done % 4) == 1)
   {
	   x = -0.3;
	   y = 0.3;
   }
   else if((trials_done % 4) == 2)
   {
	   //111111111ii
	   x = 0;
	   y = 0.3;
   }
   else
   {
	   x = 0.3;
	   y = 0.3;
   }

   cFBPos.Set(0, 0.2, 0.0f);
   //cFBPos.Set(x, y, 0.0f);
   bool bDone = MoveEntity(m_pcFootBot2->GetEmbodiedEntity(), cFBPos, cFBRot);


   cFBPos.Set(0.2, 0.2, 0.0f);
   r = CRadians(0);
   cFBRot.FromAngleAxis(r, CVector3::Z);

   //levizim footbotin 1
   bDone = MoveEntity(m_pcFootBot1->GetEmbodiedEntity(), cFBPos, cFBRot);


   std::cout<<"perfundoi prova "<< trials_done <<std::endl;
   std::ofstream outfile;

   outfile.open("results.txt", std::ios_base::app);
   if(trials_done == 0)
   {
	   outfile << "Prova\t" << "Episode\t" << "Reward" <<std::endl;

   }
   outfile << trials_done << "\t" << trial_steps_done << "\t" << accumulatedReward<<std::endl;
   outfile.close();
   episodeLength[trials_done] = trial_steps_done;
   trials_done ++;
   trial_steps_done = 0;
   accumulatedReward = 0;
   skipFirst = true;
}

/****************************************/
/****************************************/

void CObjectTrackingLoopFunctions::PostStep() {

	 //argos::LOG << "steps "<< trial_steps_done << std::endl;

	//duhen mare te dhenat mbi gjendjen e re, dhe shperblimi
	//keto duhet te perdoren per te perditesuar vleren e state-value

	bool isEpisodeFInished = false;
	//std::cout<<"post step"<<std::endl;

	    //merret lista e te gjithe entiteteve te tipit foot-bot
		CSpace::TMapPerType& m_cFootbots = GetSpace().GetEntitiesByType("foot-bot");
		if(m_cFootbots.size() == 1) Reset();
	    //std::cout<<"robots number "<<m_cFootbots.size()<<std::endl;
			 for(CSpace::TMapPerType::iterator it = m_cFootbots.begin();it != m_cFootbots.end(); ++it)
			 {

				 /* Get handle to foot-bot entity and controller */
				  CFootBotEntity& cFootBot = *any_cast<CFootBotEntity*>(it->second);

				  std::string id = cFootBot.GetId();
				  if(id == "fb_0")//ky eshte foot-boti target
				  {

				  }
				  //std::cout<<"id "<<id<<std::endl;
				  else if(id == "fb_1")//ky eshte foot-boti qe po trajnohet
				  {
					  CObstacleAvoidance& cController = dynamic_cast<CObstacleAvoidance&>(cFootBot.GetControllableEntity().GetController());


					   int turn = cController.getTurn();
					   //nqs eshte prova e pare
					   //nuk bejme asnje veprim
					   if(skipFirst == true)
					   {
						   double current_state_bot[2];
						   cController.GetCurrentState(current_state_bot);

						   previous_state[0] = current_state_bot[0];
						   previous_state[1] = current_state_bot[1];

						   turn = turn + 1;
						   cController.setTurn(turn);
						   skipFirst = false;
						   return;
					   }
					   //ka qene radha e agjentit
					   //ketu shikohet rezultati i veprimit paraardhes te targetit
					   //agjenti nuk ben gje ketu
					   //vetem ruajme gjendjen e re te targetit tek previous state
					   if((turn % 2) ==0)
					   {

					   //marim state qe ka bere act
					   //nga foot boti merren leximet per dy gjendjen e fundit
					   //double prev_state_bot[2];
					   //double previous_current[2];

					   //cController.GetPreviousState(prev_state_bot);
					   //std::cout<<"previous "<<prev_state_bot[0]<<" "<<prev_state_bot[1]<<std::endl;
					   //cController.GetPreviousCurrent(previous_current);
					   //std::cout<<"current previous"<<previous_current[0]<<" "<<previous_current[1]<<std::endl;

					   double current_state_bot[2];
					   cController.GetCurrentState(current_state_bot);

					   previous_state[0] = current_state_bot[0];
					   previous_state[1] = current_state_bot[1];
					   //previous_state[2] = previous_current[0];
					   //previous_state[3] = previous_current[1];
					   std::cout<<"post step nuk eshte radha agjent "<<"current state "<<current_state_bot[0]<<" "<<current_state_bot[1]<<std::endl;

					   turn = turn +1;
				       cController.setTurn(turn);
				    }
					//ketu ka vepruar targeti, por rezultati i tij nuk duket ketu
					//ketu shohim rezultatin e veprimit te meparshem te agjentit
					else
					{
						//marrim gjendjen e re ku ka shkuar
						double curr_state_bot[2];
						cController.GetCurrentState(curr_state_bot);
						//std::cout<<"current "<<curr_state_bot[0]<<" "<<curr_state_bot[1]<<std::endl;

						//marrim action e fundit qe eshte bere
						int action = cController.GetLastAction();

						//marrim reward, ketu shfaqet gjendja e updateuar nga veprimi paraardhes
						//reward e llogaritim nga dy gjendjen e fundit gjatesi kend
						double distanca_previous = previous_state[0];
						double kendi_previous = previous_state[1];

						double distanca_current = curr_state_bot[0];
						double kendi_current = curr_state_bot[1];

						std::cout<<"post step agjent "<<"previous state "<<distanca_previous<<" "<<kendi_previous<<std::endl;
						std::cout<<"post step agjent "<<"current state "<<distanca_current<<" "<<kendi_current<<std::endl;

						//double dif_distance = distanca_current - distanca_previous;

						//if(dif_distance < 0) dif_distance = dif_distance * (-1);

						//double dif_kend = kendi_current - kendi_previous;

						//std::cout<<"dif distanca "<<dif_distance<<" dif kendi"<<dif_kend<<std::endl;

						//if(dif_kend < 0) dif_kend = dif_kend * (-1);

						int reward;
						if(distanca_current > 22 || distanca_current < 18) reward = -10;
						else reward = +1;

						//if(dif_kend > 0.5 || dif_distance > 0.5)
						//	reward = -1;
						//else reward = +1;

						//if(distanca_current > distanca_previous || kendi_current > kendi_previous)
						//	reward = -1;
						//else reward = +1;
					    accumulatedReward += reward;

					    //llogaritim nese ka perfunduar episodi
					    //std::cout<<"distanca "<<distanca_current<<" "<<distanca_previous<<std::endl;
					    //std::cout<<"kendi "<<kendi_current<<" "<<kendi_previous<<std::endl;

					    bool done;
					    //if(dif_kend > 0.5 || dif_distance > 0.5)
					    //   done = true;
					    //else done = false;

					    if(distanca_current > 22 || distanca_current < 18) done = true;
					    else done = false;


					    //if(distanca_current > distanca_previous || kendi_current > kendi_previous)
						//	done = true;
						//else done = false;

						//bejme remember
					    //double new_experiment_state[4];
					    //new_experiment_state[0] = previous_state[2];
					    //new_experiment_state[1] = previous_state[3];
					    //new_experiment_state[2] = curr_state_bot[0];
					    //new_experiment_state[3] = curr_state_bot[1];

					    double previous_state_to_remember[2];
					    previous_state_to_remember[0] = previous_state[0];
						previous_state_to_remember[1] = previous_state[1];

						double current_state_to_remember[2];
						current_state_to_remember[0] = curr_state_bot[0];
						current_state_to_remember[1] = curr_state_bot[1];


					    cController.Remember(previous_state_to_remember, action, current_state_to_remember, reward, done);
					    cController.Replay();
					    cController.TargetTrain();

						turn = turn +1;
						cController.setTurn(turn);

						trial_steps_done ++;
						//nqs mbaroi episodi, bejme reset
						if(done  == true)
					    {
							double previous[2] ={0,0};
							cController.SetPreviousState(previous);
							cController.setHasTurn(false);
							cController.setTurn(1);
						   //std::cout<<"reseting "<<id<<std::endl;
						   Reset();
						   break;
					    }
					}
                 }
			 }

	 //trial_steps_done: bejme reset nqs arrihet numri i hereve qe zhvillohet nje episod
	 if(trial_steps_done == trial_length)
	 {
		 Reset();
	 }

}

bool CObjectTrackingLoopFunctions::IsExperimentFinished()
{
	//std::cout<<"is experiment finished "<<std::endl;
	//std::cout<<"trials done "<<trials_done<<std::endl;
	//std::cout<<"number of trials "<<number_of_trials<<std::endl;
	if(trials_done == number_of_trials)
	{

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
							  //std::cout<<"saving model "<<std::endl;
							  cController.SaveModel();
							  cController.WriteToFile(episodeLength, 200);
						  }
					 }


	 return true;
	}
	return false;
}
/****************************************/
/****************************************/

REGISTER_LOOP_FUNCTIONS(CObjectTrackingLoopFunctions, "object_tracking_loop_functions")


