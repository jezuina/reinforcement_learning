/*
 * object_tracking_loop_functions.h
 *
 *  Created on: Sep 1, 2018
 *      Author: jkoroveshi
 */

#ifndef OBJECT_TRACKING_LOOP_FUNCTIONS_H
#define TRAJECTORY_LOOP_FUNCTIONS_H

#include <argos3/core/simulator/loop_functions.h>
#include <argos3/plugins/robots/foot-bot/simulator/footbot_entity.h>
/* The NN controller */
#include <controllers/footbot_nn/footbot_nn_controller.h>

using namespace argos;

class CObjectTrackingLoopFunctions : public CLoopFunctions {


public:

   CObjectTrackingLoopFunctions();

   virtual ~CObjectTrackingLoopFunctions() {}

   virtual void Init(TConfigurationNode& t_tree);

   virtual void Reset();

   virtual void PostStep();

   virtual bool IsExperimentFinished();


private:

   CFootBotEntity* m_pcFootBot1;
   CFootBotNNController* m_pcController1;
   CFootBotEntity* m_pcFootBot2;
   CFootBotNNController* m_pcController2;
   int number_of_trials = 1000;
   int trial_length = 500;
   int trials_done = 0;
   int trial_steps_done = 0;

};

#endif
