//============================================================================
// Name        : PythonEmbedding.cpp
// Author      : jezuina
// Version     :
// Copyright   : Your copyright notice
// Description : Hello World in C++, Ansi-style
//============================================================================
#include </usr/include/python2.7/Python.h>
#include <iostream>
#include <vector>
#include "ObjectTrackingNeuralNetwork.h"

using namespace std;

ObjectTrackingNeuralNetwork::ObjectTrackingNeuralNetwork()
    {
	   Py_Initialize();
	   char *args[1] = {"/home/jkoroveshi/eclipse-workspace/PythonEmbedding/Debug/PythonEmbedding"};
	   PySys_SetArgv(1,args); // must call this to get sys.argv and relative imports


	   /*PyRun_SimpleString("import os, sys\n"
						  "print sys.argv\n"
						  "print sys.argv, \"\\n\".join(sys.path)\n"
						  "print os.getcwd()\n"
						  "import numpy\n" // import a relative module
						 );*/

		module = PyImport_ImportModule("ObjectTrackingNeuralNetwork");
		assert(module != NULL);

		klasa = PyObject_GetAttrString(module, "OTNeuralNetwork");
		assert(klasa != NULL);

		instance = PyInstance_New(klasa, NULL, NULL);
		assert(instance != NULL);
    }

void ObjectTrackingNeuralNetwork::callMethodRemember(const double state[], int action, int reward, const double new_state[], bool done, int state_dimension )
	{
		PyObject *pValue;
		//Transfer the C++ state vector to a python tuple
		PyObject *pArgsState = PyTuple_New(state_dimension);
		   for (int i = 0; i < state_dimension; ++i)
		   {
			   pValue = PyFloat_FromDouble(state[i]);
			   if (!pValue)
			   {
				 fprintf(stderr, "Cannot convert argument\n");
			   }
		    /* pValue reference stolen here: */
		    PyTuple_SetItem(pArgsState, i, pValue);
		   }

		 //Transfer the C++ new_state vector to a python tuple
		 PyObject *pArgsNewState = PyTuple_New(state_dimension);
		   for (int i = 0; i < state_dimension; ++i)
		   {
			   pValue = PyFloat_FromDouble(new_state[i]);
			   if (!pValue)
			   {
				  fprintf(stderr, "Cannot convert argument\n");
			   }
		   /* pValue reference stolen here: */
		   PyTuple_SetItem(pArgsNewState, i, pValue);
		  }

		  PyObject* remember = PyObject_CallMethod(instance, "remember", "(OiiOb)", pArgsState, action, reward, pArgsNewState, done);
		  assert(remember != NULL);

	}


	int ObjectTrackingNeuralNetwork::callMethodAct(const double state[], int state_dimension)
	{
		PyObject *pValue;
		//Transfer the C++ state vector to a python tuple
		PyObject *pArgsState = PyTuple_New(state_dimension);
		   for (int i = 0; i < state_dimension; ++i)
		   {
			   pValue = PyFloat_FromDouble(state[i]);
			   if (!pValue)
			   {
				 fprintf(stderr, "Cannot convert argument\n");
			   }
			/* pValue reference stolen here: */
			PyTuple_SetItem(pArgsState, i, pValue);
		   }

		   PyObject* act = PyObject_CallMethod(instance, "act", "(O)", pArgsState);
		   assert(act != NULL);

		   int r =  PyInt_AsLong(act);
		   return r;
	}

	void ObjectTrackingNeuralNetwork::callMethodReplay()
	{

	   PyObject* replay = PyObject_CallMethod(instance, "replay", "()");
	   assert(replay != NULL);

	}

	void ObjectTrackingNeuralNetwork::callMethodTargetTrain()
	{

	   PyObject* targetTrain = PyObject_CallMethod(instance, "target_train", "()");
	   assert(targetTrain != NULL);

	}

	void ObjectTrackingNeuralNetwork::callMethodSaveModel(string fileName)
	{

	   PyObject* saveModel = PyObject_CallMethod(instance, "save_model", "(s)", "file.model");
	   assert(saveModel != NULL);

	}
