/*
 * ObjectTrackingNeuralNetwork.h
 *
 *  Created on: Jan 4, 2019
 *      Author: jkoroveshi
 */

#ifndef OBJECTTRACKINGNEURALNETWORK_H_
#define OBJECTTRACKINGNEURALNETWORK_H_

#include </usr/include/python2.7/Python.h>
#include <iostream>
#include <vector>
using namespace std;

class ObjectTrackingNeuralNetwork
{
private:
	PyObject* module;
	PyObject* klasa;
	PyObject* instance;

public:

	ObjectTrackingNeuralNetwork();
	void callMethodRemember(const double[], int, int, const double[], bool, int);
	int callMethodAct(const double[], int);
	void callMethodReplay();
	void callMethodTargetTrain();
	void callMethodSaveModel(string);

};

#endif /* OBJECTTRACKINGNEURALNETWORK_H_ */
