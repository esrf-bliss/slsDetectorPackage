/*
 * qTabAdvanced.cpp
 *
 *  Created on: May 10, 2012
 *      Author: l_maliakal_d
 */
#include "qTabAdvanced.h"
#include "qDefs.h"
/** Project Class Headers */
#include "slsDetector.h"
#include "multiSlsDetector.h"
/** C++ Include Headers */
#include<iostream>
using namespace std;




qTabAdvanced::qTabAdvanced(QWidget *parent,multiSlsDetector*& detector):QWidget(parent),myDet(detector){
	setupUi(this);
	SetupWidgetWindow();
	Initialization();
}




qTabAdvanced::~qTabAdvanced(){
	delete myDet;
}




void qTabAdvanced::SetupWidgetWindow(){

}



void qTabAdvanced::Initialization(){

}


//-------------------------------------------------------------------------------------------------------------------------------------------------


void qTabAdvanced::Refresh(){

}


//-------------------------------------------------------------------------------------------------------------------------------------------------

