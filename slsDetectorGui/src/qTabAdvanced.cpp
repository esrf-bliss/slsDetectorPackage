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
	myDetType = (int)myDet->getDetectorsType();
	SetupWidgetWindow();
	Initialization();
}




qTabAdvanced::~qTabAdvanced(){
	delete myDet;
}




void qTabAdvanced::SetupWidgetWindow(){
	/** Temperature */
	if(myDetType==slsDetectorDefs::GOTTHARD)
		getTemperature(); //check if gotthard???
	else{
		btnTemp->setEnabled(false);
		dispTempAdc->setEnabled(false);
		dispTempFpga->setEnabled(false);
	}
}



void qTabAdvanced::Initialization(){
	/** Temperature */
	connect(btnTemp,SIGNAL(clicked()),this,SLOT(getTemperature()));//check if gotthard???
}



void qTabAdvanced::Enable(bool enable){
	btnTemp->setEnabled(enable);
	dispTempAdc->setEnabled(enable);
	dispTempFpga->setEnabled(enable);
	pushButton->setEnabled(enable);
}


void qTabAdvanced::getTemperature(){
	char ctemp[200];
	/** adc */
	double tempadc = myDet->getADC(slsDetectorDefs::TEMPERATURE_ADC);
	sprintf(ctemp,"%f%cC",tempadc,0x00B0);
	dispTempAdc->setText(QString(ctemp));
	/** fpga */
	double tempfpga = myDet->getADC(slsDetectorDefs::TEMPERATURE_FPGA);
	sprintf(ctemp,"%f%cC",tempfpga,0x00B0);
	dispTempFpga->setText(QString(ctemp));
#ifdef VERBOSE
	cout<<"Temperature of ADC: "<<tempadc<<"°C and FPGA: "<<tempfpga<<"°C"<<endl;
#endif
}

//-------------------------------------------------------------------------------------------------------------------------------------------------


void qTabAdvanced::Refresh(){

}


//-------------------------------------------------------------------------------------------------------------------------------------------------

