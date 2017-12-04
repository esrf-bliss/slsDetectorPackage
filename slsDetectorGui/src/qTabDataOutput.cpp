/*
 * qTabDataOutput.cpp
 *
 *  Created on: May 10, 2012
 *      Author: l_maliakal_d
 */
/**********************************************************************
 * TO DO
 * 1. Rate correction auto: for eiger depends on settings, tdeadtime{vv,vv,vv} in postprocessing.h
 * ********************************************************************/

#include "qTabDataOutput.h"
// Project Class Headers
#include "slsDetector.h"
#include "multiSlsDetector.h"
// Qt Include Headers
#include <QFileDialog>
// C++ Include Headers
#include <iostream>
#include <string>
using namespace std;


//-------------------------------------------------------------------------------------------------------------------------------------------------



qTabDataOutput::qTabDataOutput(QWidget *parent,multiSlsDetector*& detector):
						QWidget(parent),myDet(detector){
	setupUi(this);
	SetupWidgetWindow();
	Refresh();
}


//-------------------------------------------------------------------------------------------------------------------------------------------------


qTabDataOutput::~qTabDataOutput(){
	delete myDet;
}


//-------------------------------------------------------------------------------------------------------------------------------------------------


void qTabDataOutput::SetupWidgetWindow(){
	// Detector Type
	detType=myDet->getDetectorsType();

	//rate correction  - not for charge integrating detectors
	if((detType == slsDetectorDefs::MYTHEN)||(detType == slsDetectorDefs::EIGER))
		chkRate->setEnabled(true);

	if((detType == slsDetectorDefs::MYTHEN)||(detType == slsDetectorDefs::GOTTHARD))
		chkAngular->setEnabled(true);

	if(detType == slsDetectorDefs::EIGER)
		chkTenGiga->setEnabled(true);

	/** error message **/
	red = QPalette();
	red.setColor(QPalette::Active,QPalette::WindowText,Qt::red);
	black = QPalette();
	black.setColor(QPalette::Active,QPalette::WindowText,Qt::black);

	red1 = new QPalette();
	red1->setColor(QPalette::Text,Qt::red);
	black1 = new QPalette();
	black1->setColor(QPalette::Text,Qt::black);

	flatFieldTip = dispFlatField->toolTip();
	errFlatFieldTip = QString("<nobr>Flat field corrections.</nobr><br>"
			"<nobr> #flatfield# filename</nobr><br><br>")+
			QString("<nobr><font color=\"red\">"
					"Enter a valid file to enable Flat Field.</font></nobr>");
	outDirTip = boxOutDir->toolTip();


	//expert mode is not enabled initially
	chkCompression->setEnabled(false);

	Initialization();

	disconnect(comboDetector,		SIGNAL(currentIndexChanged(int)), 	this, 	SLOT(GetOutputDir()));
	PopulateDetectors();
	connect(comboDetector,		SIGNAL(currentIndexChanged(int)), 	this, 	SLOT(GetOutputDir()));

	//flat field correction from server
#ifdef VERBOSE
	cout  << "Getting flat field" << endl;
#endif
	UpdateFlatFieldFromServer();


	//rate correction - not for charge integrating detectors
	if((detType == slsDetectorDefs::MYTHEN)||(detType == slsDetectorDefs::EIGER)){
#ifdef VERBOSE
		cout  << "Getting rate correction" << endl;
#endif
		UpdateRateCorrectionFromServer();
	}


	//update angular conversion from server
	if((detType == slsDetectorDefs::MYTHEN)||(detType == slsDetectorDefs::GOTTHARD)){
#ifdef VERBOSE
		cout  << "Getting angular conversion" << endl;
#endif
		int ang;
		if(myDet->getAngularConversion(ang))
			chkAngular->setChecked(true);
		emit AngularConversionSignal(chkAngular->isChecked());
	}


	//discard bad channels from server
#ifdef VERBOSE
	cout  << "Getting bad channel correction:" << myDet->getBadChannelCorrection() << endl;
#endif



	disconnect(chkDiscardBad,		SIGNAL(toggled(bool)));
	if(myDet->getBadChannelCorrection())
		chkDiscardBad->setChecked(true);
	else
		chkDiscardBad->setChecked(false);
	connect(chkDiscardBad,		SIGNAL(toggled(bool)), 	this, 	SLOT(DiscardBadChannels()));


	if(detType == slsDetectorDefs::MYTHEN){
		comboDetector->hide();
	}

	qDefs::checkErrorMessage(myDet,"qTabDataOutput::SetupWidgetWindow");
}


//-------------------------------------------------------------------------------------------------------------------------------------------------


void qTabDataOutput::Initialization(){
	//output dir
	connect(comboDetector,		SIGNAL(currentIndexChanged(int)), 	this, 	SLOT(GetOutputDir()));
	connect(dispOutputDir,		SIGNAL(editingFinished()), 	this, 	SLOT(SetOutputDir()));


	connect(btnOutputBrowse,	SIGNAL(clicked()), 			this, 	SLOT(browseOutputDir()));
	//flat field correction
	connect(chkFlatField,		SIGNAL(toggled(bool)), 		this, 	SLOT(SetFlatField()));
	connect(btnFlatField,		SIGNAL(clicked()), 			this, 	SLOT(BrowseFlatFieldPath()));
	//rate correction
	connect(chkRate,			SIGNAL(toggled(bool)), 			this,	SLOT(SetRateCorrection()));
	connect(radioAuto,			SIGNAL(toggled(bool)), 			this, 	SLOT(SetRateCorrection()));
	connect(radioDeadTime,		SIGNAL(toggled(bool)), 			this, 	SLOT(SetRateCorrection()));
	connect(spinDeadTime,		SIGNAL(valueChanged(double)), 	this, 	SLOT(SetRateCorrection()));
	//angular correction
	connect(chkAngular,			SIGNAL(toggled(bool)), 	this, 	SLOT(SetAngularCorrection()));
	//discard bad channels
	connect(chkDiscardBad,		SIGNAL(toggled(bool)), 	this, 	SLOT(DiscardBadChannels()));
	//compression
	connect(chkCompression,		SIGNAL(toggled(bool)), 	this, 	SLOT(SetCompression(bool)));
	//10GbE
	connect(chkTenGiga,		SIGNAL(toggled(bool)), 	this, 	SLOT(EnableTenGigabitEthernet(bool)));
}


//-------------------------------------------------------------------------------------------------------------------------------------------------


void qTabDataOutput::SetExpertMode(bool enable){
	if((detType == slsDetectorDefs::GOTTHARD) || (detType == slsDetectorDefs::MOENCH)){
		chkCompression->setEnabled(enable);
		GetCompression();
	}

}


//-------------------------------------------------------------------------------------------------------------------------------------------------


void qTabDataOutput::browseOutputDir()
{
	QString directory = QFileDialog::getExistingDirectory(this,tr("Choose Output Directory "),dispOutputDir->text());
	if (!directory.isEmpty())
		dispOutputDir->setText(directory);
	SetOutputDir();
}


//-------------------------------------------------------------------------------------------------------------------------------------------------


void qTabDataOutput::SetFlatField(){
#ifdef VERYVERBOSE
	cout << "Entering Set Flat Field Correction Function" << endl;
#endif
	// so that it doesnt call it twice
	disconnect(dispFlatField,		SIGNAL(editingFinished()),	this, 			SLOT(SetFlatField()));

	//enable/disable
	dispFlatField->setEnabled(chkFlatField->isChecked());
	btnFlatField->setEnabled(chkFlatField->isChecked());

	if(chkFlatField->isChecked()){
		if(dispFlatField->text().isEmpty()){
			chkFlatField->setToolTip(errFlatFieldTip);
			dispFlatField->setToolTip(errFlatFieldTip);
			chkFlatField->setPalette(red);
			chkFlatField->setText("Flat Field File:*");
#ifdef VERBOSE
		cout << "Flat Field File is not set." << endl;
#endif
		}else{
			QString fName = dispFlatField->text();
			QString file = fName.section('/',-1);
			QString dir = fName.section('/',0,-2,QString::SectionIncludeLeadingSep);

			chkFlatField->setToolTip(flatFieldTip);
			dispFlatField->setToolTip(flatFieldTip);
			chkFlatField->setPalette(chkRate->palette());
			chkFlatField->setText("Flat Field File:");
			//set ff dir
			myDet->setFlatFieldCorrectionDir(dir.toAscii().constData());
			//set ff file and catch error if -1
			if(myDet->setFlatFieldCorrectionFile(file.toAscii().constData())<0){
				string sDir = dir.toAscii().constData(),sFile = file.toAscii().constData();
				if(sDir.length()<1) {sDir = string(QDir::current().absolutePath().toAscii().constData()); /*"/home/";*/}
				qDefs::Message(qDefs::WARNING,"Invalid Flat Field file: "+sDir+"/"+sFile+
						".\nUnsetting Flat Field.","qTabDataOutput::SetFlatField");

				//Unsetting flat field
				myDet->setFlatFieldCorrectionFile("");
				dispFlatField->setText("");
				chkFlatField->setToolTip(errFlatFieldTip);
				dispFlatField->setToolTip(errFlatFieldTip);
				chkFlatField->setPalette(red);
				chkFlatField->setText("Flat Field File:*");
#ifdef VERBOSE
		cout << "Invalid Flat Field File - "<< sDir << sFile << ". Unsetting Flat Field." << endl;
#endif
			}
			else{
#ifdef VERBOSE
		cout << "Setting flat field file to "<< dispFlatField->text().toAscii().constData() << endl;
#endif
			}
		}
	}else{
		chkFlatField->setToolTip(flatFieldTip);
		dispFlatField->setToolTip(flatFieldTip);
		chkFlatField->setPalette(chkRate->palette());
		chkFlatField->setText("Flat Field File:");
		//Unsetting flat field
		myDet->setFlatFieldCorrectionFile("");
		dispFlatField->setText("");
#ifdef VERBOSE
		cout << "Unsetting flat field correction file" << endl;
#endif
	}

	connect(dispFlatField,SIGNAL(editingFinished()),this,SLOT(SetFlatField()));

	qDefs::checkErrorMessage(myDet,"qTabDataOutput::SetFlatField");
}

//-------------------------------------------------------------------------------------------------------------------------------------------------

void qTabDataOutput::UpdateFlatFieldFromServer(){
	disconnect(dispFlatField,	SIGNAL(editingFinished()),	this, 	SLOT(SetFlatField()));

	dispFlatField->setText(QString(myDet->getFlatFieldCorrectionDir().c_str())+"/"+QString(myDet->getFlatFieldCorrectionFile().c_str()));
#ifdef VERBOSE
	cout << "Getting flat field correction file" << dispFlatField->text().toAscii().constData() << endl;
#endif
	//calls setflatfield to ensure the file still exists or disable it
	if(!QString(myDet->getFlatFieldCorrectionFile().c_str()).compare("none")){
		dispFlatField->setText("");
		chkFlatField->setChecked(false);
#ifdef VERBOSE
		cout << "Flat Field is not set." << endl;
#endif
	}
	else
		chkFlatField->setChecked(true);

	chkFlatField->setToolTip(flatFieldTip);
	dispFlatField->setToolTip(flatFieldTip);
	chkFlatField->setPalette(chkRate->palette());
	chkFlatField->setText("Flat Field File:");

	connect(dispFlatField,		SIGNAL(editingFinished()),	this,	SLOT(SetFlatField()));

	qDefs::checkErrorMessage(myDet,"qTabDataOutput::UpdateFlatFieldFromServer");
}

//-------------------------------------------------------------------------------------------------------------------------------------------------

void qTabDataOutput::BrowseFlatFieldPath(){
	QString fName = dispFlatField->text();
	QString dir = fName.section('/',0,-2,QString::SectionIncludeLeadingSep);
	if(dir.isEmpty()) dir =  QString(myDet->getFlatFieldCorrectionDir().c_str());/*"/home/";*/
	fName = QFileDialog::getOpenFileName(this,
			tr("Load Flat Field Correction File"),dir,
					     tr("Data Files(*.raw *.dat);; All Files (*.*)"),0,QFileDialog::ShowDirsOnly);
	if (!fName.isEmpty()){
		dispFlatField->setText(fName);
		SetFlatField();
	}

	qDefs::checkErrorMessage(myDet,"qTabDataOutput::BrowseFlatFieldPath");

}

//-------------------------------------------------------------------------------------------------------------------------------------------------


void qTabDataOutput::SetRateCorrection(){
	disconnect(radioAuto,		SIGNAL(toggled(bool)), 			this, 	SLOT(SetRateCorrection()));
	disconnect(radioDeadTime,	SIGNAL(toggled(bool)), 			this, 	SLOT(SetRateCorrection()));
	disconnect(spinDeadTime,	SIGNAL(valueChanged(double)), 	this, 	SLOT(SetRateCorrection()));

#ifdef VERBOSE
	cout << "Entering Set Rate Correction function" << endl;
#endif

	if(chkRate->isChecked()){
		radioAuto->setEnabled(true);
		radioDeadTime->setEnabled(true);
		//set auto as default if nothing selected
		if(!radioAuto->isChecked()&&!radioDeadTime->isChecked())
			radioAuto->setChecked(true);
		//auto
		if(radioAuto->isChecked()){
			spinDeadTime->setEnabled(false);
			myDet->setRateCorrection(-1);
#ifdef VERBOSE
			cout << "Setting rate corrections with default dead time"  << endl;
#endif
		}//custom dead time
		else{
			spinDeadTime->setEnabled(true);
			myDet->setRateCorrection((double)spinDeadTime->value());
#ifdef VERBOSE
			cout << "Setting rate corrections with dead time "<< spinDeadTime->value() << endl;
#endif
		}
	}//unsetting
	else{
		radioAuto->setEnabled(false);
		radioDeadTime->setEnabled(false);
		spinDeadTime->setEnabled(false);
		//Unsetting rate correction

		myDet->setRateCorrection(0);
#ifdef VERBOSE
		cout << "Unsetting rate correction" << endl;
#endif
	}
	connect(radioAuto,		SIGNAL(toggled(bool)), 			this, 	SLOT(SetRateCorrection()));
	connect(radioDeadTime,	SIGNAL(toggled(bool)), 			this, 	SLOT(SetRateCorrection()));
	connect(spinDeadTime,	SIGNAL(valueChanged(double)), 	this, 	SLOT(SetRateCorrection()));

	qDefs::checkErrorMessage(myDet,"qTabDataOutput::SetRateCorrection");
}


//-------------------------------------------------------------------------------------------------------------------------------------------------


void qTabDataOutput::UpdateRateCorrectionFromServer(){
	disconnect(chkRate,			SIGNAL(toggled(bool)), 			this, 	SLOT(SetRateCorrection()));
	disconnect(radioAuto,		SIGNAL(toggled(bool)), 			this, 	SLOT(SetRateCorrection()));
	disconnect(radioDeadTime,	SIGNAL(toggled(bool)), 			this, 	SLOT(SetRateCorrection()));
	disconnect(spinDeadTime,	SIGNAL(valueChanged(double)), 	this, 	SLOT(SetRateCorrection()));

	double rate;
	rate = (double)myDet->getRateCorrectionTau();
	qDefs::checkErrorMessage(myDet,"qTabDataOutput::UpdateRateCorrectionFromServer");
#ifdef VERBOSE
	cout << "Getting rate correction from server:" << rate << " : ";
#endif
	if(rate==0){
#ifdef VERBOSE
	cout << "None" << endl;
#endif
		radioAuto->setEnabled(false);
		radioDeadTime->setEnabled(false);
		spinDeadTime->setEnabled(false);

		chkRate->setChecked(false);
	}else if(rate<0){
#ifdef VERBOSE
	cout << "Auto" << endl;
#endif
		radioAuto->setEnabled(true);
		radioDeadTime->setEnabled(true);
		spinDeadTime->setEnabled(false);

		chkRate->setChecked(true);
		radioAuto->setChecked(true);
	}else{
#ifdef VERBOSE
	cout << "Custom" << endl;
#endif
		radioAuto->setEnabled(true);
		radioDeadTime->setEnabled(true);
		spinDeadTime->setEnabled(true);

		chkRate->setChecked(true);
		radioDeadTime->setChecked(true);
		spinDeadTime->setValue((double)rate);
	}

	connect(chkRate,		SIGNAL(toggled(bool)), 			this, 	SLOT(SetRateCorrection()));
	connect(radioAuto,		SIGNAL(toggled(bool)), 			this, 	SLOT(SetRateCorrection()));
	connect(radioDeadTime,	SIGNAL(toggled(bool)), 			this, 	SLOT(SetRateCorrection()));
	connect(spinDeadTime,	SIGNAL(valueChanged(double)), 	this, 	SLOT(SetRateCorrection()));
}


//-------------------------------------------------------------------------------------------------------------------------------------------------


void qTabDataOutput::SetAngularCorrection(){
	disconnect(chkAngular,			SIGNAL(toggled(bool)), 	this, 	SLOT(SetAngularCorrection()));
#ifdef VERYVERBOSE
	cout << "Entering Set Angular Correction function" << endl;
#endif
	bool enabled = chkAngular->isChecked();
	//set
	if(myDet->setAngularCorrectionMask(enabled) == enabled){
#ifdef VERBOSE
		cout << "Angular Conversion mask:"  << enabled << endl;
#endif
	}
	//error
	else{
#ifdef VERBOSE
		cout << "Could not set angular conversion to default"  << endl;
#endif
		qDefs::Message(qDefs::WARNING,"Angular Conversion could not be set/reset. Please set the default file name using the command line, if you want to set it.","qTabDataOutput::SetAngularCorrection");
		chkAngular->setChecked(!enabled);
	}

	emit AngularConversionSignal(chkAngular->isChecked());
	connect(chkAngular,			SIGNAL(toggled(bool)), 	this, 	SLOT(SetAngularCorrection()));
	qDefs::checkErrorMessage(myDet,"qTabDataOutput::SetAngularCorrection");
}


//-------------------------------------------------------------------------------------------------------------------------------------------------


void qTabDataOutput::DiscardBadChannels(){
#ifdef VERBOSE
	cout << "Entering Discard bad channels function" << endl;
#endif
	if(chkDiscardBad->isChecked()){
#ifdef VERBOSE
		cout << "Setting bad channel correction to default"  << endl;
#endif
		myDet->setBadChannelCorrection("default");
	}else{
#ifdef VERBOSE
		cout << "Unsetting bad channel correction" << endl;
#endif
		myDet->setBadChannelCorrection("");
	}

	qDefs::checkErrorMessage(myDet,"qTabDataOutput::DiscardBadChannels");
}


//-------------------------------------------------------------------------------------------------------------------------------------------------


void qTabDataOutput::PopulateDetectors(){
#ifdef VERBOSE
	cout << "Populating detectors" << endl;
#endif
	comboDetector->clear();
	if(myDet->setReceiverOnline() == slsDetectorDefs::OFFLINE_FLAG){
		boxOutDir->setTitle("Output Directory");
		comboDetector->addItem("All");
	}

	//if receiver, add detectors
	else{
		boxOutDir->setTitle("Receiver Output Directory");
		for(int i=0;i<myDet->getNumberOfDetectors();i++)
			comboDetector->addItem(QString(myDet->getHostname(i).c_str()));
	}
	GetOutputDir();
}


//-------------------------------------------------------------------------------------------------------------------------------------------------


void qTabDataOutput::GetOutputDir(){
#ifdef VERBOSE
	cout << "Getting output directory" << endl;
#endif

	if(!comboDetector->itemText(0).compare("All")){
		dispReadOutputDir->setText(QString(myDet->getFilePath().c_str()));
	}else{
		slsDetector *det = 	myDet->getSlsDetector(comboDetector->currentIndex());
		qDefs::checkErrorMessage(myDet,"qTabDataOutput::GetOutputDir");
		dispReadOutputDir->setText(QString(det->getFilePath().c_str()));
	}


	VerifyOutputDirectory();

	//clear field to write
	disconnect(dispOutputDir,		SIGNAL(editingFinished()), 	this, 	SLOT(SetOutputDir()));
	dispOutputDir->setText("");
	dispOutputDir->setPalette(*black1);
	connect(dispOutputDir,		SIGNAL(editingFinished()), 	this, 	SLOT(SetOutputDir()));
}


//-------------------------------------------------------------------------------------------------------------------------------------------------


int qTabDataOutput::VerifyOutputDirectory(){
#ifdef VERBOSE
	cout << "Verifying output directory" << endl;
#endif
	bool error = false;
	QString errTip = outDirTip;
	bool receiver = (comboDetector->itemText(0).compare("All"));
	string path = string(dispReadOutputDir->text().toAscii().constData());
	string detName = "";
	string mess = "";

	//for each detector
	for(int i=0;i<myDet->getNumberOfDetectors();i++){
		slsDetector *det = 	myDet->getSlsDetector(i);
		qDefs::checkErrorMessage(myDet,"qTabDataOutput::VerifyOutputDirectory");
		if(receiver){
			detName = string("\n - ") + string(comboDetector->itemText(i).toAscii().constData()) ;
			path = det->getFilePath();
		}

		if(det->setFilePath(path).empty()){
			mess. append(detName);
			error = true;
		}else if(!qDefs::checkErrorMessage(det,"qTabDataOutput::VerifyOutputDirectory",false).empty()){
			mess. append(detName);
			error = true;
		}
		/*
		//verify all paths are the same for no receiver
		if ((!receiver) && (path != det->getFilePath())){
			error = true;
			qDefs::Message(qDefs::WARNING,string("Enter a valid output directory ") + detName,"Data Output Verify");
		}*/
		myDet->setFilePath(det->getFilePath());
	}


	//set the read output dir anyway
	if(receiver)
		path = myDet->getSlsDetector(comboDetector->currentIndex())->getFilePath();
	else
		path = myDet->getFilePath();
	dispReadOutputDir->setText(QString(path.c_str()));


	//if error, display in red
	if(error){
#ifdef VERBOSE
		cout << "The output path doesnt exist anymore" << endl;
#endif
		qDefs::Message(qDefs::WARNING,string("Invalid Output Directory ")+ mess ,"qTabDataOutput::VerifyOutputDirectory");
		dispReadOutputDir->setPalette(*red1);
		boxOutDir->setPalette(red);

		//replace all \n with <br>
		size_t pos = 0;
		while((pos = mess.find("\n", pos)) != string::npos){
			mess.replace(pos, 1, "<br>");
			pos += 1;
		}
		errTip = errTip +
				QString("<br><nobr><font color=\"red\">"
						"Invalid <b>Output Directory</b>") + QString(mess.c_str()) +
						QString( ".</font></nobr>");
		boxOutDir->setToolTip(errTip);

		if(receiver)
			boxOutDir->setTitle("Receiver Output Directory*");
		else
			boxOutDir->setTitle("Output Directory*");

		return slsDetectorDefs::FAIL;
	}

	//no error
	else{
#ifdef VERBOSE
		cout << "The output path has been verified" << endl;
#endif
		dispReadOutputDir->setPalette(*black1);
		boxOutDir->setPalette(black);
		boxOutDir->setToolTip(outDirTip);

		if(receiver)
			boxOutDir->setTitle("Receiver Output Directory");
		else
			boxOutDir->setTitle("Output Directory");
	}

	return slsDetectorDefs::OK;

}


//-------------------------------------------------------------------------------------------------------------------------------------------------


void qTabDataOutput::SetOutputDir(){

#ifdef VERBOSE
	cout << "Setting output directory" << endl;
#endif

	bool error = false;
	bool receiver = (comboDetector->itemText(0).compare("All"));
	QString path = dispOutputDir->text();

	if(path.isEmpty())
		return;

	disconnect(dispOutputDir,		SIGNAL(editingFinished()), 	this, 	SLOT(SetOutputDir()));


	//gets rid of the end '/'s
	while(path.endsWith('/')) path.chop(1);
	dispOutputDir->setText(path);


	if(receiver){
		slsDetector *det = 	myDet->getSlsDetector(comboDetector->currentIndex());
		qDefs::checkErrorMessage(myDet,"qTabDataOutput::SetOutputDir");
		det->setFilePath(string(dispOutputDir->text().toAscii().constData()));
		if(!qDefs::checkErrorMessage(det,"qTabDataOutput::SetOutputDir").empty()){
#ifdef VERBOSE
			cout << "The output path could not be set" << endl;
#endif
			det->setFilePath(string(dispReadOutputDir->text().toAscii().constData()));
			error = true;
			dispOutputDir->setPalette(*red1);
		}
		myDet->setFilePath(det->getFilePath());
	}


	else{
		//for each detector
		for(int i=0;i<myDet->getNumberOfDetectors();i++){
			slsDetector *det = 	myDet->getSlsDetector(i);
			qDefs::checkErrorMessage(myDet,"qTabDataOutput::SetOutputDir");
			if(det->setFilePath(string(path.toAscii().constData())).empty()){
				error = true;
			}else if(!qDefs::checkErrorMessage(det,"qTabDataOutput::SetOutputDir",false).empty()){
				error = true;
			}
			myDet->setFilePath(det->getFilePath());
		}


		if(error){
			qDefs::Message(qDefs::WARNING,string("Invalid output directory "),"qTabDataOutput::SetOutputDir");
			for(int i=0;i<myDet->getNumberOfDetectors();i++){
				slsDetector *det = 	myDet->getSlsDetector(i);
				qDefs::checkErrorMessage(myDet,"qTabDataOutput::SetOutputDir");
				det->setFilePath(string(dispReadOutputDir->text().toAscii().constData()));
			}
		}
		dispReadOutputDir->setText(QString(myDet->getFilePath().c_str()));
	}



	if(error){
#ifdef VERBOSE
		cout << "The output path could not be set" << endl;
#endif
		dispOutputDir->setPalette(*red1);
	}
	else{
#ifdef VERBOSE
		cout << "The output path has been modified" << endl;
#endif
		dispOutputDir->setPalette(*black1);
		dispReadOutputDir->setText(dispOutputDir->text());
		dispOutputDir->setText("");
		VerifyOutputDirectory();
	}


	connect(dispOutputDir,		SIGNAL(editingFinished()), 	this, 	SLOT(SetOutputDir()));

}


//-------------------------------------------------------------------------------------------------------------------------------------------------


void qTabDataOutput::GetCompression(){
	disconnect(chkCompression,		SIGNAL(toggled(bool)), 	this, 	SLOT(SetCompression(bool)));
	int ret = myDet->enableReceiverCompression();
	if(ret > 0)	chkCompression->setChecked(true);
	else		chkCompression->setChecked(false);
	connect(chkCompression,			SIGNAL(toggled(bool)), 	this, 	SLOT(SetCompression(bool)));

	qDefs::checkErrorMessage(myDet,"qTabDataOutput::GetCompression");
}


//-------------------------------------------------------------------------------------------------------------------------------------------------


void qTabDataOutput::SetCompression(bool enable){
	disconnect(chkCompression,		SIGNAL(toggled(bool)), 	this, 	SLOT(SetCompression(bool)));
	int ret = myDet->enableReceiverCompression(enable);
	if(ret > 0)	chkCompression->setChecked(true);
	else		chkCompression->setChecked(false);
	connect(chkCompression,			SIGNAL(toggled(bool)), 	this, 	SLOT(SetCompression(bool)));

	qDefs::checkErrorMessage(myDet,"qTabDataOutput::SetCompression");
}


//-------------------------------------------------------------------------------------------------------------------------------------------------


void qTabDataOutput::EnableTenGigabitEthernet(bool enable,int get){
#ifdef VERBOSE
	cout  << endl << "Enabling/Disabling 10GbE" << endl;
#endif
	disconnect(chkTenGiga,	SIGNAL(toggled(bool)), 	this, 	SLOT(EnableTenGigabitEthernet(bool)));
	int ret;
	if(get)
		ret = myDet->enableTenGigabitEthernet(-1);
	else
		ret = myDet->enableTenGigabitEthernet(enable);
	if(ret > 0)	chkTenGiga->setChecked(true);
	else		chkTenGiga->setChecked(false);
	connect(chkTenGiga,		SIGNAL(toggled(bool)), 	this, 	SLOT(EnableTenGigabitEthernet(bool)));

	qDefs::checkErrorMessage(myDet,"qTabDataOutput::EnableTenGigabitEthernet");
}


//-------------------------------------------------------------------------------------------------------------------------------------------------


void qTabDataOutput::Refresh(){
#ifdef VERBOSE
	cout  << endl << "**Updating DataOutput Tab" << endl;
#endif


	// output dir
#ifdef VERBOSE
	cout  << "Getting output directory" << endl;
#endif
	disconnect(comboDetector,		SIGNAL(currentIndexChanged(int)), 	this, 	SLOT(GetOutputDir()));
	PopulateDetectors();
	connect(comboDetector,		SIGNAL(currentIndexChanged(int)), 	this, 	SLOT(GetOutputDir()));


	//flat field correction from server
#ifdef VERBOSE
	cout  << "Getting flat field" << endl;
#endif
	UpdateFlatFieldFromServer();


	//rate correction - not for charge integrating detectors
	if((detType == slsDetectorDefs::MYTHEN)||(detType == slsDetectorDefs::EIGER)){
#ifdef VERBOSE
		cout  << "Getting rate correction" << endl;
#endif
		UpdateRateCorrectionFromServer();
	}


	//update angular conversion from server
	if((detType == slsDetectorDefs::MYTHEN)||(detType == slsDetectorDefs::GOTTHARD)){
#ifdef VERBOSE
		cout  << "Getting angular conversion" << endl;
#endif
		int ang;
		if(myDet->getAngularConversion(ang))
			chkAngular->setChecked(true);
		emit AngularConversionSignal(chkAngular->isChecked());
	}


	//discard bad channels from server
#ifdef VERBOSE
	cout  << "Getting bad channel correction" << endl;//cout << "ff " << myDet->getBadChannelCorrection() << endl;
#endif


	disconnect(chkDiscardBad,		SIGNAL(toggled(bool)));
	if(myDet->getBadChannelCorrection())
		chkDiscardBad->setChecked(true);
	else
		chkDiscardBad->setChecked(false);
	connect(chkDiscardBad,		SIGNAL(toggled(bool)), 	this, 	SLOT(DiscardBadChannels()));

	if(myDet->setReceiverOnline()==slsDetectorDefs::ONLINE_FLAG){
		btnOutputBrowse->setEnabled(false);
		btnOutputBrowse->setToolTip("<font color=\"red\">This button is disabled as receiver PC is different from "
				"client PC and hence different directory structures.</font><br><br>" + dispOutputDir->toolTip());
	}else{
		btnOutputBrowse->setEnabled(true);
		btnOutputBrowse->setToolTip(dispOutputDir->toolTip());
	}

	//getting compression
	if(chkCompression->isEnabled()){
#ifdef VERBOSE
		cout  << "Getting compression" << endl;
#endif
		GetCompression();
	}

	//getting 10GbE
	if(chkTenGiga->isEnabled()){
#ifdef VERBOSE
		cout  << "Getting 10GbE enable" << endl;
#endif
		EnableTenGigabitEthernet(-1,1);
	}




#ifdef VERBOSE
	cout  << "**Updated DataOutput Tab" << endl << endl;
#endif

	qDefs::checkErrorMessage(myDet,"qTabDataOutput::Refresh");
}


//-------------------------------------------------------------------------------------------------------------------------------------------------


