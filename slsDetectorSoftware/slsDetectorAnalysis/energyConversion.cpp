#include "energyConversion.h"

#include <fstream>
#include <sstream>
#include <iostream>
#include <cstring>

#include "fileIOStatic.h"

using namespace std;




int energyConversion::readCalibrationFile(string fname, double &gain, double &offset){

  string str;
  ifstream infile;
#ifdef VERBOSE
  printf( "Opening file %s \n", fname.c_str() );
#endif
  infile.open(fname.c_str(), ios_base::in);
  if (infile.is_open()) {
    getline(infile,str);
#ifdef VERBOSE
    printf( "%s \n", str.c_str() );
#endif
    istringstream ssstr(str);
    ssstr >> offset >> gain;
    infile.close();
	printf( "Calibration file loaded: %s \n", fname.c_str() );
  } else {
    printf( "Could not open calibration file %s \n", fname.c_str() );
    gain=0.;
    offset=0.;
#ifndef MYROOT
    return FAIL;
#endif
    return -1;
  }
#ifndef MYROOT
  return OK;
#endif
  return 0;
};

int energyConversion::writeCalibrationFile(string fname, double gain, double offset){
  //printf( "Function not yet implemented \n" );
  ofstream outfile;
  outfile.open (fname.c_str());

  // >> i/o operations here <<
  if (outfile.is_open()) {
    outfile << offset << " " << gain << std::endl;
  } else {
    printf( "Could not open calibration file %s for writing \n", fname.c_str() );
#ifndef MYROOT
    return FAIL;
#endif
    return -1;
  }

  outfile.close();

#ifndef MYROOT
  return OK;
#endif
  return 0;
};


int energyConversion::readCalibrationFile(string fname, int *gain, int *offset){

	string str;
	ifstream infile;
	double o,g;
	int ig=0;
#ifdef VERBOSE
	printf( "Opening file %s \n", fname.c_str() );
#endif
	infile.open(fname.c_str(), ios_base::in);
	if (infile.is_open()) {
		//get gain and offset
		for (ig=0; ig<4; ig++) {
			//while ( (getline(infile,str)) > -1) {
			getline(infile,str);
#ifdef VERBOSE
			printf( "%s \n", str.c_str() );
#endif
			istringstream ssstr(str);
			ssstr >> o >> g;
			offset[ig]=(int)(o*1000);
			gain[ig]=(int)(g*1000);
			// ig++;
			if (ig>=4)
				break;
		}
		infile.close();
		printf( "Calibration file loaded: %s \n", fname.c_str() );
	} else {
		printf( "Could not open calibration file: %s \n", fname.c_str() );
		gain[0]=0;
		offset[0]=0;
#ifndef MYROOT
		return FAIL;
#endif
		return -1;
	}
#ifndef MYROOT
	return OK;
#endif
	return 0;

};

int energyConversion::writeCalibrationFile(string fname, int *gain, int *offset){
  //printf( "Function not yet implemented \n" );
  ofstream outfile;	
  outfile.open (fname.c_str());
  // >> i/o operations here <<
  if (outfile.is_open()) {
    for (int ig=0; ig<4; ig++)
      outfile << ((double)offset[ig]/1000) << " " << ((double)gain[ig]/1000) << std::endl;
  } else {
    printf( "Could not open calibration file %s for writing \n", fname.c_str() );
#ifndef MYROOT
    return FAIL;
#endif
    return -1;
  }

  outfile.close();
#ifndef MYROOT
  return OK;
#endif
  return 0;
};



slsDetectorDefs::sls_detector_module* energyConversion::interpolateTrim(detectorType myDetectorType, sls_detector_module* a, sls_detector_module* b, const int energy, const int e1, const int e2){
	// only implemented for eiger currently (in terms of which dacs)
	if(myDetectorType != EIGER) {
		printf("Interpolation of Trim values not implemented for this detector!\n");
		return NULL;
	}

	sls_detector_module*  myMod = createModule(myDetectorType);
	enum eiger_DacIndex{SVP,VTR,VRF,VRS,SVN,VTGSTV,VCMP_LL,VCMP_LR,CAL,VCMP_RL,RXB_RB,RXB_LB,VCMP_RR,VCP,VCN,VIS};

	//Copy other dacs
	int num_dacs_to_copy = 9;
	int dacs_to_copy[] = {SVP,VTR,VRS,SVN,VTGSTV,RXB_RB,RXB_LB,VCN,VIS};
	for (int i = 0; i <  num_dacs_to_copy; ++i) {
		if(a->dacs[dacs_to_copy[i]] != b->dacs[dacs_to_copy[i]]) {
			deleteModule(myMod);
			return NULL;
		}
		myMod->dacs[dacs_to_copy[i]] = a->dacs[dacs_to_copy[i]];
	}

	//Copy irrelevant dacs (without failing): CAL
	if (a->dacs[CAL] != b->dacs[CAL]) {
		printf("Warning: DAC CAL differs in both energies (%d, %d)! ",
		       a->dacs[CAL], b->dacs[CAL]);
		printf("Taking first: %d\n", a->dacs[CAL]);
	}
	myMod->dacs[CAL] = a->dacs[CAL];

	//Interpolate vrf, vcmp, vcp
	int num_dacs_to_interpolate = 6;
	int dacs_to_interpolate[] = {VRF,VCMP_LL,VCMP_LR,VCMP_RL,VCMP_RR,VCP};
	for (int i = 0; i <  num_dacs_to_interpolate; ++i) {
		myMod->dacs[dacs_to_interpolate[i]] = linearInterpolation(energy, e1, e2,
				a->dacs[dacs_to_interpolate[i]], b->dacs[dacs_to_interpolate[i]]);
	}

	//Interpolate all trimbits
	for (int i = 0; i<myMod->nchan; i++)
		myMod->chanregs[i] = linearInterpolation(energy, e1, e2, a->chanregs[i], b->chanregs[i]);
	return myMod;
}



#ifndef MYROOT

/* I/O */


slsDetectorDefs::sls_detector_module* energyConversion::readSettingsFile(string fname, detectorType myDetectorType, int& iodelay, int& tau, sls_detector_module* myMod){




	int nflag=0;


	if (myMod==NULL) {
		myMod=createModule(myDetectorType);
		nflag=1;
	}

	int id=0,i;
	string names[100];
	string myfname;
	string str;
	ifstream infile;
	ostringstream oss;
	int iline=0;
	string sargname;
	int ival;
	int ichan=0, ichip=0, idac=0;
	int nch=((myMod->nchan)/(myMod->nchip));

	//ascii settings/trim file
	switch (myDetectorType) {
	case MYTHEN:
		break;
	case MOENCH:
		names[id++]="Vdac0";
		names[id++]="Vdac1";
		names[id++]="Vdac2";
		names[id++]="Vdac3";
		names[id++]="Vdac4";
		names[id++]="Vdac5";
		names[id++]="Vdac6";
		names[id++]="Vdac7";
		break;
	case GOTTHARD:
	case PROPIX:
		names[id++]="Vref";
		names[id++]="VcascN";
		names[id++]="VcascP";
		names[id++]="Vout";
		names[id++]="Vcasc";
		names[id++]="Vin";
		names[id++]="Vref_comp";
		names[id++]="Vib_test";
		break;
	case EIGER:
		break;
	case JUNGFRAU:
		names[id++]="VDAC0";
		names[id++]="VDAC1";
		names[id++]="VDAC2";
		names[id++]="VDAC3";
		names[id++]="VDAC4";
		names[id++]="VDAC5";
		names[id++]="VDAC6";
		names[id++]="VDAC7";
		names[id++]="VDAC8";
		names[id++]="VDAC9";
		names[id++]="VDAC10";
		names[id++]="VDAC11";
		names[id++]="VDAC12";
		names[id++]="VDAC13";
		names[id++]="VDAC14";
		names[id++]="VDAC15";
		break;
	default:
		printf( "Unknown detector type - unknown format for settings file \n" );
		return NULL;
	}

#ifdef VERBOSE
	printf( " reading settings file for module number %d \n", myMod->module );
#endif
	myfname=fname;
#ifdef VERBOSE
	printf( "file name is %s \n", myfname.c_str() );
#endif

	switch (myDetectorType) {

	case MYTHEN:
		infile.open(myfname.c_str(), ios_base::in);
		if (infile.is_open()) {
			for (int iarg=0; iarg<myMod->ndac; iarg++) {
				getline(infile,str);
				iline++;
				istringstream ssstr(str);
				ssstr >> sargname >> ival;
#ifdef VERBOSE
				printf( "%s dac nr. %d is %d \n", sargname.c_str(), idac, ival );
#endif
				myMod->dacs[idac]=ival;
				idac++;
			}
			for (ichip=0; ichip<myMod->nchip; ichip++) {
				getline(infile,str);
				iline++;
#ifdef VERYVERBOSE
				printf( "%s \n", str.c_str() );
#endif
				istringstream ssstr(str);
				ssstr >> sargname >> ival;
#ifdef VERYVERBOSE
				printf( "chip %d %s is %d \n", ichip, sargname.c_str(), ival );
#endif

				myMod->chipregs[ichip]=ival;
				for (ichan=0; ichan<nch; ichan++) {
					getline(infile,str);
#ifdef VERYVERBOSE
					printf( " %s \n", str.c_str() );
#endif
					istringstream ssstr(str);

#ifdef VERYVERBOSE
					printf( "channel %d iline %d \n", ichan+ichip*myMod->nchan, iline );
#endif
					iline++;
					myMod->chanregs[ichip*nch+ichan]=0;
					for (int iarg=0; iarg<6 ; iarg++) {
						ssstr >>  ival;
						//if (ssstr.good()) {
						switch (iarg) {
						case 0:
#ifdef VERYVERBOSE
							printf( "trimbits %d", ival );
#endif
							myMod->chanregs[ichip*nch+ichan]|=ival&TRIMBITMASK;
							break;
						case 1:
#ifdef VERYVERBOSE
							printf( " compen %d", ival );
#endif
							myMod->chanregs[ichip*nch+ichan]|=ival<<9;
							break;
						case 2:
#ifdef VERYVERBOSE
							printf( " anen %d", ival );
#endif
							myMod->chanregs[ichip*nch+ichan]|=ival<<8;
							break;
						case 3:
#ifdef VERYVERBOSE
							printf( " calen %d", ival ) ;
#endif
							myMod->chanregs[ichip*nch+ichan]|=ival<<7;
							break;
						case 4:
#ifdef VERBOSE
							printf( " outcomp %d", ival ) ;
#endif
							myMod->chanregs[ichip*nch+ichan]|=ival<<10;
							break;
						case 5:
#ifdef VERBOSE
							printf( " counts %d \n", ival );
#endif
							myMod->chanregs[ichip*nch+ichan]|=ival<<11;
							break;
						default:
							printf( " too many columns \n" );
							break;
						}
					}
				}
				//	}
			}
#ifdef VERBOSE
			printf( "read %d channels \n", ichan*ichip );
#endif

			infile.close();
			strcpy(settingsFile,fname.c_str());
			printf("Settings file loaded: %s\n",settingsFile);
			return myMod;

		}


		break;

	case EIGER:
		infile.open(myfname.c_str(),ifstream::binary);
		if (infile.is_open()) {
			infile.read((char*) myMod->dacs,sizeof(dacs_t)*(myMod->ndac));
			infile.read((char*)&iodelay,sizeof(iodelay));
			infile.read((char*)&tau,sizeof(tau));
			infile.read((char*) myMod->chanregs,sizeof(int)*(myMod->nchan));
#ifdef VERBOSE
			for(int i=0;i<myMod->ndac;i++)
				printf( "dac %d:%d \n", i, myMod->dacs[i] );
			printf( "iodelay:%d \n", iodelay );
			printf( "tau:%d \n", tau );
#endif
			if(infile.eof()){
				printf( "Error, could not load trimbits end of file reached: %s \n\n", myfname.c_str() );
				if (nflag)
					deleteModule(myMod);

				return NULL;
			}
			infile.close();
			strcpy(settingsFile,fname.c_str());
			printf("Settings file loaded: %s\n",settingsFile);
			return myMod;

		}

		break;

	case MOENCH:
	case GOTTHARD:
	case PROPIX:
	case JUNGFRAU:
		//---------------dacs---------------
		infile.open(myfname.c_str(), ios_base::in);
		if (infile.is_open()) {
			while(infile.good()) {
				getline(infile,str);
				iline++;
#ifdef VERBOSE
				printf( "str: %s \n", str.c_str() );
#endif
				istringstream ssstr(str);
				ssstr >> sargname >> ival;
				for (i=0;i<id;i++){
					if (!strcasecmp(sargname.c_str(),names[i].c_str())){
						myMod->dacs[i]=ival;
						idac++;
#ifdef VERBOSE
						printf( " %s dac nr. %d is %d \n", sargname.c_str(), idac, ival );
#endif
						break;
					}
				}
			}
			if (i < id) {
#ifdef VERBOSE
				printf( " %s dac nr. %d is %d \n", sargname.c_str(), idac, ival );
#endif
			}else
				printf( "Unknown dac %s \n", sargname.c_str() );

			infile.close();
			strcpy(settingsFile,fname.c_str());
			printf("Settings file loaded: %s\n",settingsFile);
			return myMod;

		}

		//----------------------------------
		break;

	default:
		printf( "Unknown detector type - don't know how to read file %s\n",  myfname.c_str());
		infile.close();
		deleteModule(myMod);
		return NULL;

	}

	printf("Error: Could not open settings file %s\n",  myfname.c_str());
	if (nflag)
		deleteModule(myMod);

	return NULL;



};


int energyConversion::writeSettingsFile(string fname, detectorType myDetectorType, sls_detector_module mod, int iodelay, int tau){

	ofstream outfile;

	int nch=((mod.nchan)/(mod.nchip));

	string names[100];
	int id=0;
	switch (myDetectorType) {
	case MYTHEN:
		names[id++]="Vtrim";
		names[id++]="Vthresh";
		names[id++]="Rgsh1";
		names[id++]="Rgsh2";
		names[id++]="Rgpr";
		names[id++]="Vcal";
		names[id++]="outBuffEnable";
		break;
	case MOENCH:
		names[id++]="Vdac0";
		names[id++]="Vdac1";
		names[id++]="Vdac2";
		names[id++]="Vdac3";
		names[id++]="Vdac4";
		names[id++]="Vdac5";
		names[id++]="Vdac6";
		names[id++]="Vdac7";
		break;
	case GOTTHARD:
	case PROPIX:
		names[id++]="Vref";
		names[id++]="VcascN";
		names[id++]="VcascP";
		names[id++]="Vout";
		names[id++]="Vcasc";
		names[id++]="Vin";
		names[id++]="Vref_comp";
		names[id++]="Vib_test";
		break;
	case EIGER:
		break;
	case JUNGFRAU:
		names[id++]="VDAC0";
		names[id++]="VDAC1";
		names[id++]="VDAC2";
		names[id++]="VDAC3";
		names[id++]="VDAC4";
		names[id++]="VDAC5";
		names[id++]="VDAC6";
		names[id++]="VDAC7";
		names[id++]="VDAC8";
		names[id++]="VDAC9";
		names[id++]="VDAC10";
		names[id++]="VDAC11";
		names[id++]="VDAC12";
		names[id++]="VDAC13";
		names[id++]="VDAC14";
		names[id++]="VDAC15";
		break;
	default:
		printf( "Unknown detector type - unknown format for settings file \n" );
		return FAIL;
	}

	int iv, ichan, ichip;
	int iv1, idac;
	int nb;

	switch (myDetectorType) {
	case EIGER:
		outfile.open(fname.c_str(), ofstream::binary);
		if (outfile.is_open()) {
			iv = 1150;
#ifdef VERBOSE
			for(int i=0;i<mod.ndac;i++)
				printf( "dac %d:%d \n", i, mod.dacs[i] );
			printf( "iodelay: %d \n", iodelay );
			printf( "tau: %d", tau);
#endif
			outfile.write((char*)mod.dacs, sizeof(dacs_t)*(mod.ndac));
			outfile.write(reinterpret_cast<char*>(&iodelay), sizeof(iodelay));
			outfile.write(reinterpret_cast<char*>(&tau), sizeof(tau));
			outfile.write((char*)mod.chanregs, sizeof(int)*(mod.nchan));

			outfile.close();
			return slsDetectorDefs::OK;
		}

		printf("Could not open Settings file %s\n", fname.c_str());
		return slsDetectorDefs::FAIL;
	default:


		outfile.open(fname.c_str(), ios_base::out);

		if (outfile.is_open()) {
			for (idac=0; idac<mod.ndac; idac++) {
				iv=(int)mod.dacs[idac];
				outfile << names[idac] << " " << iv << std::endl;
			}

			if(myDetectorType==MYTHEN){
				for (ichip=0; ichip<mod.nchip; ichip++) {
					iv1=mod.chipregs[ichip]&1;
					outfile << names[idac] << " " << iv1 << std::endl;
					for (ichan=0; ichan<nch; ichan++) {
						iv=mod.chanregs[ichip*nch+ichan];
						iv1= (iv&TRIMBITMASK);
						outfile <<iv1 << " ";
						nb=9;
						iv1=((iv&(1<<nb))>>nb);
						outfile << iv1 << " ";
						nb=8;
						iv1=((iv&(1<<nb))>>nb);
						outfile << iv1 << " ";
						nb=7;
						iv1=((iv&(1<<nb))>>nb);
						outfile <<iv1  << " ";
						nb=10;
						iv1=((iv&(1<<nb))>>nb);
						outfile << iv1 << " ";
						nb=11;
						iv1= ((iv&0xfffff800)>>nb);
						outfile << iv1  << std::endl;
					}
				}
			}
			outfile.close();
			return OK;
		}
		printf( "could not open SETTINGS file %s \n", fname.c_str() );
		return FAIL;

	}


};



#endif
