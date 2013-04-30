#include "slsDetectorUtils.h"
#include "usersFunctions.h"
#include "slsDetectorCommand.h"
#include "postProcessing.h"
#include "enCalLogClass.h"
#include "angCalLogClass.h"

#include  <cstdlib>
#include  <sys/ipc.h>
#include  <sys/shm.h>

using namespace std;
slsDetectorUtils::slsDetectorUtils()  {


#ifdef VERBOSE
  cout << "setting callbacks" << endl;
#endif
  acquisition_finished=NULL;
  acqFinished_p=NULL;
  measurement_finished=NULL;
  measFinished_p=NULL;
  progress_call=0;
  pProgressCallArg=0;
  registerGetPositionCallback(&defaultGetPosition, NULL);
  registerConnectChannelsCallback(&defaultConnectChannels,NULL);
  registerDisconnectChannelsCallback(&defaultDisconnectChannels,NULL);
  registerGoToPositionCallback(&defaultGoToPosition,NULL);
  registerGoToPositionNoWaitCallback(&defaultGoToPositionNoWait,NULL);
  registerGetI0Callback(&defaultGetI0,NULL);
#ifdef VERBOSE

  registerAcquisitionFinishedCallback(&dummyAcquisitionFinished,this);
  registerMeasurementFinishedCallback(&dummyMeasurementFinished,this);
  cout << "done " << endl;
#endif

};
 




void  slsDetectorUtils::acquire(int delflag){

  bool receiver = (setReceiverOnline()==ONLINE_FLAG);

  // setTotalProgress();
  //moved these 2 here for measurement change
  progressIndex=0;
  *stoppedFlag=0;

  angCalLogClass *aclog=NULL;
  enCalLogClass *eclog=NULL;
  //  int lastindex=startindex, nowindex=startindex;
  int connectChannels=0;

#ifdef VERBOSE
  cout << "Acquire function "<< delflag << endl;
  cout << "Stopped flag is "<< stoppedFlag << delflag << endl;
#endif

  void *status;




  if ((*correctionMask&(1<< ANGULAR_CONVERSION)) || (*correctionMask&(1<< I0_NORMALIZATION)) || getActionMode(angCalLog) || (getScanMode(0)==positionScan)|| (getScanMode(1)==positionScan)) {
    if (connectChannels==0)
      if (connect_channels) {
	connect_channels(CCarg);
	connectChannels=1;
      }      
  }
  

  if (getActionMode(angCalLog))
    aclog=new angCalLogClass(this);
 
  if (getActionMode(enCalLog))
    eclog=new enCalLogClass(this);
  
  
  
  
  setJoinThread(0);
  positionFinished(0);




  int nm=timerValue[MEASUREMENTS_NUMBER];
  if (nm<1)
    nm=1;

 
  int  np=getNumberOfPositions();
  if (np<1)
    np=1;
  
  
  int ns0=1;
  if (*actionMask & (1 << MAX_ACTIONS)) {
    ns0=getScanSteps(0);
    if (ns0<1)
      ns0=1;
  }

  int ns1=1;
  if (*actionMask & (1 << (MAX_ACTIONS+1))) {
    ns1=getScanSteps(1);
    if (ns1<1)
      ns1=1;
  }

  if(receiver){
	  if(getReceiverStatus()!=IDLE)
		  stopReceiver();
	  if(setReceiverOnline()==OFFLINE_FLAG)
		  *stoppedFlag=1;


	  //resets frames caught in receiver
	  resetFramesCaught();


	  if(setReceiverOnline()==OFFLINE_FLAG)
		  *stoppedFlag=1;
  }


  if (*threadedProcessing) {
    startThread(delflag);
  }
#ifdef VERBOSE
  cout << " starting thread " << endl;
#endif



  for(int im=0;im<nm;im++) {

#ifdef VERBOSE
    cout << " starting measurement "<< im << " of " << nm << endl;
#endif

    //cout << "data thread started " << endl;
 

    //loop measurements

//     pthread_mutex_lock(&mp);
//     setStartIndex(*fileIndex);
//     pthread_mutex_unlock(&mp);
 
    //cout << "action at start" << endl;
    if (*stoppedFlag==0) {
      executeAction(startScript);
    }

    for (int is0=0; is0<ns0; is0++) {
      //  cout << "scan0 loop" << endl;

      if (*stoppedFlag==0) {
	executeScan(0,is0);
      } else
	break;
  

      for (int is1=0; is1<ns1; is1++) {
	// cout << "scan1 loop" << endl;

	if (*stoppedFlag==0) {
	  executeScan(1,is1);
	} else
	  break;

	if (*stoppedFlag==0) {
	  executeAction(scriptBefore);
	} else
	  break;

	ResetPositionIndex();
     
	for (int ip=0; ip<np; ip++) {
	  //   cout << "positions " << endl;
	  if (*stoppedFlag==0) {
	    if  (getNumberOfPositions()>0) {
	      moveDetector(detPositions[ip]);
	      IncrementPositionIndex();
#ifdef VERBOSE
	      std::cout<< "moving to position" << std::endl;
#endif
	    } 
	  } else
	    break;
       
       
	  pthread_mutex_lock(&mp);
	  	  createFileName();
	  pthread_mutex_unlock(&mp);

	  if (*stoppedFlag==0) {
	    executeAction(scriptBefore);
	  } else
	    break;
       
       
       

	  if (*stoppedFlag==0) {


	    executeAction(headerBefore);
	 
	    if (*correctionMask&(1<< ANGULAR_CONVERSION) || aclog){// || eclog) { 
	      positionFinished(0);
	      setCurrentPosition(getDetectorPosition());
	    }
      

	    if (aclog)
	    	aclog->addStep(getCurrentPosition(), getCurrentFileName());

	    if (eclog)
			eclog->addStep(setDAC(-1,THRESHOLD), getCurrentFileName());


	    if (*correctionMask&(1<< I0_NORMALIZATION)) {
	      if (get_i0)
		get_i0(0, IOarg);
	    }

	    setCurrentFrameIndex(0);
		//if ((timerValue[FRAME_NUMBER]*timerValue[CYCLES_NUMBER])>1) {
	    if ((setTimer(FRAME_NUMBER,-1)*setTimer(CYCLES_NUMBER,-1))>1){
	      setFrameIndex(0);
	    } else {
	      setFrameIndex(-1);
	    }

	    if(receiver){
	    	//send receiver file name
	    	pthread_mutex_lock(&mp);
	    	createFileName();
	    	pthread_mutex_unlock(&mp);

	    	pthread_mutex_lock(&mg);
	    	setFileName(fileIO::getFileName());
	    	if(setReceiverOnline()==OFFLINE_FLAG){
	    		stopReceiver();
	    		pthread_mutex_unlock(&mg);
	    		break;
	    	}
	    	//start receiver
	    	startReceiver();
	    	if(setReceiverOnline()==OFFLINE_FLAG){
	    		stopReceiver();
	    		pthread_mutex_unlock(&mg);
	    		break;
	    	}
	    	pthread_mutex_unlock(&mg);
	    }
#ifdef VERBOSE
	    cout << "Acquiring " << endl;
#endif     
	    startAndReadAll();
#ifdef VERBOSE
	    cout << "finished " << endl;
	    cout << "returned! " << endl;
#endif     
       


	    if (*correctionMask&(1<< I0_NORMALIZATION)) {
	      if (get_i0) 
		currentI0=get_i0(1,IOarg); // this is the correct i0!!!!!
	    }
#ifdef VERBOSE
	    cout << "pos finished? " << endl;
#endif     

	    positionFinished(1);
       
#ifdef VERBOSE
	    cout << "done! " << endl;
#endif     
 

	    if (*threadedProcessing==0){
#ifdef VERBOSE
	      cout << "start unthreaded process data " << endl;
#endif

	      processData(delflag); 
	    } 

	  } else
	    break;


	  pthread_mutex_lock(&mg);
	  if(setReceiverOnline()==OFFLINE_FLAG){
	  // wait until data processing thread has finished the data

#ifdef VERBOSE
	  cout << "check data queue size " << endl;
#endif
	  while (dataQueueSize()){
#ifdef VERBOSE
	    cout << "AAAAAAAAA check data queue size " << endl;
#endif
	    usleep(100000);
	  }

	  if ((getDetectorsType()==GOTTHARD) || (getDetectorsType()==MOENCH)){
		  if((*correctionMask)&(1<<WRITE_FILE))
			  closeDataFile();
	  }

	  }else{
		 while(stopReceiver()!=OK);
	  }
	  pthread_mutex_unlock(&mg);



	  pthread_mutex_lock(&mp);
	  if (*stoppedFlag==0) {
	    executeAction(headerAfter);

		  pthread_mutex_unlock(&mp);
	    // setLastIndex(*fileIndex);
	  } else {
	    //   setLastIndex(*fileIndex);

		pthread_mutex_unlock(&mp);
	    break;
	  }


	  if (*stoppedFlag) {
#ifdef VERBOSE
	    std::cout<< "exiting since the detector has been stopped" << std::endl;
#endif
	    break;
	  } else if (ip<(np-1)) {
// 	    pthread_mutex_lock(&mp);
// 	    *fileIndex=setStartIndex(); 
// 	    pthread_mutex_unlock(&mp);
	  }
	} // loop on position finished

	//script after
	if (*stoppedFlag==0) {
	  executeAction(scriptAfter);
	} else
	  break;
     
     
	if (*stoppedFlag) {
#ifdef VERBOSE
	  std::cout<< "exiting since the detector has been stopped" << std::endl;
#endif
	  break;
	} else if (is1<(ns1-1)) {
// 	  pthread_mutex_lock(&mp);
// 	  *fileIndex=setStartIndex(); 
// 	  pthread_mutex_unlock(&mp);
	}
      } 

      //end scan1 loop is1
  

      if (*stoppedFlag) {
#ifdef VERBOSE
	std::cout<< "exiting since the detector has been stopped" << std::endl;
#endif
	break;
      } else if (is0<(ns0-1)) {
// 	pthread_mutex_lock(&mp);
// 	*fileIndex=setStartIndex();  
// 	pthread_mutex_unlock(&mp);
      }

    } //end scan0 loop is0

//     pthread_mutex_lock(&mp);
//     *fileIndex=setLastIndex();  
//     pthread_mutex_unlock(&mp);

    if (*stoppedFlag==0) {
      executeAction(stopScript);
    } else{

  	  pthread_mutex_lock(&mg);
#ifdef VERBOSE
    cout << "findex incremented " << endl;
 #endif
    if(*correctionMask&(1<<WRITE_FILE))
      IncrementFileIndex();
    setFileIndex(fileIO::getFileIndex());
    pthread_mutex_unlock(&mg);

      break;
    }



	  pthread_mutex_lock(&mg);
#ifdef VERBOSE
  cout << "findex incremented " << endl;
#endif
  if(*correctionMask&(1<<WRITE_FILE))
    IncrementFileIndex();
  setFileIndex(fileIO::getFileIndex());
  pthread_mutex_unlock(&mg);


    if (measurement_finished)
      measurement_finished(im,*fileIndex,measFinished_p);

    if (*stoppedFlag) {
      break;
    } 


    // loop measurements
  }


  // waiting for the data processing thread to finish!
  if (*threadedProcessing) {
    setJoinThread(1);
    pthread_join(dataProcessingThread, &status);
  }


  if(progress_call)
	progress_call(getCurrentProgress(),pProgressCallArg);


  if (connectChannels) {
    if (disconnect_channels)
      disconnect_channels(DCarg);
  }
   
  if (aclog)
    delete aclog;
   
  if (eclog)
    delete eclog;

  if (acquisition_finished)
    acquisition_finished(getCurrentProgress(),getDetectorStatus(),acqFinished_p);

}





//Naveen change

int slsDetectorUtils::setTotalProgress() {
  
  int nf=1, npos=1, nscan[MAX_SCAN_LEVELS]={1,1}, nc=1, nm=1;

  if (timerValue[FRAME_NUMBER])
    nf=timerValue[FRAME_NUMBER];

  if (timerValue[CYCLES_NUMBER]>0)
    nc=timerValue[CYCLES_NUMBER];

  if (timerValue[MEASUREMENTS_NUMBER]>0)
    nm=timerValue[MEASUREMENTS_NUMBER];

  if (*numberOfPositions>0)
    npos=*numberOfPositions;

  if ((nScanSteps[0]>0) && (*actionMask & (1 << MAX_ACTIONS)))
    nscan[0]=nScanSteps[0];

  if ((nScanSteps[1]>0) && (*actionMask & (1 << (MAX_ACTIONS+1))))
    nscan[1]=nScanSteps[1];
      
  totalProgress=nm*nf*nc*npos*nscan[0]*nscan[1];

#ifdef VERBOSE
  cout << "nc " << nc << endl;
  cout << "nm " << nm << endl;
  cout << "nf " << nf << endl;
  cout << "npos " << npos << endl;
  cout << "nscan[0] " << nscan[0] << endl;
  cout << "nscan[1] " << nscan[1] << endl;

  cout << "Set total progress " << totalProgress << endl;
#endif
  return totalProgress;
}









int slsDetectorUtils::setBadChannelCorrection(string fname, int &nbadtot, int *badchanlist, int off){

  int nbad;
  int badlist[MAX_BADCHANS];

  ifstream infile;
  string str;
  //int interrupt=0;
  //int ich;
  //int chmin,chmax;
#ifdef VERBOSE
  std::cout << "utils: Setting bad channel correction to " << fname << std::endl;
#endif
 // int modmi=0;
  int modma=1;
  int singlefile=0;

  string fn;
  int offset=off;


  nbadtot=0;

  if (fname=="" || fname=="none") {
    ;
  } else { 

    if (fname.find(".sn")==string::npos && fname.find(".chans")==string::npos) {
      modma=setNumberOfModules();
      singlefile=1;
    }

    for (int im=0; im<modma; im++) {
      

      if (singlefile) {

	ostringstream ostfn;
	ostfn << fname << ".sn"  << setfill('0') << setw(3) << hex << getId(MODULE_SERIAL_NUMBER, im); 
	fn=ostfn.str();
  
      } else
	fn=fname;
      


      infile.open(fn.c_str(), ios_base::in);
      if (infile.is_open()==0) {
	std::cout << "could not open file " << fname <<std::endl;
	return -1;
      }


      nbad=setBadChannelCorrection(infile, nbad, badlist, offset);
      infile.close();
    
      for (int ich=0; ich<nbad; ich++) {
	if (nbadtot<MAX_BADCHANS) {
	  badchanlist[nbadtot]=badlist[ich];
	  nbadtot++;
	}
      }
    
      offset+=getChansPerMod(im); 
    
    }
  
  }
  if (nbadtot>0 && nbadtot<MAX_BADCHANS) {
    return nbadtot;
  } else
    return 0;

}




double slsDetectorUtils::getCurrentProgress() {
  pthread_mutex_lock(&mp);
#ifdef VERBOSE
  cout << progressIndex << " / " << totalProgress << endl;
#endif
  
  double p=100.*((double)progressIndex)/((double)totalProgress);
  pthread_mutex_unlock(&mp);
  return p;
}



void slsDetectorUtils::incrementProgress(int i)  {
  pthread_mutex_lock(&mp);
  progressIndex+=i;
  cout << fixed << setprecision(2) << setw (6) << 100.*((double)progressIndex)/((double)totalProgress) << " \%";
  pthread_mutex_unlock(&mp);
#ifdef VERBOSE
  cout << endl;
#else
  cout << "\r" << flush;
#endif

};





int slsDetectorUtils::retrieveDetectorSetup(string const fname1, int level){



  slsDetectorCommand *cmd;


 // char ext[100];
  int skip=0;
  string fname;
  string str;
  ifstream infile;
  int iargval;
  int interrupt=0;
  char *args[10];

  char myargs[10][1000];

  //args[0]=myargs[0];
  //args[1]=myargs[1];

  string sargname, sargval;
  int iline=0;
  
  if (level==2) {
    //     fname=fname1+string(".config");
    //     readConfigurationFile(fname);
#ifdef VERBOSE
    cout << "config file read" << endl;
#endif
    fname=fname1+string(".det");
  }  else
    fname=fname1;

  infile.open(fname.c_str(), ios_base::in);
  if (infile.is_open()) {
    cmd=new slsDetectorCommand(this);
    while (infile.good() and interrupt==0) {
      sargname="none";
      sargval="0";
      getline(infile,str);
      iline++;
#ifdef VERBOSE
      std::cout<<  str << std::endl;
#endif
      if (str.find('#')!=string::npos) {
#ifdef VERBOSE
	std::cout<< "Line is a comment " << std::endl;
	std::cout<< str << std::endl;
#endif
	continue;
      } else {
	istringstream ssstr(str);
	iargval=0;
	while (ssstr.good()) {
	  ssstr >> sargname;
	  //  if (ssstr.good()) {
	  strcpy(myargs[iargval],sargname.c_str());
	  args[iargval]=myargs[iargval];
#ifdef VERBOSE
	  std::cout<< args[iargval]  << std::endl;
#endif
	  iargval++;
	  // }
	  skip=0;
	}

	if (level!=2) {
	  if (string(args[0])==string("flatfield"))
	    skip=1;
	  else if  (string(args[0])==string("badchannels"))
	    skip=1;
	  else if (string(args[0])==string("trimbits"))
	    skip=1;
	}
	if (skip==0)
	  cmd->executeLine(iargval,args,PUT_ACTION);
      }
      iline++;
    }
    delete cmd;
    infile.close();

  } else {
    std::cout<< "Error opening  " << fname << " for reading" << std::endl;
    return FAIL;
  }
#ifdef VERBOSE
  std::cout<< "Read  " << iline << " lines" << std::endl;
#endif
  return iline;


}


int slsDetectorUtils::dumpDetectorSetup(string const fname, int level){

  slsDetectorCommand *cmd;

  string names[]={
    "fname",\
    "index",\
    "flags",\
    "dr",\
    "settings",\
    "threshold",\
    "exptime",\
    "period",\
    "delay",\
    "gates",\
    "frames",\
    "cycles",\
    "probes",\
    "timing",\
    "fineoff",\
    "startscript",\
    "startscriptpar",\
    "stopscript",\
    "stopscriptpar",\
    "scriptbefore",\
    "scriptbeforepar",\
    "scriptafter",\
    "scriptafterpar",\
    "scan0script",\
    "scan0par",\
    "scan0prec",\
    "scan0steps",\
    "scan1script",\
    "scan1par",\
    "scan1prec",\
    "scan1steps",\
    "ratecorr",\
    "flatfield",\
    "badchannels",\
    "trimbits"
  };
  int nvar=35;



 // char ext[100];

  int iv=0;
  string fname1;



  ofstream outfile;
  char *args[2];
  for (int ia=0; ia<2; ia++) {
    args[ia]=new char[1000];
  }





  int nargs;
  if (level==2)
    nargs=2;
  else
    nargs=1;


  if (level==2) {
    fname1=fname+string(".config");
    writeConfigurationFile(fname1);
    fname1=fname+string(".det");
  } else
    fname1=fname;



  outfile.open(fname1.c_str(),ios_base::out);
  if (outfile.is_open()) {
    cmd=new slsDetectorCommand(this);
    for (iv=0; iv<nvar-3; iv++) {
      strcpy(args[0],names[iv].c_str());
      outfile << names[iv] << " " << cmd->executeLine(1,args,GET_ACTION) << std::endl;
    }


    strcpy(args[0],names[iv].c_str());
    if (level==2) {
      fname1=fname+string(".ff");
      strcpy(args[1],fname1.c_str());
    }
    outfile << names[iv] << " " << cmd->executeLine(nargs,args,GET_ACTION) << std::endl;
    iv++;

    strcpy(args[0],names[iv].c_str());
    if (level==2) {
      fname1=fname+string(".bad");
      strcpy(args[1],fname1.c_str());
    }
    outfile << names[iv] << " " << cmd->executeLine(nargs,args,GET_ACTION) << std::endl;
    iv++;



    if (level==2) {
      strcpy(args[0],names[iv].c_str());
      size_t c=fname.rfind('/');
      if (c<string::npos) {
	fname1=fname.substr(0,c+1)+string("trim_")+fname.substr(c+1);
      } else {
	fname1=string("trim_")+fname;
      }
      strcpy(args[1],fname1.c_str());
#ifdef VERBOSE
      std::cout<< "writing to file " << fname1 << std::endl;
#endif
      outfile << names[iv] << " " << cmd->executeLine(nargs,args,GET_ACTION) << std::endl;
      iv++;
  

    }

 
    delete cmd;

    outfile.close();
  }
  else {
    std::cout<< "Error opening parameters file " << fname1 << " for writing" << std::endl;
    return FAIL;
  }
  
#ifdef VERBOSE
  std::cout<< "wrote " <<iv << " lines to  "<< fname1 << std::endl;
#endif
  
  return 0;

} 


