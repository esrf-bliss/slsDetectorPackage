#ifndef NEWMYTHENMACROS_H
#define NEWMYTHENMACROS_H
#include <stdio.h>
#include <iostream>
#include <fstream>
#include <TMath.h>
#include <TStyle.h>
#include <TH1F.h>
#include <TH2F.h>
#include <TF1.h>
#include <TGraph.h>
#include <TGraphErrors.h>
#include <TROOT.h>
#include <THStack.h>
#include <TColor.h>
#include <TCanvas.h>
#include <TAxis.h>
#include <iostream>
#include <TMultiGraph.h>
#include <TLegend.h>


#define NMOD 1
#define NCHMOD 1280
#define NCHAN 128
#define NCHIP 10
#define NCHANS NCHAN*NCHIP*NMOD

#define max(a,b) ((a) > (b) ? (a) : (b))
#define min(a,b) ((a) < (b) ? (a) : (b))
#define ELEM_SWAP(a,b) { register int t=(a);(a)=(b);(b)=t; }

//    printf("module %i center %.3E +- %.2E conversion %.4E +- %.2E offset %.5f +- %.5f \n",mod,center[mod],errcenter[mod],conversion[mod],errconversion[mod],moffset[mod],erroff[mod]);

class angConvFactors //Sample Class for the C++ Tutorial 
{
 private:
  int index;
  float center, errcenter, conversion, errconversion, moff, erroff; 
 public: 
  angConvFactors() //Constructor for the C++ tutorial 
    { 
      index=-1;
      center=640;
      errcenter=0; 
      conversion=6.561E-5;
      errconversion=0; 
      moff=0;
      erroff=0; 
    }
  int Scan(char *line) 
  { 
    if (sscanf(line," module %i center %E +- %E conversion %E +- %E offset %f +- %f \n",&index,&center,&errcenter,&conversion,&errconversion,&moff,&erroff)==7)
      return 1;
    else {
      printf("scanned %d args: %i, %f %f,%f %f,%f %f\n",index,center,errcenter,conversion,errconversion,moff,erroff);
      return 0;
    }
  }
  float getCenter(){return center;};
  float getOffset(){return moff;};
  float getConversion(){return conversion;};

}; 


int rootlogon (); // sets nice plotting settings
TH1F *readDataFile(char* fname, int nch);// reads a data file fname with nch channels and return an 1D histogram
int writeDataFile(char* fname, TH1 *hd);// writes a 1D histogram into a data file fname (ch-value integer format)
int writeImage(char* fname, TH2 *hd);// writes a 2D histogram into a data file fname (one frame per line, float format)
TH1F *translateHisto(TH1F *hin, float delta); // shifts the X axis of a 1D histogram of an offset delta


int flatField(char* fndata, char *fnff, char *fnout, int nch);// flat field correct the data in fndata using the ff file fnff and writing the results in fnout
TH1F *createProjection(TH2F *h2, char *fout);// Writes a file with the projection along y of a 2d histogram (visible part)

TH2F* createThrscan(char* fn, int mi, int ma, int step, int run, int nch, char *ext=".raw"); // Creates a 2D histogram from a set of files acquired with a threshold scan (fn is the path and file name root)
TH2F* createScan(char* fn, int mi, int ma, int step, int nch);// Creates a 2D histogram from a set of files (fn is the format of the file name)
TH2F* createScan(char* fn, float mi, float ma, float step, int nch);// Creates a 2D histogram from a set of files (fn is the format of the file name)


TH2F*  normalize(TH2F* h2, TH2F *hff) ;// Flat field corrects the histogram h2 with the data contained in the 2D histogram hff
TH2F*  resize(TH2F* h2,Double_t  irowmin,Double_t  irowmax,Double_t  icolmin,Double_t  icolmax) ; // resizes a 2D histogram between in a given region of interest
TH1F* getCh(TH2F* h2, int chan);// returns a 1D histogram corresponding to the channel chan
TH1F* getStep(TH2F* h2, int step); // returns a 1D histogram corresponding to the step step
TH1F* getStep(TH2F* h2, Double_t fstep); // returns a 1D histogram corresponding to the step fstep in user's coordinates

TH1F* spectrumFromCh(TH1F* hch) ;// returns the derivated histogram of a 1d histogram 
Double_t erfFunctionAll(Double_t *x, Double_t *par);
Double_t erfFunction(Double_t *x, Double_t *par); // basic erf function
Double_t erfFunction3(Double_t *x, Double_t *par); // erf function with charge sharing
Double_t erfFuncFluo(Double_t *x, Double_t *par);
TF1 *fitERFAll(TH1F *h1, Double_t *mypar, Double_t *emypar, int plot);
Double_t fitERF4(TH1F *h1, Double_t *mypar, Double_t *emypar, int plot=0);
Double_t fitERF4(TH1F *h1, float mi, float ma, float step, Double_t *mypar, Double_t *emypar, int plot=0); // fits the histogram with an erf function between min and max - the start parameters should be in mypar (Constant, Flex, RMS, Amplitude, Charge Sharing). It make an estimate about where the noise starts by using the starting parameter 3 (amplitude)
float calCh(int nen, Double_t *en, TH1F **hch, Double_t &gain, Double_t &off, int plot=0);
TF1 *fitERF4old(TH1F *h1, float mi, float ma, float step, Double_t *mypar, Double_t *emypar, int plot);
Double_t gaussChargeSharing(Double_t *x, Double_t *par); // gaussian function with pedestal
Double_t *fitGaussCS(TH1F *h1, float mi, float ma, float step, Double_t *mypar); // fits the spectrum h1 with a gaussian function with pedestal. the start parameters should be in mypar (to be improved)

float median(float *x, int n);//calculates the median value of the n-elements array x
int readrun_name(char fname[80], float *data) ;// reads the data file fname and fills the array data

TGraph *ReadTrimbits(char *fn);// reads a trimbit file and returns a graph of the trimbits
TGraph *ReadTrimbits(char *fn, int *vtrim);// reads a trimbit file and returns a graph of the trimbits and the value of vtrim, useful for calibration
TH1F *TrimbitsHisto(TGraph *gt);// returns an histogram of the distribution of the trimbits contained in the graph gt 

TH1F *StepStatistic(TH1F *step);// returns an histogram of the distribution of the counts of the step histogram
TH1F *Smooth(TH1F* hin);// returns a smoothed histogram


Double_t convenStandard(float i);//returns  the value in dac units of the energy i with standard settings (0 trimbits)
Double_t convdacStandard(float i); //returns  the energy corresponding to the dac units i with standard settings (0 trimbits)

Double_t convenFast(float i);//returns  the value in dac units of the energy i with fast settings (0 trimbits)
Double_t convdacFast(float i);//returns  the energy corresponding to the dac units i with fast settings (0 trimbits)
Double_t convenVeryFast(float i);//returns  the value in dac units of the energy i with very fast settings (0 trimbits)
Double_t convdacVeryFast(float i);//returns  the energy corresponding to the dac units i with very fast settings (0 trimbits)
Double_t convenHighGain(float i) ;//returns  the value in dac units of the energy i with high gain settings (0 trimbits)
Double_t convdacHighGain(float i) ;//returns  the energy corresponding to the dac units i with high gain settings (0 trimbits)
Double_t convdacTrimOffset(char *trimfile);// estimates the offset in dac units due to the trimfile
float convdac(float i);// converts from dac units to mv
float convmv(float i);// converts from mv to dac units 
void MakeHisto(TGraph *g, TH1F *h);// fills the already existing distribution histogram h with the points of the graph g
int scanHeader(char *fname, float *encoder, float *ic, float *t, int *settings);
int kth_smallest(int *a, int n, int k);
int quick_select(int arr[], int n);

int writeTrimFile(string fname, int* trims);
int readParab(char *fname, float &encoder, float &intensity);
angConvFactors* readAngCal(char *fname);

#endif
