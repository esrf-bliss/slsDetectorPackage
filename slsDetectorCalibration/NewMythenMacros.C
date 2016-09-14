#include "NewMythenMacros.h"

#define  TWOTHETAENCODER "X04SA-ES2-TH2:RO.RBV"
#define  IONISATION "X04SA-ES2-SC:CH6"

// sets nice plotting settings
int rootlogon ()
{
  gStyle->SetDrawBorder(0);
  gStyle->SetCanvasColor(kWhite);     
  gStyle->SetCanvasDefH(800);
  gStyle->SetCanvasDefW(800);
  gStyle->SetCanvasBorderMode(0);     
  gStyle->SetPadBorderMode(0);
  gStyle->SetPaintTextFormat("5.2f"); 
  gStyle->SetLineWidth(2);
  gStyle->SetTextSize(1.1);
  gStyle->SetLabelSize(0.04,"xy");
  gStyle->SetTitleSize(0.05,"xy");
  gStyle->SetTitleOffset(1.0,"x");
  gStyle->SetTitleOffset(1.6,"y");
  gStyle->SetPadTopMargin(0.05);
  gStyle->SetPadRightMargin(0.05);
  gStyle->SetPadBottomMargin(0.15);
  gStyle->SetPadLeftMargin(0.15);
  gStyle->SetLegendBorderSize(1);
  gStyle->SetFrameBorderMode(0);
  gStyle->SetFrameFillColor(kWhite);
  gStyle->SetTitleFillColor(kWhite);
  gStyle->SetStatFontSize(0.03);
  gStyle->SetStatBorderSize(1);
  gStyle->SetStatFormat("6.4g");
  gStyle->SetStatX(0.95);
  gStyle->SetStatY(0.95);
  gStyle->SetStatW(0.2);
  gStyle->SetStatH(0.2);
  gStyle->SetStatColor(kWhite);
  gStyle->SetTitleX(0.3);
  gStyle->SetTitleY(0.98);
  gStyle->SetTitleBorderSize(1);
  gStyle->SetTitleFontSize(0.06);
  gStyle->SetLegendBorderSize(1);
  gROOT->SetStyle("Default");
  gROOT->ForceStyle();          
  return 0;

}
/*
// reads a data file fname with nch channels and return an 1D histogram
TH1F *readDataFile(char* fname, int nch) {
  int ch,counts;
  TH1F *h = new TH1F("h1","",nch,-0.5,nch-0.5);

  FILE* fp=fopen(fname,"r");
  if (fp==NULL) {
    printf("Could not open file %s\n",fname);
    return h;
  }
  for (int j=0; j<nch; j++) {  
      fscanf(fp,"%d %d",&ch,&counts);
      //h->Fill(ch,counts);
      printf("%d %d %d\n", j, ch, counts);
      h->SetBinContent(j+1,counts);
  }
  fclose(fp);
  h->Sumw2();
  return h;
}
*/
// reads a data file fname with nch channels and return an 1D histogram
TH1F *readDataFile(char* fname, int nch) {
  int ch,counts;
  float fc;
  TH1F *h = new TH1F("h1","",nch,-0.5,nch-0.5);

  FILE* fp=fopen(fname,"r");
  if (fp==NULL) {
    printf("Could not open file %s\n",fname);
    delete h;
    return NULL;
  }
  for (int j=0; j<nch; j++) {  
      fscanf(fp,"%d %f",&ch,&fc);
      
      counts=fc;
      //h->Fill(ch,counts);
      //  printf("%d %d %d\n", j, ch, counts);
      h->SetBinContent(j+1,fc);
  }
  fclose(fp);
  h->Sumw2();
  return h;
}
// reads a data file fname with nch channels and return an TGraphErrors
TGraphErrors *readDatFile(char* fname, int er=1) {
  int ch=0;
  float ang, counts, err=0;
  TGraphErrors *g = new TGraphErrors();
  char line[1000], format[1000];
  if (er)
    strcpy(format,"%f %f %f\n");
  else
    strcpy(format,"%f %f\n");
    

  FILE* fp=fopen(fname,"r");
  if (fp==NULL) {
    printf("Could not open file %s\n",fname);
    return NULL;
  }
  while (fgets(line,1000,fp)) {
    sscanf(line,format,&ang,&counts,&err);
    //printf(format,ang,counts,err);
    g->SetPoint(ch,ang,counts);
    if (er==0)
      err=TMath::Sqrt(counts);
    g->SetPointError(ch,0.,err);
    ch++;
  }
  fclose(fp);
  return g;
}

// writes a 1D histogram into a data file fname (ch-value integer format)
int writeDataFile(char* fname, TH1 *hd) {
  FILE* fp=fopen(fname,"w");
  if (fp==NULL)
    return -1;
  int nx=hd->GetNbinsX();
  for (int j=0; j<nx; j++) {  
    fprintf(fp,"%d %d\n",j,hd->GetBinContent(j+1));
  }
  fclose(fp);
  return 0;
}

// writes a 2D histogram into a data file fname (one frame per line, float format)
int writeImage(char* fname, TH2 *hd) {
  FILE* fp=fopen(fname,"w");
  if (fp==NULL)
    return -1;
  int nx=hd->GetNbinsX();
  int ny=hd->GetNbinsY();

  for (int i=1; i<ny+1; i++) {
    for (int j=0; j<nx; j++) {  
      fprintf(fp,"%f ",hd->GetBinContent(j+1,i));
    }
    fprintf(fp, "\n");
  }
  fclose(fp);
  return 0;
}


// shifts the X axis of a 1D histogram of an offset delta
TH1F *translateHisto(TH1F *hin, float delta) {
  float min, max;
  int nb;

  nb=hin->GetNbinsX();
  min=hin->GetXaxis()->GetXmin();
  max=hin->GetXaxis()->GetXmax();

  TH1F *hout=(TH1F*)hin->Clone();
  hout->GetXaxis()->Set(nb,min+delta,max+delta);
  return hout;
}


// flat field correct the data in fndata using the ff file fnff and writing the results in fnout

TH1F* removeBad(TH1F *hdata, TH1F *hbad) {
  TH1F *hgood=(TH1F*)hdata->Clone();
  hgood->Multiply(hbad);
  return hgood;
}

TH1F *ffCorrect(TH1F *hdata, TH1F *hff, TH1F *hbad=NULL) {
  TH1F *hcorr;
  TH1F *hff1;
  if (hbad) {
    hff1=removeBad(hff,hbad);
    hcorr=removeBad(hdata,hbad);
  } else {
    hcorr=(TH1F*)hdata->Clone();
    hff1=(TH1F*)hff->Clone();
  }
  hcorr->Divide(hff1);
  hcorr->Scale(hff1->Integral()/hff->GetNbinsX());
  delete hff1;
  return hcorr;
}

int flatField(char* fndata, char *fnff, char *fnout, int nch) {
  TH1F *hin = readDataFile(fndata, nch);
  TH1F *hff = readDataFile(fnff, nch);
  TH1F *hout=ffCorrect(hin,hff);
  writeDataFile(fnout,hout);
  return 0;
}

/*
float angle(int channel, float enc) {
  int imod, channelmod;
  float ang;
  imod=channel/NCHMOD;
  channelmod=channel-imod*NCHMOD;
  if (angcalread) {
    ang = 2.404350+moffset[imod]+57.29577951*atan(( ((float) channelmod)-center[imod] )*conversion[imod])+enc+globaloff;
  } else {
    ang = 2.404350+imod*5.0+57.29577951*atan(( ((float) channelmod)-640.0 )*6.561E-5)+enc+globaloff;
  }
  return sign*ang;
}
*/


int writeTrimFile(string fname, int* trims){

  ofstream outfile;
  string names[]={"Vtrim", "Vthresh", "Rgsh1", "Rgsh2", "Rgpr", "Vcal", "outBuffEnable"};
  int iv, ichan, ichip;
  int iv1, idac;
  int nb;
    outfile.open(fname.c_str(), ios_base::out);

    if (outfile.is_open()) {
      for (idac=0; idac<6; idac++) {
	iv=-1;
	outfile << names[idac] << " " << iv << std::endl;
      }
      
      for (ichip=0; ichip<10; ichip++) {
	  iv1=0;
	  outfile << names[idac] << " " << iv1 << std::endl;
	  for (ichan=0; ichan<128; ichan++) {
	    outfile << trims[ichip*128+ichan] << " 1 0 0 0 0" << endl;
	  }
	}
      outfile.close();
      return 0;
    } else {
      std::cout<< "could not open trim file " << fname << std::endl;
      return -1;
    }

};






int readParab(char *fname, float &encoder, float &intensity) {

  float enc1, enc2, val;
   char temp[80], line[256];

   FILE *fp;
  int n, k, i1, i2, nchan, iarg;

   // read in parameter file
    fp = fopen(fname,"r");
    if (fp == NULL) {
      printf("can't open parameter file %s!\n",fname);
	return -1;
    } else {
      //data[nchan]
      encoder=0;
      i1=0;
      i2=0;
      enc1=0;
      enc2=0;
   
      // the following epics variables have to be changed to the values used
    while ( fgets(line,256,fp)!=NULL ) {
      //       printf("%s",line);
      if (strstr(line,TWOTHETAENCODER)!=NULL) {
	printf("enc %s %s\n",TWOTHETAENCODER,"%f");
	sprintf(temp,"%s %s\n",TWOTHETAENCODER,"%f");
	if (enc1==0) {
	 sscanf(line,temp,&enc1);
	} else {
	  if ( (enc1!=0) && (enc2==0) ) sscanf(line,temp,&enc2);
	}
	printf(line);
      }
      if (strstr(line,IONISATION)!=NULL) {
	printf("i0 %s %s\n",IONISATION,"%i");
	sprintf(temp,"%s %s\n",IONISATION,"%i");
	if (i1==0) {
	  sscanf(line,temp,&i1);
	} else {
	  if ( (i1!=0) && (i2==0) ) sscanf(line,temp,&i2);
	}
	printf(line);
      }
    }

    fclose(fp);
    
    intensity=(float) i1-i2;
    encoder=enc1;

	  
      
      
      


      printf("... found %f encoder reading ...\n",encoder);
      encoder=encoder;
      printf("... setting encoder to %f ...\n",encoder);
      printf("... i1 %i i2 %i  intensity %f ...\n",i1,i2,intensity);
    }



    return 0;




}














angConvFactors* readAngCal(char *fname) {
  int mod, modtmp;
  float centertmp, errcentertmp,conversiontmp,errconversiontmp,moffsettmp,errofftmp;
  FILE *fp;
// read file with angular calibration constants
  angConvFactors *convFactor=new angConvFactors[54];
  char line[1000];
  fp = fopen(fname,"r");
  if (fp == NULL) {
    printf("can't open parameter file !\n");
    exit(1);
  }
  mod=0;
  while (fgets(line, 1000, fp)) {
    if (convFactor[mod].Scan(line)) {
      printf(line);
      mod++;
    } else 
      printf("could not scan %s",line);
  }
  fclose (fp);
  return convFactor;
}




TH1F *readBad(char *badfname, int nch) {
  int n1,n2, nchan, nbad;
  char line[1000];
  TH1F *hbad=new TH1F("hbad","hbad",nch,-0.5,nch-0.5);
  FILE *fp;
  for (int ibin=1; ibin<nch+1; ibin++) {
    hbad->SetBinContent(ibin,1);
  }
  fp = fopen(badfname,"r");
  if (fp == NULL) {
    printf("can't open bad channel file !\n");
    return NULL;
  } else {
    while (fgets(line,1000,fp)) {
      if (sscanf(line,"%d-%d",&n1,&n2)==2) {
	for (int in=n1+1; in<n2+2; in++) {
	  hbad->SetBinContent(in,0);
	  nbad++;
	  printf("channel %i is bad ...\n", in);
	}
      } else if (sscanf(line,"%d",&n1)==1) {
	hbad->SetBinContent(n1+1,0);
	nbad++;
	printf("channel %i is bad ...\n", n1);
      } 
    }
    fclose(fp);
    printf("... found %i bad channels ...\n", nbad);
  }
  return hbad;
}

// Writes a file with the projection along y of a 2d histogram (visible part)
TH1F *createProjection(TH2F *h2, char *fout) {
  int ymin=h2->GetYaxis()->GetFirst();
  int ymax=h2->GetYaxis()->GetLast();
  int nb=h2->GetNbinsY();

  TH1D *hff = h2->ProjectionX("_px",ymin,ymax,"");
  hff->Scale(1./((float)nb));
  writeDataFile(fout,(TH1F*)hff);
  return (TH1F*)hff;
}


// Creates a 2D histogram from a set of files acquired with a threshold scan (fn is the path and file name root)
TH2F* createThrscan(char* fn, int mi, int ma, int step, int run, int nch, char *ext) {
  char fname[1000];
  TH1F *hdum;
  TH2F *h2 = new TH2F("h2","Thrscan",nch,-0.5,nch-0.5,((ma-mi)/step)+1,mi-(float)step/2.,ma+(float)step/2.);
  printf("2d histo created\n");
  h2->SetStats(kFALSE);
  for (int i=mi; i<ma+1; i=i+step) {
    sprintf(fname,"%s_adv%d_%d%s",fn,i,run,ext);
    printf("%s...",fname);
    hdum=readDataFile(fname, nch);
    printf("read\n");
    if (hdum) {
      for (int j=0; j<nch; j++)
	h2->Fill(j,i,hdum->GetBinContent(j+1));
      delete hdum;
	;
    }
  }
  h2->Sumw2();
  printf("Threshold scan creted\n");
  return h2;
}


// Creates a 2D histogram from a set of files acquired with a threshold scan (fn is the path and file name root)
TH2F* createThrscanNorm(char* fn, int mi, int ma, int step, int run, int nch) {
  char fname[100];
  char hname[100];
  float enc, ic, t, ictot=0;
  int sett;
  TH1F *hdum;
  TH2F *h2 = new TH2F("h2","Thrscan",nch,-0.5,nch-0.5,((ma-mi)/step)+1,mi-(float)step/2.,ma+(float)step/2.);
  h2->SetStats(kFALSE);
  for (int i=mi; i<ma+1; i=i+step) {
    sprintf(fname,"%s_adv%d_%d.raw",fn,i,run);
    sprintf(hname,"%s_adv%d_%d.parab",fn,i,run);
    hdum=readDataFile(fname, nch);
    scanHeader(hname, &enc, &ic, &t, &sett);
    
    ictot+=ic;
    for (int j=0; j<nch; j++) 
      if (ic>0) h2->Fill(j,i,hdum->GetBinContent(j+1)/ic);
    delete hdum;
  }
  h2->Scale(ictot);
  return h2;
}


// Creates a 2D histogram from a set of files (fn is the format of the file name)
TH2F* createScan(char* fn, int mi, int ma, int step, int nch) {
  char fname[10000];
  TH1F *hdum=NULL;
  TH2F *h2 = new TH2F("h2","Thrscan",nch,-0.5,nch-0.5,((ma-mi)/step)+1,mi-(Double_t)step/2.,ma+(Double_t)step/2.);
  h2->SetStats(kFALSE);
  //printf("h2 created\n");
  for (int i=mi; i<ma+1; i=i+step) {
    sprintf(fname,fn,i);
    printf(" l %i %s\n",i,fname);
    hdum=readDataFile(fname, nch);
    if (hdum) {
      for (int j=0; j<nch; j++)
	h2->Fill(j,i,hdum->GetBinContent(j+1));
      delete hdum;
    }
  }
  return h2;
}

// Creates a 2D histogram from a set of files (fn is the format of the file name)
TH2F* createScan(char* fn, float mi, float ma, float step, int nch) {
  char fname[100];
  TH1F *hdum;
  Int_t nstep=((ma-mi)/step)+1;
  TH2F *h2 = new TH2F("h2","Thrscan",nch,-0.5,nch-0.5,nstep,mi-step/2., ma+step/2.);
  h2->SetStats(kFALSE);
  for (float i=mi; i<ma+1; i=i+step) {
    sprintf(fname,fn,i);
    hdum=readDataFile(fname, nch);
    for (int j=0; j<nch; j++)
      h2->Fill(j,i,hdum->GetBinContent(j+1));
    delete hdum;
  }
  return h2;
}

// Flat field corrects the 2D histogram h2 with the data contained in the 2D histogram hff
TH2F*  normalize(TH2F* h2, TH2F *hff) {

  Int_t ix,iy,ymin,ymax,nbinsx, nbinsx1;
  Double_t tmp,fscale,mean;

  ymin=hff->GetYaxis()->GetFirst();
  ymax=hff->GetYaxis()->GetLast();
  nbinsx=h2->GetNbinsX();
  nbinsx1=hff->GetNbinsX();
  if (nbinsx!=nbinsx1) {
    printf("The size of the data and of the flat field histogram don't match!\n");
    return h2;
  }
  TH1D *hpx = new TH1D("_px","px",nbinsx,0,nbinsx);
   
 for (ix=1;ix<nbinsx+1;ix++) {
   mean=0;
   for (iy=1;iy<hff->GetNbinsY()+1;iy++) {
     mean=mean+hff->GetBinContent(ix,iy);
   }
   hpx->SetBinContent(ix, mean);
 }

 TH2F* hn = (TH2F*) h2->Clone();

 hpx->Scale(1.0/((float)(ymax-ymin+1)));

 mean=0.0;
 for (ix=1;ix<nbinsx+1;ix++) {
   mean=mean+hpx->GetBinContent(ix)/nbinsx;
 }
 for (ix=1;ix<nbinsx+1;ix++) {
   fscale=hpx->GetBinContent(ix);
   if (fscale!=0) fscale=mean/fscale;

   for (iy=1;iy<h2->GetNbinsY()+1;iy++) {
     tmp=h2->GetBinContent(ix,iy)*fscale;
     hn->SetBinContent(ix,iy,(float) tmp);
   }
 }
 return hn;
}

// resizes a 2D histogram between in a given region of interest
TH2F*  resize(TH2F* h2,Double_t  irowmin,Double_t  irowmax,Double_t  icolmin,Double_t  icolmax) {
  
 Int_t ix,iy;
 Int_t nbinsx, nbinsy;
 Double_t ymin,ymax, xmin,xmax, xstep, ystep;

  xmin=h2->GetXaxis()->GetXmin();
  xmax=h2->GetXaxis()->GetXmax();
  ymin=h2->GetYaxis()->GetXmin();
  ymax=h2->GetYaxis()->GetXmax();
  nbinsx=h2->GetNbinsX();
  nbinsy=h2->GetNbinsY();
  xstep=(xmax-xmin)/nbinsx;
  ystep=(ymax-ymin)/nbinsy;
  nbinsx=(irowmax-irowmin)/xstep+1;
  nbinsy=(icolmax-icolmin)/ystep+1;
  TH2F *hsmall=new TH2F("hsmall",h2->GetTitle(),nbinsx, irowmin,irowmax,nbinsy, icolmin, icolmax);
  for (ix=0; ix<nbinsx; ix++) {
    for (iy=0; iy<nbinsy; iy++) {
      hsmall->SetBinContent(ix+1, iy+1, h2->GetBinContent(ix+1+(irowmin-xmin)/xstep,iy+1+(icolmin-ymin)/ystep ));
    }
  }
  return hsmall;
}

// returns a 1D histogram corresponding to the channel chan
TH1F* getCh(TH2F* h2, int chan) {
  int nstep=h2->GetYaxis()->GetNbins();
  Double_t mi=h2->GetYaxis()->GetXmin();
  Double_t ma=h2->GetYaxis()->GetXmax();
  TH1F* h1=new TH1F("h1", "Channel",nstep,mi,ma);
  for (int i=1; i<nstep+1; i++)
    h1->SetBinContent(i, h2->GetBinContent(chan+1,i));
  return h1;
} 

// returns a 1D histogram corresponding to the step step
TH1F* getStep(TH2F* h2, int step) {
  TH1F* h1=new TH1F("h1", "Step",h2->GetNbinsX(),-0.5,h2->GetNbinsX()-0.5);
  int nstep=h2->GetNbinsX();
  for (int i=1; i<nstep+1; i++)
    h1->SetBinContent(i, h2->GetBinContent(i,step+1));
  return h1;
} 
// returns a 1D histogram corresponding to the step fstep in user's coordinates
TH1F* getStep(TH2F* h2, Double_t fstep) {
  TH1F* h1=new TH1F("h1", "Step",h2->GetNbinsX(),-0.5,h2->GetNbinsX()-0.5);
  int nstep=h2->GetYaxis()->GetNbins();
  Double_t mi=h2->GetYaxis()->GetXmin();
  Double_t ma=h2->GetYaxis()->GetXmax();

  int step=(fstep-mi)*nstep/(ma-mi);

  for (int i=1; i<h2->GetNbinsX()+1; i++)
    h1->SetBinContent(i, h2->GetBinContent(i,step+1));
  return h1;
} 



// returns the derivated histogram of a 1d histogram 
TH1F* spectrumFromCh(TH1F* hch) {
  int thr;
  float th,hi,lo,de, elo,ehi, eth;

  Int_t nst;
  Double_t mi,ma,st; 

  TAxis *ax=hch->GetXaxis();

  ma=ax->GetXmax();
  mi=ax->GetXmin();
  st=ax->GetBinWidth(1);
  nst=(ax->GetNbins());
  TH1F *hspectrum=new TH1F("hs", "Spectrum",nst-1,mi+0.5*st,ma-0.5*st);
  for (int i=1; i<nst; i++) {
    //thr=(hch->GetXaxis()->GetBinCenter(i)+hch->GetXaxis()->GetBinCenter(i+1))/2.;
    thr=mi+(0.5+i)*st;
    th=hch->GetBinContent(i+1);
    lo=hch->GetBinContent(i);
    de=(th-lo)/2;
    hspectrum->SetBinContent(i, de/st);
    elo=hch->GetBinError(i);
    if (elo<=0)
      elo=TMath::Sqrt(lo);
    eth=hch->GetBinError(i+1);
    if (eth<=0)
      eth=TMath::Sqrt(eth);
    hspectrum->SetBinError(i, TMath::Sqrt(eth*eth+elo*elo)/st);
  }
  return hspectrum;
}

// basic erf function
Double_t erfFunction(Double_t *x, Double_t *par) {
  return (par[0]+(par[3]/2.*(1+TMath::Erf((x[0]-par[1])/(TMath::Sqrt(2)*par[2])))));  
}
Double_t erfFunctionInv(Double_t *x, Double_t *par) {
  return (par[0]+(par[3]/2.*(1-TMath::Erf((x[0]-par[1])/(TMath::Sqrt(2)*par[2])))));  
}
// erf function with charge sharing
/*Double_t erfFunction3(Double_t *x, Double_t *par) {
  Double_t f;
  f=erfFunction(x, par)*(1+par[4]*(x[0]-par[1]));
  return f;										
}
*/
Double_t erfFunction3(Double_t *x, Double_t *par) {
  Double_t f;

  f=erfFunction(x, par)*(1+par[4]*(x[0]-par[1])/(par[2]));
  return f;										
}
Double_t saturationFunction(Double_t *x, Double_t *par) {

  return 1./(1+par[0]*x[0]*x[0]);
}

Double_t erfFunctionGeant(Double_t *x, Double_t *par) {
  Double_t f;

  if (x[0]>0 && x[0]<par[1])
    f=1+(TMath::ATan((par[1]-x[0])/par[5])-TMath::ATan((x[0])/par[5]))*2./TMath::Pi();
  return erfFunctionInv(x, par)*(1+par[4]*(par[1]-x[0]))*f;										
}

Double_t erfFunctionAll(Double_t *x, Double_t *par) {
  Double_t f;
  Double_t dp[4];
  dp[0]=par[0];
  dp[1]=par[6];
  //dp[2]=par[7];
  dp[2]=par[2];
  dp[3]=par[5];
  Double_t y=-1*x[0];
  f=erfFunction(x, par)*(1+par[4]*(x[0]-par[1])/(par[2]))+erfFunction(x, dp);
  return f;										
}

Double_t erfFunctionAllInv(Double_t *x, Double_t *par) {
  Double_t f;
  Double_t dp[4];
  dp[0]=par[0];
  dp[1]=0;
  //dp[2]=par[7];
  dp[2]=par[2];
  dp[3]=par[5];
  Double_t y=-1*x[0];
  f=erfFunctionInv(x, par)*(1-par[4]*(x[0]-par[1])/(par[2]))+erfFunctionInv(x, dp);
  return f;										
}

Double_t spectrumAllInv(Double_t *x, Double_t *par) {
  Double_t f;
  Double_t y=-1*x[0];
  f=par[3]*TMath::Exp(-1*(x[0]-par[1])*(x[0]-par[1])/(2*par[2]*par[2]));
  f=f+par[4]*(par[3]/2*(TMath::Erfc((x[0]-par[1])/(TMath::Sqrt(2.)*par[2]))));
  f+=par[5]*TMath::Exp(-1*(x[0])*(x[0])/(2*par[2]*par[2]));
  return f;										
}

Double_t erfFuncFluo(Double_t *x, Double_t *par) {
  Double_t f;

  f=erfFunction3(x, par)+erfFunction3(x, par+5);
  return f;										
}

// erf function with charge sharing
Double_t erfFunction3inv(Double_t *x, Double_t *par) {
  Double_t f, y;
    f=erfFunctionInv(x, par)*(1-par[4]*(x[0]-par[1]));
  return f;										
}

Double_t erfFuncFluoInv(Double_t *x, Double_t *par) {
  Double_t f;

  f=erfFunction3inv(x, par)+erfFunction3inv(x, par+5);
  return f;										
}



// test func

Double_t doubleExponential(Double_t *u, Double_t *par) {
  Double_t f=1.;
  f=TMath::Exp(TMath::Exp(-1.*u[0]));
  return f;										
}


Double_t levyCumulative(Double_t *x, Double_t *par) {
  Double_t f=1.;
  if (x[0]>0)
    f=TMath::Erfc(TMath::Sqrt(0.5*par[0]/(x[0])));//erfFunctionInv(x, par)*(1-par[4]*(x[0]-par[1]));
  else
    f=0;
  return f;										
}

Double_t logNormCumulative(Double_t *x, Double_t *par) {
  Double_t f=1.;
  if (x[0]>par[1])
    f=0.5*(1.+TMath::Erf(0.5*(TMath::Sqrt(x[0])-0.5*par[0]*par[0])/par[0]));//erfFunctionInv(x, par)*(1-par[4]*(x[0]-par[1]));
  else
    f=0;
  return f;										
}

Double_t levyCumulative1(Double_t *x, Double_t *par) {
  Double_t f=1.;

  Double_t y=par[1]-x[0];
  f=par[2]*levyCumulative(&y,par);
  return f;										
}

Double_t test(Double_t *x, Double_t *par) {
  Double_t f=1.;
  Double_t y=par[1]-x[0];
  //f=erfFunctionInv(x, par)*(1.-levyCumulative(x,par+5)+levyCumulative(&y,par+5));
  f=1.-logNormCumulative(x, par+5)+logNormCumulative(&y, par+5);
  return f;										
}











Double_t fitERF4(TH1F *h1, Double_t *mypar, Double_t *emypar, int plot) {
  float mi=h1->GetXaxis()->GetXmin();
  float ma=h1->GetXaxis()->GetXmax();
  float step=h1->GetXaxis()->GetBinWidth(1);
  return fitERF4(h1, mi, ma, step,mypar, emypar, plot);
}



// fits the histogram with an erf function between min and max - the start parameters should be in mypar (Constant, Flex, RMS, Amplitude, Charge Sharing). It make an estimate about where the noise starts by using the starting parameter 3 (amplitude)
Double_t fitERF4(TH1F *h1, float mi, float ma, float step, Double_t *mypar, Double_t *emypar, int plot) {
  TF1* fitfun;
  int nst=((ma-mi)/step)+1;
  float ma1;
  float chi=100.;
  int ist, mst, nb;  
  float xdum[5], ydum[5];
  
  /*  TGraph *gdum=new TGraph();
  float fdum;
  int zeros;

  nb=h1->GetNbinsX()+1;
  ma1=-1;
  mypar[1]=-1;



  for (ist=1; ist<nb-11; ist++) {
    zeros=0;
    for (int ib=0; ib<11;ib++) {
      gdum->SetPoint(ib,h1->GetXaxis()->GetBinCenter(ist+ib),h1->GetBinContent(ist+ib));
      if (h1->GetBinContent(ist+ib)==0)
	zeros=1;
    }
    if (zeros==0) {
    gdum->Fit("pol1","0Q");
    fdum=h1->GetBinContent(ist+5);
    if (fdum==0)
      fdum=1;
    if (gdum->GetFunction("pol1")->GetParameter(1)/TMath::Sqrt(fdum)>10) {
      if (ma1==-1) {
	ma1=h1->GetXaxis()->GetBinCenter(ist+5);
	mypar[3]=h1->GetBinContent(ist);
      }
    }
    if (gdum->GetFunction("pol1")->GetParameter(1)/TMath::Sqrt(fdum)>2) {
      if (mypar[1]==-1) {
	mypar[1]=h1->GetXaxis()->GetBinCenter(ist+5);
      }
    }

    }

  }
  delete gdum;
  */

  ma1=h1->GetMean()-30;
  // mypar[1]=ma1-50;
  mypar[3]=h1->GetBinContent(h1->GetXaxis()->FindBin(ma1-30));

  TF1 *fun1 = new TF1("fun1",erfFunction3,mi,ma1,5);
  fun1->SetParameters(mypar); 
  fun1->SetParameter(0,0.);
  fun1->SetParLimits(0,0.,10.);
  //  fun1->SetParLimits(1,mypar[1]-300.,ma1); 
  fun1->SetParameter(2,10.);
  fun1->SetParLimits(2,0.,100.);
  //fun1->SetParLimits(3,mypar[3]/50.,mypar[3]*50.);  
  fun1->SetParLimits(4,0.,1.);
  fun1->SetParameter(4,0.1);
  fun1->SetParNames("Constant", "Flex", "RMS", "Amplitude", "Charge Sharing");
  fun1->SetRange(mi,ma1);

  if (plot) {
    cout << "start parameters = " << endl;
    cout << "Constant " << mypar[0] <<endl;
    cout << "Flex " << mypar[1] <<endl;
    cout << "RMS " << mypar[2] <<endl;
    cout << "Amplitude " << mypar[3] <<endl;
    cout << "Charge Sharing " << mypar[4] <<endl;
    
  }

  if (plot) {
    h1->Fit("fun1","R");
  } else
    h1->Fit("fun1","R0Q");
  fitfun= h1->GetFunction("fun1");
  fitfun->GetParameters(mypar);
  emypar[1]=fitfun->GetParError(1);
  emypar[0]=fitfun->GetParError(0);
  emypar[2]=fitfun->GetParError(2);
  emypar[3]=fitfun->GetParError(3);
  emypar[4]=fitfun->GetParError(4);
  chi=fitfun->GetChisquare()/(fitfun->GetNDF());
  delete fun1;
  if (plot)
    printf("Chi square =%e\n",chi);
  return chi;
}




float calCh(int nen, Double_t *en, TH1F **hch, Double_t &gain, Double_t &off, int plot) {

  float chi;
  Double_t mypar[6], epar[6];
  Double_t flex[nen], eflex[nen];
  char flags[]="0Q";
  TCanvas *c;
  if (plot>0) {
    c=new TCanvas();
  }
  for (int ien=0; ien<nen; ien++) {
    mypar[1]=hch[ien]->GetMean()-10.*en[ien]; 
    fitERF4(hch[ien], mypar, epar, plot);
    flex[ien]=mypar[1];
    eflex[ien]=epar[1];
  }  
  if (plot>0) {
    c->Divide(2,nen/2+nen%2);
      
    char tit[100];
    for (int ien=0; ien<nen; ien++) {
      sprintf(tit,"Energy=%f keV",en[ien]);
      hch[ien]->SetTitle(tit);
      c->cd(ien+1);
      hch[ien]->DrawCopy();
    }
    c=new TCanvas();
    strcpy(flags,"");
  }
    
  TGraphErrors *glin=new TGraphErrors(nen,en,flex,0,eflex);
  glin->Fit("pol1", flags);  
  if (plot>0) {
    c->cd();
    glin->SetMarkerStyle(20);
    glin->Draw("ALP");
      
  }
  gain=-1*glin->GetFunction("pol1")->GetParameter(1);
  off=glin->GetFunction("pol1")->GetParameter(0);
  chi=glin->GetFunction("pol1")->GetChisquare()/(glin->GetFunction("pol1")->GetNDF());
  cout << " chi " << chi << " gain " << gain << " off " << off << endl;
  if (plot==0)
    delete glin;
  return chi;
}



// fits the histogram with an erf function between min and max - the start parameters should be in mypar (Constant, Flex, RMS, Amplitude, Charge Sharing). It make an estimate about where the noise starts by using the starting parameter 3 (amplitude)
TF1 *fitERF4old(TH1F *h1, float mi, float ma, float step, Double_t *mypar, Double_t *emypar, int plot) {
  TF1* fitfun;
  Double_t *epar;
  int nst=((ma-mi)/step)+1;
  float ma1;
  Double_t chi=100., chiold=1000.;
  int last=0, first=1;
  ma1=ma;
  int ist, mst;
  
  for (ist=1; ist<h1->GetNbinsX()+1; ist++) {
    if (h1->GetBinContent(ist)>10*mypar[3]) {
      ma1=mi+ist*step-5;
      mst=ist;
      break;
    }
  }
  //printf("%f %f %f\n",mypar[3],h1->GetBinContent(mst), ma1);
  ma1=mi+ist*step-20;
  mst=ist;

  TF1 *fun1 = new TF1("fun1",erfFunction3,mi,ma1,5);
  fun1->SetParameters(mypar); 
  fun1->FixParameter(0,0.);
  //fun1->SetParLimits(1,mypar[1]-300.,mypar[1]+200.);
  fun1->SetParLimits(1,mypar[1]-300.,ma1);
  fun1->SetParLimits(2,1.,50.);
  fun1->SetParLimits(3,mypar[3]/50.,mypar[3]*50.);  
  fun1->SetParLimits(4,0.,1.);
  fun1->SetParNames("Constant", "Flex", "RMS", "Amplitude", "Charge Sharing");

    if (ma1<mypar[1]) {
      //ma1=mypar[1]+20;
      mypar[1]=ma1-20;
      fun1->SetParLimits(1,mypar[1]-300.,ma1);
      //last=1;
      //printf("too low max!\n");
    }
    fun1->SetRange(mi,ma1);
    if (plot) 
      h1->Fit("fun1","R");
    else
      h1->Fit("fun1","R0Q");
    fitfun= h1->GetFunction("fun1");
    chi=fitfun->GetChisquare()/(fitfun->GetNDF());
  fitfun->GetParameters(mypar);
  epar=fitfun->GetParErrors();
  emypar[1]=epar[1];
  delete fun1;
  return fitfun;  
}

// fits the histogram with an erf function between min and max - the start parameters should be in mypar (Constant, Flex, RMS, Amplitude, Charge Sharing). It make an estimate about where the noise starts by using the starting parameter 3 (amplitude)
TF1 *fitERFAll(TH1F *h1, Double_t *mypar, Double_t *emypar, int plot) {
  TF1* fitfun;
  Double_t *epar;
  float cold=0.;
  int i;
  for (i=h1->GetNbinsX(); i>0; i--) {
    if (h1->GetBinContent(i)<cold) break;
    cold=h1->GetBinContent(i);
  } 
  mypar[7]=mypar[2];
  TF1 *fun1 = new TF1("fun1",erfFunctionAll,h1->GetXaxis()->GetXmin(),h1->GetXaxis()->GetBinCenter(i),7);
  fun1->SetParameters(mypar); 
  fun1->FixParameter(0,0.);
  //fun1->SetParLimits(1,mypar[1]-300.,mypar[1]+200.);
  fun1->SetParLimits(1,mypar[1]-300.,mypar[1]+300);
  fun1->SetParLimits(2,1.,20.);
  fun1->SetParLimits(3,mypar[3]/50.,mypar[3]*50.);  
  fun1->SetParLimits(4,0.,10.);
  fun1->SetParLimits(5,1E+4,1E+10);
  fun1->SetParLimits(6,700.,800.);
  // fun1->SetParLimits(7,1.,50.);

  //  fun1->SetParNames("Constant", "Flex", "ENC", "Amplitude", "Charge Sharing","Noise","Baseline","ENC2");
  fun1->SetParNames("Constant", "Flex", "ENC", "Amplitude", "Charge Sharing","Noise","Baseline");

  if (plot) {
    h1->SetStats(kFALSE);
      h1->Fit("fun1","R");
    } else
      h1->Fit("fun1","R0Q");
    fitfun= h1->GetFunction("fun1");
    fun1->SetRange(h1->GetXaxis()->GetXmin(),fitfun->GetParameter(6)-5*fitfun->GetParameter(2));
    if (plot) {
      h1->SetStats(kFALSE);
      h1->Fit("fun1","RL");
    } else
      h1->Fit("fun1","R0Q");
    fitfun= h1->GetFunction("fun1");
    if (plot)
      printf("%f  %f OK!\n", h1->GetXaxis()->GetBinCenter(i), fitfun->GetChisquare()/(fitfun->GetNDF()));
    fitfun->GetParameters(mypar);
    epar=fitfun->GetParErrors();
    emypar[1]=epar[1];
    delete fun1;
    return fitfun;  
}






// gaussian function with pedestal
Double_t gaussChargeSharing(Double_t *x, Double_t *par) {
  Double_t f;
  f=par[3]*TMath::Exp(-1*(x[0]-par[1])*(x[0]-par[1])/(2*par[2]*par[2]));
  f=f+par[0]*(par[3]/2*(TMath::Erfc((par[1]-x[0])/(TMath::Sqrt(2.)*par[2]))));
  return f;                                       
}

// fits the spectrum h1 with a gaussian function with pedestal. the start parameters should be in mypar (to be improved)
Double_t *fitGaussCS(TH1F *h1, float mi, float ma, float step, Double_t *mypar) {
  TF1 *fun1 = new TF1("fun1",gaussChargeSharing,mi,ma,4);
  fun1->SetParameters(mypar);        
  fun1->FixParameter(0,0.);
  fun1->SetParLimits(0,0.,.5);
  fun1->SetParLimits(1,mypar[1]-100,mypar[1]+200.);
  fun1->SetParLimits(2,1.,100.);
  fun1->SetParLimits(3,mypar[3]/10.,mypar[3]*50.); 
  fun1->SetParNames("Charge Sharing","Flex", "RMS", "Amplitude");
  h1->Fit("fun1","R");
  TF1 *fitfun= h1->GetFunction("fun1");
  fitfun->GetParameters(mypar);
  return mypar;  
}

//calculates the median value of the n-elements array x
float median(float *x, int n) {
  // sorts x into xmed array and returns median 
  // n is number of values already in the xmed array
  float *xmed=new float[n];
  int k,i,j;
  float v;

  for (i=0; i<n; i++) {
    k=0;
    for (j=0; j<n; j++) {
      if(*(x+i)>*(x+j))
	k++;
      if (*(x+i)==*(x+j)) {
	if (i>j) 
	  k++;
      }
    }
    xmed[k]=*(x+i);
  }
  k=n/2;
  v=xmed[k];
  delete [] xmed;
  return v;
}


// reads the data file fname and fills the array data
int readrun_name(char fname[80], float *data) {

    int n, k;
    int nchannel;
    FILE *fp;

    printf("file with input data         : %s\n",fname);

    // read in data file
    fp = fopen(fname,"r");
    if (fp == NULL) {
	printf("can't open data file !\n");
	return -1;
    }    

    nchannel = 0;
    while ( fscanf(fp,"%d %d\n",&n,&k)!=EOF ) {
	*(data+nchannel)= (float) k;
	nchannel++;
    }
    fclose(fp);
    printf("... found %i data ...\n\n",nchannel);


    return nchannel;
}

// reads a trimbit file and returns a graph of the trimbits
TGraph *ReadTrimbits(char *fn) {
  int ch[1280], trims[1280];
  char line[100];
  TGraph *gt;
  FILE *fp=fopen(fn,"r");
  for (int i=0;i<6;i++) {
    if ( fgets(line,255,fp)!=NULL ) {
      printf("%s",line);
    } else {
      printf("\n\n error reading initialization file: dac %d  not found !!\n\n",i);
      return gt;
      }
  }

      for (int ichip=0;ichip<NCHIP;ichip++) {
	  if ( fgets(line,256,fp)!=NULL ) {
	    //printf("%s",line);
	  } else {
	    printf("\n\n error reading initialization file: outbuffEnable %i not found !!\n\n",ichip);
	    return gt;
	  } 

	  
	  
	  for (int ichannel=0;ichannel<NCHAN;ichannel++) {
	    ch[ichip*NCHAN+ichannel]=ichip*NCHAN+ichannel;
	    if ( fgets(line,256,fp)!=NULL )
	      sscanf(line,"%i ",trims+ichip*NCHAN+ichannel);
	    else {
	      printf("\n\n error reading initialization file: not enough values for chip %i channel %i  !!\n\n",ichip,ichannel);
	      return gt;
	    }
	  }
      }
      fclose(fp);
      gt = new TGraph(1280, ch, trims);

      return gt;
}


// reads a trimbit file and returns a graph of the trimbits and the value of vtrim, useful for calibration
TGraph *ReadTrimbits(char *fn, int *vtrim) {

  int ch[1280], trims[1280];
  char line[100];
  TGraph *gt;
  FILE *fp=fopen(fn,"r");

    if ( fgets(line,255,fp)!=NULL )
      sscanf(line,"Vtrim %d",vtrim);

  for (int i=0;i<5;i++) {
    if ( fgets(line,255,fp)!=NULL ) {
      printf("%s",line);
    } else {
      printf("\n\n error reading initialization file: dac %d  not found !!\n\n",i);
      return gt;
      }
  }

      for (int ichip=0;ichip<NCHIP;ichip++) {
	  if ( fgets(line,256,fp)!=NULL ) {
	    printf("%s",line);
	  } else {
	    printf("\n\n error reading initialization file: outbuffEnable %i not found !!\n\n",ichip);
	    return gt;
	  } 
	  for (int ichannel=0;ichannel<NCHAN;ichannel++) {
	    ch[ichip*NCHAN+ichannel]=ichip*NCHAN+ichannel;
	    if ( fgets(line,256,fp)!=NULL ) {
	      printf("%d %d %s",ichip,ichannel,line);
	      sscanf(line,"%i ",trims+ichip*NCHAN+ichannel);
	   } else {
	      printf("\n\n error reading initialization file: not enough values for chip %i channel %i  !!\n\n",ichip,ichannel);
	      return gt;
	    }
	  }
      }
	
      fclose(fp);
      gt = new TGraph(1280, ch, trims);

      return gt;
}

// returns an histogram of the distribution of the trimbits contained in the graph gt 
TH1F *TrimbitsHisto(TGraph *gt) {
  TH1F *ht=new TH1F("ht", "Trims histo", 64, -0.5,63.3);
  Double_t x,y;
  for (int i=0; i<gt->GetN(); i++) {
    gt->GetPoint(i,x,y);
    printf("%i %f %f\n",i,x,y);
    ht->Fill(y);
  }
  return ht;
}

// returns an histogram of the distribution of the counts of the step histogram
TH1F *StepStatistic(TH1F *step) {
  Double_t f;
  TH1F *et=new TH1F("et","Counts distribution",100,step->GetMinimum(),step->GetMaximum());
  for (int i=1; i<step->GetNbinsX(); i++) {
    f=step->GetBinContent(i);
    et->Fill(f);
  }
  et->SetStats(kFALSE);
  return et;
}

// returns a smoothed histogram
TH1F *Smooth(TH1F* hin) {

  int k=3;
  int nbins=hin->GetNbinsX();
  TH1F *hout=(TH1F*)hin->Clone();
  Double_t mean=0, err=0;
  for (int i=k; i<nbins-k; i++) {
    mean=0;
    err=0;
    for (int j=0; j<2*k; j++) {
      mean=mean+hin->GetBinContent(i-k+j+1)/(2*k);
      err=err+hin->GetBinError(i-k+j+1)/(2*k);
    }
    hout->SetBinContent(i+1, mean);
    hout->SetBinError(i+1, err);
  }
  return hout;
} 

//returns  the value in dac units of the energy i with standard settings (0 trimbits)
Double_t convenStandard(float i) {
  float conven=1000./3.6;
  float el=1.67E-4;
  Double_t off=491.; /*mv*/
  Double_t gain=160.; /* mv/fc*/
  Double_t gain1=convmv(gain)*conven*el; /* dac/kev*/
  Double_t off1=convmv(off); /* dac*/

  return (off1-i)/gain1;

}

//returns  the energy corresponding to the dac units i with standard settings (0 trimbits)
Double_t convdacStandard(float i) {
  float conven=1000./3.6;
  float el=1.67E-4;
  Double_t off=495.; /*mv*/
  Double_t gain=152.; /* mv/fc*/
  Double_t gain1=convmv(gain)*conven*el; /* dac/kev*/
  Double_t off1=convmv(off); /* dac*/


  return off1-gain1*i;

}

//returns  the value in dac units of the energy i with fast settings (0 trimbits)
Double_t convenFast(float i) {
  float conven=1000./3.6;
  float el=1.67E-4;
  Double_t off=497; /*mv*/
  Double_t gain=126; /* mv/fc*/
  Double_t gain1=convmv(gain)*conven*el; /* dac/kev*/
  Double_t off1=convmv(off); /* dac*/
  return (off1-i)/gain1;

}

//returns  the energy corresponding to the dac units i with fast settings (0 trimbits)
Double_t convdacFast(float i) {
  float conven=1000./3.6;
  float el=1.67E-4;
  Double_t off=497; /*mv*/
  Double_t gain=126; /* mv/fc*/
  Double_t gain1=convmv(gain)*conven*el; /* dac/kev*/
  Double_t off1=convmv(off); /* dac*/
  return off1-gain1*i;

}
//returns  the value in dac units of the energy i with very fast settings (0 trimbits)
Double_t convenVeryFast(float i) {
  float conven=1000./3.6;
  float el=1.67E-4;
  Double_t off=506.3; /*mv*/
  Double_t gain=90.55; /* mv/fc*/
  Double_t gain1=convmv(gain)*conven*el; /* dac/kev*/
  Double_t off1=convmv(off); /* dac*/
  return (off1-i)/gain1;

}


//returns  the energy corresponding to the dac units i with very fast settings (0 trimbits)
Double_t convdacVeryFast(float i) {
  float conven=1000./3.6;
  float el=1.67E-4;
  Double_t off=506.3; /*mv*/
  Double_t gain=90.55; /* mv/fc*/
  Double_t gain1=convmv(gain)*conven*el; /* dac/kev*/
  Double_t off1=convmv(off); /* dac*/
  return off1-gain1*i;

}

//returns  the value in dac units of the energy i with high gain settings (0 trimbits)
Double_t convenHighGain(float i) {
  float conven=1000./3.6;
  float el=1.67E-4;
  Double_t off=487; /*mv*/ //480.87
  Double_t gain=207; /* mv/fc*/ //190?
  Double_t gain1=convmv(gain)*conven*el; /* dac/kev*/
  Double_t off1=convmv(off); /* dac*/
  return (off1-i)/gain1;

}
//returns  the energy corresponding to the dac units i with high gain settings (0 trimbits)
Double_t convdacHighGain(float i) {
  float conven=1000./3.6;
  float el=1.67E-4;
  Double_t off=487; /*mv*/
  Double_t gain=207; /* mv/fc*/
  Double_t gain1=convmv(gain)*conven*el; /* dac/kev*/
  Double_t off1=convmv(off); /* dac*/
  return off1-gain1*i;

}

// estimates the offset in dac units due to the trimfile
Double_t convdacTrimOffset(char *trimfile) {
  int vtrim;
  Double_t mean, offset, c=52.430851, b=-0.102022, a=0.000050;
  TGraph *trims=ReadTrimbits(trimfile, &vtrim);
  mean=trims->GetMean(2);
  printf("Vtrim=%d, mean=%f\n",vtrim,mean);
  offset=mean*(c+b*vtrim+a*vtrim*vtrim);
  return offset;
}

// converts from dac units to mv
float convdac(float i) {
  float DAC_MAX=1.940;
  float DAC_DR=1024.;
  float VTHRESH_R1=10.;
  float VTHRESH_R2=4.7;
  return 1000.*(((DAC_MAX*i/DAC_DR)*VTHRESH_R2)/(VTHRESH_R1+VTHRESH_R2));
}
// converts from mv to dac units 
float convmv(float i) {
  float DAC_MAX=1.940;
  float DAC_DR=1024.;
  float VTHRESH_R1=10.;
  float VTHRESH_R2=4.7;

  return i*(VTHRESH_R1+VTHRESH_R2)/(1000.*(DAC_MAX/DAC_DR)*VTHRESH_R2);
}

// fills the already existing distribution histogram h with the points of the graph g
void MakeHisto(TGraph *g, TH1F *h) {
  Double_t x,y;
  for (int i=0;i<g->GetN(); i++) {
    g->GetPoint(i,x,y); 
    h->Fill(y);
  }
  return;
}

int scanHeader(char *fname, float *encoder, float *ic, float *t, int *settings) {
  char line[200], format[100], sett[10];
  int res=-1;
  float ic1,ic2;
  *encoder=360;
  *ic=-1;
  *t=-1;
  *settings=-1;

  ic1=0;
  ic2=0;


  FILE *fp;

  fp=fopen(fname,"r");
  if (fp==NULL) {
    printf("Could not open header file %s\n", fname);
    return res;
  }
  while (fgets(line,200,fp)) {
    //printf(line);
    if (strstr(line,"X04SA-ES2-TH2:RO.RBV")!=NULL) {
      sscanf(line,"X04SA-ES2-TH2:RO.RBV %f", encoder);
      res=0;
    }
    if (strstr(line,"X04SA-ES2-SC:CH6")!=NULL) {
      if (ic1==0) {
	sscanf(line,"X04SA-ES2-SC:CH6 %f", &ic1);
	*ic=ic1;
      } else  if (ic2==0) {
	sscanf(line,"X04SA-ES2-SC:CH6 %f", &ic2);
	*ic=ic2-ic1;
      }	
    }
    if (strstr(line,"acquisition time =")!=NULL) {
      sscanf(line,"acquisition time = %f", t);
      *t=(*t)*1E9;
    }
    if (strstr(line,"mythen gain mode =")!=NULL) {
      sscanf(line,"mythen gain mode = %s", sett);
      if (strstr(sett,"standard")!=NULL)
	*settings=1;
      else if (strstr(sett,"fast")!=NULL)
	*settings=2;
      else if (strstr(sett,"highgain")!=NULL)
	*settings=3;
      else
	printf("Found unknown settings %s\n", sett);
    }    
  }
  fclose(fp);
  return res;
}


int quick_select(int arr[], int n) 
{
    int low, high ;
    int median;
    int middle, ll, hh;

    low = 0 ; high = n-1 ; median = (low + high) / 2;
    for (;;) {
        if (high <= low) /* One element only */
            return arr[median] ;

        if (high == low + 1) {  /* Two elements only */
            if (arr[low] > arr[high])
                ELEM_SWAP(arr[low], arr[high]) ;
            return arr[median] ;
        }

    /* Find median of low, middle and high items; swap into position low */
    middle = (low + high) / 2;
    if (arr[middle] > arr[high])    ELEM_SWAP(arr[middle], arr[high]) ;
    if (arr[low] > arr[high])       ELEM_SWAP(arr[low], arr[high]) ;
    if (arr[middle] > arr[low])     ELEM_SWAP(arr[middle], arr[low]) ;

    /* Swap low item (now in position middle) into position (low+1) */
    ELEM_SWAP(arr[middle], arr[low+1]) ;

    /* Nibble from each end towards middle, swapping items when stuck */
    ll = low + 1;
    hh = high;
    for (;;) {
        do ll++; while (arr[low] > arr[ll]) ;
        do hh--; while (arr[hh]  > arr[low]) ;

        if (hh < ll)
        break;

        ELEM_SWAP(arr[ll], arr[hh]) ;
    }

    /* Swap middle item (in position low) back into correct position */
    ELEM_SWAP(arr[low], arr[hh]) ;

    /* Re-set active partition */
    if (hh <= median)
        low = ll;
        if (hh >= median)
        high = hh - 1;
    }
}


int kth_smallest(int *a, int n, int k)
{
    register int i,j,l,m ;
    register float x ;

    l=0 ; m=n-1 ;
    while (l<m) {
        x=a[k] ;
        i=l ;
        j=m ;
        do {
            while (a[i]<x) i++ ;
            while (x<a[j]) j-- ;
            if (i<=j) {
                ELEM_SWAP(a[i],a[j]) ;
                i++ ; j-- ;
            }
        } while (i<=j) ;
        if (j<k) l=i ;
        if (k<i) m=j ;
    }
    return a[k] ;
}

