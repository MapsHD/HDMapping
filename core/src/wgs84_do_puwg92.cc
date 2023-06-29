/*
Autor: Zbigniew Szymanski
E-mail: z.szymanski@szymanski-net.eu
Wersja: 1.1 
Historia zmian:
		1.1 dodano przeksztalcenie odwrotne PUWG 1992 ->WGS84
		1.0 przeksztalcenie WGS84 -> PUWG 1992
Data modyfikacji: 2012-11-27
Uwagi: Oprogramowanie darmowe. Dozwolone jest wykorzystanie i modyfikacja 
       niniejszego oprogramowania do wlasnych celow pod warunkiem 
       pozostawienia wszystkich informacji z naglowka. W przypadku 
       wykorzystania niniejszego oprogramowania we wszelkich projektach
       naukowo-badawczych, rozwojowych, wdrozeniowych i dydaktycznych prosze
       o zacytowanie nastepujacego artykulu:
       
       Zbigniew Szymanski, Stanislaw Jankowski, Jan Szczyrek, 
       "Reconstruction of environment model by using radar vector field histograms.",
       Photonics Applications in Astronomy, Communications, Industry, and 
       High-Energy Physics Experiments 2012, Proc. of SPIE Vol. 8454, pp. 845422 - 1-8,
       doi:10.1117/12.2001354
       
Literatura:
       Uriasz, J., “Wybrane odwzorowania kartograficzne”, Akademia Morska w Szczecinie,
       http://uriasz.am.szczecin.pl/naw_bezp/odwzorowania.html
*/

#include <math.h>
#include <complex>
using namespace std;

int wgs84_do_puwg92(double B_stopnie, double L_stopnie, double *Xpuwg, double *Ypuwg)
/*
Opis:
    konwersja wspolrzednych z ukladu WGS 84 do ukladu PUWG 1992
Parametry:
    B_stopnie - szerokosc geograficzna wyrazona w stopniach
    L_stopnie - dlugosc geograficzna wyrazona w stopniach
    Xpuwg - wskazanie na wspolrzedna X ukladu PUWG 1992 (UWAGA - wspolrzedna pionowa)
    Ypuwg - wskazanie na wspolrzedna Y ukladu PUWG 1992 (UWAGA - wspolrzedna pozioma)
Zwracana wartosc:
    0 - konwersja powiodla sie
    1 - szerokosc geograficzna B poza zakresem
    2 - dlugosc geograficzna L poza zakresem
*/
    {
    // Parametry elipsoidy GRS-80
    double e=0.0818191910428;  	//pierwszymimo¶ród elipsoidy
    double R0=6367449.14577; 		//promieñ sfery Lagrange.a
    double Snorm=2.0E-6;   		//parametr normuj±cy
    double xo=5760000.0; 		//parametr centruj±cy
    
    //Wspolczynniki wielomianu
    double a0=5765181.11148097;
    double a1=499800.81713800;
    double a2=-63.81145283;
    double a3=0.83537915;
    double a4=0.13046891;
    double a5=-0.00111138;
    double a6=-0.00010504;
    
    // Parametry odwzorowania Gaussa-Kruegera dla uk³adu PUWG92
    double L0_stopnie=19.0; 		//Pocz±tek uk³adu wsp. PUWG92 (d³ugo¶æ)
    double m0=0.9993;
    double x0=-5300000.0;
    double y0= 500000.0;
    
    // Zakres stosowalnosci metody
    double Bmin=48.0*M_PI/180.0;
    double Bmax=56.0*M_PI/180.0;
    double dLmin=-6.0*M_PI/180.0;
    double dLmax=6.0*M_PI/180.0;
    
    // Weryfikacja danych wejsciowych
    double B=B_stopnie*M_PI/180.0;
    double dL_stopnie=L_stopnie-L0_stopnie;
    double dL=dL_stopnie*M_PI/180.0;
    
    if ((B<Bmin) || (B>Bmax))
          return 1;
          
    if ((dL<dLmin) || (dL>dLmax))
          return 2;
          
    //etap I - elipsoida na kulê
    double U=1.0-e*sin(B);
    double V=1.0+e*sin(B);
    double K=pow((U/V),(e/2.0));
    double C=K*tan(B/2.0+M_PI/4.0);
    double fi=2.0*atan(C)-M_PI/2.0;
    double d_lambda=dL;
    
    // etap II - kula na walec
    double p=sin(fi);
    double q=cos(fi)*cos(d_lambda);
    double r=1.0+cos(fi)*sin(d_lambda);
    double s=1.0-cos(fi)*sin(d_lambda);
    double XMERC=R0*atan(p/q);
    double YMERC=0.5*R0*log(r/s);

    //etap III - walec na p³aszczyznê
    complex<double> Z((XMERC-xo)*Snorm,YMERC*Snorm);
    complex<double> Zgk;
    Zgk=a0+Z*(a1+Z*(a2+Z*(a3+Z*(a4+Z*(a5+Z*a6)))));
    double Xgk=Zgk.real();
    double Ygk=Zgk.imag();
    
    //Przej¶cie do uk³adu aplikacyjnego
    *Xpuwg=m0*Xgk+x0;
    *Ypuwg=m0*Ygk+y0;
    
    return 0;
    }
  
int puwg92_do_wgs84(double Xpuwg, double Ypuwg, double *B_stopnie, double *L_stopnie)
/*
Opis:
    konwersja wspolrzednych z ukladu PUWG 1992 do ukladu WGS 84
Parametry:
    Xpuwg - wskazanie na wspolrzedna X ukladu PUWG 1992 (UWAGA - wspolrzedna pionowa)
    Ypuwg - wskazanie na wspolrzedna Y ukladu PUWG 1992 (UWAGA - wspolrzedna pozioma)
    B_stopnie - szerokosc geograficzna wyrazona w stopniach
    L_stopnie - dlugosc geograficzna wyrazona w stopniach
Zwracana wartosc:
    0 - konwersja powiodla sie
*/  
	{
	double L0_stopnie=19.0; 		//Pocz±tek uk³adu wsp. PUWG92 (d³ugo¶æ)
	double m0=0.9993;
    double x0=-5300000.0;
    double y0= 500000.0;
    
    double R0=6367449.14577; 	//promieñ sfery Lagrange.a
    double Snorm=2.0E-6;   		//parametr normuj±cy
    double xo_prim=5765181.11148097; 		//parametr centruj±cy
    
    // Wspolczynniki wielomianu
    double b0=5760000;
    double b1=500199.26224125;
    double b2=63.88777449;
    double b3=-0.82039170;
    double b4=-0.13125817;
    double b5=0.00101782;
    double b6=0.00010778;
        
    // Wspolczynniki szeregu tryg.
    double c2=0.0033565514856;
    double c4=0.0000065718731;
    double c6=0.0000000176466;
    double c8=0.0000000000540;
    
	//Przejscie z ukladu aplikacyjnego
	double Xgk, Ygk;
	Xgk=(Xpuwg-x0)/m0;
	Ygk=(Ypuwg-y0)/m0;
		
	//etap I - (Xgk, Ygk) -> (Xmerc, Ymerc)
	complex<double> Z((Xgk-xo_prim)*Snorm,Ygk*Snorm);
	complex<double> Zmerc;
	
	Zmerc=b0+Z*(b1+Z*(b2+Z*(b3+Z*(b4+Z*(b5+Z*b6)))));
	
	double Xmerc=Zmerc.real(); 
	double Ymerc=Zmerc.imag();
	
	//etap II - Xmerc,Ymerc -> fi, delta_lambda
	double alfa=Xmerc/R0;
	double beta=Ymerc/R0;
	
	double w=2.0*atan(exp(beta))-M_PI/2.0;
	double fi=asin(cos(w)*sin(alfa));
	double d_lambda=atan(tan(w)/cos(alfa));
	
	//etap III
	double B=fi+c2*sin(2.0*fi)+c4*sin(4.0*fi)+c6*sin(6.0*fi)+c8*sin(8.0*fi);
	double dL=d_lambda;
	
	//Obliczenia koncowe
	*B_stopnie=B/M_PI*180.0;
	double dL_stopnie=dL/M_PI*180.0;
	*L_stopnie=dL_stopnie+L0_stopnie;
    
	return 0;
	}