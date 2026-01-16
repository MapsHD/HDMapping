#ifndef WGS84_DO_PUWG92_H
#define WGS84_DO_PUWG92_H
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

int wgs84_do_puwg92(double B_stopnie, double L_stopnie, double *Xpuwg, double *Ypuwg);
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

int puwg92_do_wgs84(double Xpuwg, double Ypuwg, double *B_stopnie, double *L_stopnie);
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

#endif