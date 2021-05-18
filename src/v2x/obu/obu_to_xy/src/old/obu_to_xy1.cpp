
#include <iostream>
#include "noval_convert_xy.hpp"
using namespace std;



int main()
{
  
  double jd=117.4375922;
  double wd=39.1921173;
  double x;
  double y;
  noval_convert_xy a;
  a.jwd_to_xy(jd,wd,x,y);
}