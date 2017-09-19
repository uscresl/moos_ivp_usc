/*****************************************************************/
/*    NAME: Stephanie Kemna                                      */
/*    ORGN: Robotic Embedded Systems Lab, CS, USC, CA, USA       */
/*    FILE: main.cpp                                             */
/*    DATE: Mar 29, 2014                                         */
/*                                                               */
/*****************************************************************/

#include <string>
#include "MBUtils.h"
#include "ColorParse.h"

#include "PGP.h"

using namespace std;

int main(int argc, char *argv[])
{
  string mission_file;
  string run_command = argv[0];

  for(int i=1; i<argc; i++) 
  {
    string argi = argv[i];
    
    if(strEnds(argi, ".moos") || strEnds(argi, ".moos++"))
      mission_file = argv[i];
    else if(strBegins(argi, "--alias="))
      run_command = argi.substr(8);
    else if(i==2)
      run_command = argi;
  }
  cout << termColor("green");
  cout << "pGP launching as " << run_command << endl;
  cout << termColor() << endl;

  GP temp1;
  temp1.Run(run_command.c_str(), mission_file.c_str());
  
  return(0);
}
