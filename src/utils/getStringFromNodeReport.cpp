#include "USCutils.h"

//---------------------------------------------------------
// Procedure: getStringFromNodeReport
//            retrieve any string value from node_report by name
//
std::string getStringFromNodeReport(std::string full_string, std::string key)
{
  // example: NAME=anton,X=2676.17,Y=1908.45,SPD=1.48,HDG=316.19,DEP=0,
  //   LAT=34.26380127,LON=-117.17504934,TYPE=SHIP,GROUP=survey,MODE=DRIVE,
  //   ALLSTOP=clear,index=57,YAW=316.19,TIME=1398119728.44,LENGTH=8

  // handle comma-separated string
  std::string output;
  std::vector<std::string> svector = parseString(full_string, ',');
  unsigned int i, vsize = svector.size();
  for(i=0; i<vsize; i++) {
    std::string param = biteStringX(svector[i], '=');
    std::string value = svector[i];
    if(param == key)
      output = value;
  }

  return output;
}
