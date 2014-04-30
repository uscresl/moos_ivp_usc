#include "USCutils.h"

//---------------------------------------------------------
// Procedure: getDoubleFromNodeReport
//            retrieve any double value from node_report by name
//
double getDoubleFromNodeReport(std::string full_string, std::string key)
{
  std::string output = getStringFromNodeReport(full_string, key);
  return atof(output.c_str());
}
