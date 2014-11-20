#include "USCutils.h"

//---------------------------------------------------------
// Procedure: getDoubleFromNodeReport
//            retrieve any double value from node_report by name
//
double getDoubleFromNodeReport(std::string full_string, std::string key)
{
  std::string output = getStringFromNodeReport(full_string, key);
  if ( !output.empty() )
  {
    return atof(output.c_str());
  }
  else
    return std::numeric_limits<double>::max();
}
