#ifndef TPHandler_H
#define TPHandler_H

#include <CTiePt.h>

#include <assert.h>
#include <stdio.h>
#include <string>
#include <sstream>
#include <fstream>
#include <iostream>
#include <vector>
#include <cmath>

class TPHandler
{
public:
    int processTiepointFile(std::string fname = "");
    int testParseTiepoint();
    std::vector<CTiePt> getTiepoints();

    std::vector<std::vector<double> > tiepoints;

private:
    int parseTiepoint(std::string line, std::vector<double>& tiepoint);
    std::vector<std::string> &split(const std::string &s, char delim, std::vector<std::string> &elems);
    std::vector<std::string> split(const std::string &s, char delim);
};

#endif // TPHandler_H
