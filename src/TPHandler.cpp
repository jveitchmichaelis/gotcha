#include "TPHandler.h"

using namespace std;

std::vector<std::string> &TPHandler::split(const std::string &s, char delim, std::vector<std::string> &elems) {
    std::stringstream ss(s);
    std::string item;
    while (std::getline(ss, item, delim)) {
        elems.push_back(item);
    }
    return elems;
}

std::vector<std::string> TPHandler::split(const std::string &s, char delim) {
    std::vector<std::string> elems;
    split(s, delim, elems);
    return elems;
}

int TPHandler::parseTiepoint(string line, std::vector<double>& tiepoint){
    double lx,ly,rx,ry;

    std::vector<std::string> coords;
    coords = split(line, ' ');

    /* Incorrect number of coordinates*/
    if(coords.size() < 4){
        cout << "Loading tiepoint failed, too few coordinates \n";
        return 0;
    }

    try{
        lx = std::stod(coords[0]);
        ly = std::stod(coords[1]);
        rx = std::stod(coords[2]);
        ry = std::stod(coords[3]);
    }
    catch (exception& e) {
        cout << e.what() << '\n';
        return 1;
    }

    /* No negative coordinates, no infs and no nans */
    if(lx < 0 || ly < 0 || rx < 0 || ry < 0)
        return 0;

    if(std::isinf(lx) || std::isinf(ly) || std::isinf(rx) || std::isinf(ry))
        return 0;

    if(lx != lx || ly != ly || rx != rx || ry != ry)
        return 0;

    tiepoint.push_back(lx);
    tiepoint.push_back(ly);
    tiepoint.push_back(rx);
    tiepoint.push_back(ry);

    return 1;
}

int TPHandler::testParseTiepoint(){
    std::vector<double> tiepoint;

    assert(parseTiepoint("1 2 3 4", tiepoint) == 1);
    assert(parseTiepoint("1     2 3 4", tiepoint) == 1);
    assert(parseTiepoint("0.0 2.6 3.0 4.0", tiepoint) == 1);

    assert(parseTiepoint("", tiepoint) == 0);
    assert(parseTiepoint("1 2 3 4 5", tiepoint) == 0);
    assert(parseTiepoint("-1 2 3 4", tiepoint) == 0);
    assert(parseTiepoint("1 a 3 4", tiepoint) == 0);
    assert(parseTiepoint("inf 2 3 4", tiepoint) == 0);
    assert(parseTiepoint("-inf 2 3 4", tiepoint) == 0);
    assert(parseTiepoint("nan 2 3 4", tiepoint) == 0);
    assert(parseTiepoint("-nan 2 3 4", tiepoint) == 0);

    return 0;
}

std::vector<CTiePt> TPHandler::getTiepoints(){
    std::vector<CTiePt> outputTiepoints;

    for(size_t i=0; i < tiepoints.size(); i++){
        CTiePt temp;

        temp.m_ptL.x = tiepoints.at(i)[0];
        temp.m_ptL.y = tiepoints.at(i)[1];
        temp.m_ptR.x = tiepoints.at(i)[2];
        temp.m_ptR.y = tiepoints.at(i)[3];

        outputTiepoints.push_back(temp);
    }

    return outputTiepoints;
}

int TPHandler::processTiepointFile(string fname){
    string line;
    std::vector<string> results;
    std::ifstream ifs;

    ifs.open(fname, std::ifstream::in);

    std::vector<double> tiepoint;

    int ntiepoints;

    if(ifs.is_open()){
        cout << "Reading tiepoint file... \n";
        std::getline(ifs, line);
        results = split(line, ' ');

        try{
            ntiepoints = std::stoi(results[0]);

            if(ntiepoints <= 0){
                cout << "The number of tiepoints must be at least one.\nCheck that the first line of the tiepoint file is a single, positive integer.\n";
                return 1;
            }
        }
        catch (exception& e) {
            cout << e.what() << '\n';
            return 1;
        }

        for(int i=0; i < ntiepoints; i++){
            try{
                std::getline(ifs, line);

                if(parseTiepoint(line, tiepoint))
                    tiepoints.push_back(tiepoint);

                tiepoint.clear();

            }catch(std::exception& e){
                cout << "Error: " << e.what();
            }
        }

        cout << "Loaded " << tiepoints.size() << " tiepoints\n";

        ifs.close();
    }else{
        cout << "File doesn't exist!";
        return 1;
    }

    return 0;
}
