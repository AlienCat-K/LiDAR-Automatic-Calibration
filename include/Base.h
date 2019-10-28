#ifndef BASE_H
#define BASE_H

#include <fstream>
#include <vector>
#include<string>
#include <map>
using namespace std;


class ParameterReader
{
public:
	ParameterReader(string filename = "../data/parameters.txt");
	string getData(string key);

public:
    map<string, string> data;
};
#endif
