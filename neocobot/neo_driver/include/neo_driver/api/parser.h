#ifndef PARSER_H
#define PARSER_H

#include "tinyxml.h"
#include <string>
#include <map>
#include <vector>

using namespace std;

class CheckSum
{
public:
	CheckSum();
	~CheckSum();
	void AddElement(string element);
	int CalcSum();

private:
	int sum;
	vector<string> _element;

};

class Parser
{
public:
	bool wrap_license(string &msg, map<string, string> &data);
	bool wrap_clientinfo(string &msg, map<string, string> &data);
	bool wrap_heart(string &msg, map<string, string> &data);
	bool wrap_msg(string &msg, map<string, string> &data);
	bool unwrap(map<string, string> &data, string &msg);

private:

};


#endif
