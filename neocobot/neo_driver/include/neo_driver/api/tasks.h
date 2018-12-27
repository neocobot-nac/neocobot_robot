#ifndef TASKS_H
#define TASKS_H

#include <map>
using namespace std;

class Tasks
{
public:
	string type;
	map<string, string> data;

	void clear()
	{
		type.clear();
		data.clear();
	}

	Tasks() 
	{
		type.clear();
		data.clear();
	}

	~Tasks() 
	{
		type.clear();
		data.clear();
	}
};

#endif