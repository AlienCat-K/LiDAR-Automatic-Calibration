#include <Base.h>

ParameterReader::ParameterReader(string filename)
{
	std::ifstream fin(filename.c_str());
	if (!fin)
	{
		return;
	}
	while (!fin.eof())
	{
		string str;
		getline(fin, str);
		if (str[0] == '#')
		{
			
			continue;
		}

		int pos = str.find("=");
		if (pos == -1)
			continue;
		string key = str.substr(0, pos);
		string value = str.substr(pos + 1, str.length());
		data[key] = value;

		if (!fin.good())
			break;
	}
}
string ParameterReader::getData(string key)
{
	map<string, string>::iterator iter = data.find(key);
	if (iter == data.end())
	{
		return string("NOT_FOUND");
	}
	return iter->second;
}