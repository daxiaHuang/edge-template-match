//***********************************************************************
// Project		    : GeoMatch
// Author           : Shiju P K
// Email			: shijupk@gmail.com
// Created          : 03-07-2010
//
// File Name		: CommandParser.h
// Last Modified By : Shiju P K
// Last Modified On : 03-07-2010
// Description      : class to implement comand line parsing
//
// Copyright        : (c) . All rights reserved.
//***********************************************************************
#pragma once

class CommandParser
{
	char		**argList;		//point to hold argument list
	int			numArgs;		// no of arguments in the list
public:
	CommandParser(void);
	CommandParser(int,char**);
	~CommandParser(void);
	char* GetParameter(const char *key);
};
