/*
 Name:		FormatTime.cpp
 Created:	28 June 2020
 Author:	Didier Coyman
 MIT License
 Copyright (c) 2020 Didier  Coyman
 Permission is hereby granted, free of charge, to any person obtaining a copy
 of this software and associated documentation files (the "Software"), to deal
 in the Software without restriction, including without limitation the rights
 to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 copies of the Software, and to permit persons to whom the Software is
 furnished to do so, subject to the following conditions:
 The above copyright notice and this permission notice shall be included in all
 copies or substantial portions of the Software.
 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 SOFTWARE.
*/
#include "FormatTime.h"

// return time formatted like `hh:mm:ss`, requires 9 bytes
void FormatTime::getFormattedTime(char* buffer, uint8_t buffsize)
{
	if (buffsize >= 9) {
		time_t rawtime;
		struct tm* timeinfo;

		time(&rawtime);
		timeinfo = localtime(&rawtime);

		strftime(buffer, buffsize, "%T", timeinfo);
	}
}

// return time formatted like `dd-mm-yyyy hh:mm:ss`, requires 20 bytes
void FormatTime::getFullFormattedTime(char* buffer, uint8_t buffsize)
{
	if (buffsize >= 20) {
		time_t rawtime;
		struct tm* timeinfo;

		time(&rawtime);
		timeinfo = localtime(&rawtime);

		strftime(buffer, buffsize, "%d-%m-%Y %T", timeinfo);
	}
}

// return time formatted like `yyyy-mm-ddThh:mm:ssZ`, requires 25 bytes
void FormatTime::getFormattedTimeISO8601(char* buffer, uint8_t buffsize)
{
	if (buffsize >= 25) {
		time_t rawtime;
		struct tm* timeinfo;

		time(&rawtime);
		timeinfo = gmtime(&rawtime);

		strftime(buffer, 25, "%FT%TZ", timeinfo);
	}
}

