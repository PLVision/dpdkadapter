/*-
 * @file dpdk_log.cpp
 *
 * The file contains definition of logging utilities.
 *
 * The file is part of DPDKAdapter project which release under proprietary license.
 * See the License file in the root of project.
 *
 * PLVISION CONFIDENTIAL
 * __________________
 *
 * Copyright (c) [2015] PLVISION sp. z o.o.
 * <developers@plvision.eu>
 * All Rights Reserved.
 */

#include "dpdk_log.h"

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <time.h>
#include <sys/time.h>
#include <stdarg.h>
#include <libgen.h>
#include <stdint.h>
#include <errno.h>
#include <sys/queue.h>

#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif //_GNU_SOURCE

#include <errno.h>

extern char *program_invocation_name;
extern char *program_invocation_short_name;

#define LOG_MAX_LINE	1024

void _Log(int level, const char *file, long line, const char *func, const char *fmt, ...)
{
	char msg[LOG_MAX_LINE + 1] = {0};
	char msgLine[LOG_MAX_LINE + 1] = {0};
	char timestamp[32] = {0};
	struct timeval currTime;
	va_list args;
	
	gettimeofday(&currTime, NULL);

	/* actual log message */
	va_start(args, fmt);
	vsnprintf(msg, LOG_MAX_LINE, fmt, args);
	va_end(args);

	strftime(timestamp, sizeof(timestamp), "%Y-%m-%d %H:%M:%S",
			localtime(&currTime.tv_sec));

	snprintf(msgLine, sizeof(msgLine), "%s %s.%03ld [%s:%ld(%s)] %s",
			  (level == LOG_LEVEL_TRACE)   ? COLOR_TRACE   "TRACE"
			: (level == LOG_LEVEL_DEBUG)   ? COLOR_DEBUG   "DEBUG"
			: (level == LOG_LEVEL_INFO)    ? COLOR_INFO    "INFO"
			: (level == LOG_LEVEL_WARNING) ? COLOR_WARNING "WARNING"
			: (level == LOG_LEVEL_ERROR)   ? COLOR_ERROR   "ERROR"
			: (level == LOG_LEVEL_PANIC)   ? COLOR_PANIC   "PANIC"
			: COLOR_RESET "UNKNOWN",
			timestamp, currTime.tv_usec / 1000,
			basename(strdup(file)), line, func, msg);
			
	printf("%s%s\n", msgLine, COLOR_RESET);
}

void _Msg(int extended, const char *file, long line, const char *func, const char* endOfLine, const char *fmt, ...)
{
	char msg[LOG_MAX_LINE + 1] = {0};
	char msgLine[LOG_MAX_LINE + 1] = {0};
	char timestamp[32] = {0};

	struct timeval currTime;
	va_list args;
	
	gettimeofday(&currTime, NULL);
	
	/* actual log message */
	va_start(args, fmt);
	vsnprintf(msg, LOG_MAX_LINE, fmt, args);
	va_end(args);
	
	strftime(timestamp, sizeof(timestamp), "%Y-%m-%d %H:%M:%S",
			localtime(&currTime.tv_sec));

	if(2 == extended)
	{
		snprintf(msgLine, sizeof(msgLine), "%s %s.%03ld [%s:%ld(%s)] %s",
			program_invocation_short_name, timestamp, currTime.tv_usec / 1000,
			basename(strdup(file)), line, func, msg);
	}
	else if(1 == extended)
	{
		snprintf(msgLine, sizeof(msgLine), "%s", msg);
	}
	else
	{
		snprintf(msgLine, sizeof(msgLine), "%s: %s", program_invocation_short_name, msg);
	}
	printf("%s%s%s%s", COLOR_INFO, msgLine, COLOR_RESET, endOfLine);
}
