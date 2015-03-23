/*-
 * @file dpdk_log.h
 *
 * The file contains declaration of logging utilities.
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

#ifndef __DPDK_LOG_H__
#define __DPDK_LOG_H__

#include <rte_debug.h>

#define LOG_LEVEL_ALL		0
#define LOG_LEVEL_TRACE		1
#define LOG_LEVEL_DEBUG		2
#define LOG_LEVEL_INFO		3
#define LOG_LEVEL_WARNING	4
#define LOG_LEVEL_ERROR		5
#define LOG_LEVEL_PANIC		6
#define LOG_LEVEL_NONE		7

#ifndef LOG_LEVEL
#define LOG_LEVEL	LOG_LEVEL_ALL
#endif

#if LOG_LEVEL <= LOG_LEVEL_TRACE
#define LogTrace(fmt, ...) _Log(LOG_LEVEL_TRACE, __FILE__, __LINE__, __FUNCTION__, fmt, ##__VA_ARGS__)
#else
#define LogTrace(fmt, ...) /* no-op */
#endif

#if LOG_LEVEL <= LOG_LEVEL_DEBUG
#define LogDebug(fmt, ...) _Log(LOG_LEVEL_DEBUG, __FILE__, __LINE__, __FUNCTION__, fmt, ##__VA_ARGS__)
#else
#define LogDebug(fmt, ...) /* no-op */
#endif

#if LOG_LEVEL <= LOG_LEVEL_INFO
#define LogInfo(fmt, ...) _Log(LOG_LEVEL_INFO, __FILE__, __LINE__, __FUNCTION__, fmt, ##__VA_ARGS__)
#else
#define LogInfo(fmt, ...) /* no-op */
#endif

#if LOG_LEVEL <= LOG_LEVEL_WARNING
#define LogWarning(fmt, ...) _Log(LOG_LEVEL_WARNING, __FILE__, __LINE__, __FUNCTION__, fmt, ##__VA_ARGS__)
#else
#define LogWarning(fmt, ...) /* no-op */
#endif

#if LOG_LEVEL <= LOG_LEVEL_ERROR
#define LogError(fmt, ...) _Log(LOG_LEVEL_ERROR, __FILE__, __LINE__, __FUNCTION__, fmt, ##__VA_ARGS__)
#else
#define LogError(fmt, ...) /* no-op */
#endif

#if LOG_LEVEL <= LOG_LEVEL_PANIC
#define LogPanic(fmt, ...) do {                                                \
		_Log(LOG_LEVEL_PANIC, __FILE__, __LINE__, __FUNCTION__,                \
				fmt, ##__VA_ARGS__);                                           \
	} while (0)
#else
#define LogPanic(fmt, ...) rte_panic(fmt, ##__VA_ARGS__)
#endif

// Define log colors
#define COLOR_RESET 		"\e[m"					// Default system color
#define COLOR_PANIC			"\e[01;31m"				// Red
#define COLOR_ERROR			"\e[01;33m"				// Yellow
#define COLOR_WARNING		"\e[01;34m"				// White
#define COLOR_INFO			"\e[m"					// Default system color
#define COLOR_DEBUG			"\e[01;35m"				// Purple
#define COLOR_TRACE			"\e[36m"				// Cyan

void _Log(int level, const char *file, long line, const char *func, const char *fmt, ...);

#define ShowMsg(fmt, ...) _Msg(0, __FILE__, __LINE__, __FUNCTION__, "\n", fmt, ##__VA_ARGS__)
#define ShowMsgEx(lineend, fmt, ...) _Msg(1, __FILE__, __LINE__, __FUNCTION__, lineend, fmt, ##__VA_ARGS__)
#define ShowMsgLog(lineend, fmt, ...) _Msg(2, __FILE__, __LINE__, __FUNCTION__, lineend, fmt, ##__VA_ARGS__)

void _Msg(int extended, const char *file, long line, const char *func, const char * enfOfLine, const char *fmt, ...);

#define qFatal		LogPanic
#define qCritical	LogError
#define qWarning    LogWarning
#define qDebug		LogTrace

#endif //__DPDK_LOG_H__
