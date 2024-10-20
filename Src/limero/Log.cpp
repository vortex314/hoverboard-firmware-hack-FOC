/*
 * LOg.cpp
 *
 *  Created on: Jul 3, 2016
 *      Author: lieven
 */
#include <limero/Log.h>
extern "C" {
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <sys/time.h>
#include <time.h>
#include <unistd.h>
}

const char* Log::time() {
    return "";
    /*static char szTime[20];
    snprintf(szTime, sizeof(szTime), "%llu", log_millis());
    return szTime;*/
}

char Log::_logLevel[7] = { 'T', 'D', 'I', 'W', 'E', 'F', 'N' };

std::string& string_format(std::string& str, const char* fmt, ...) {
    int size = strlen(fmt) * 2 + 50; // Use a rubric appropriate for your code
    va_list ap;
    while (1) { // Maximum two passes on a POSIX system...
        str.resize(size);
        va_start(ap, fmt);
        int n = vsprintf((char*)str.data(), fmt, ap);
        va_end(ap);
        if (n > -1 && n < size) { // Everything worked
            str.resize(n);
            return str;
        }
        if (n > -1)       // Needed size returned
            size = n + 1; // For null char
        else
            size *= 2; // Guess at a larger size (OS specific)
    }
    return str;
}

void bytesToHex(std::string& ret, uint8_t* input, uint32_t length, char sep) {
    static const char characters[] = "0123456789ABCDEF";
    for (uint32_t i = 0; i < length; i++) {
        ret += (characters[input[i] >> 4]);
        ret += characters[input[i] & 0x0F];
        if (sep != 0)
            ret += sep;
    }
}

void Log::serialLog(char* start, uint32_t length) {
    *(start + length) = '\0';
    ::printf("%s\n", start);
}

Log::Log(uint32_t size)
    : _enabled(true), _logFunction(serialLog), _hostname("stm32"), _application("hoverboard"), _level(LOG_INFO) {
    if (_line == 0) {
        _line = new std::string;
        _line->reserve(size);
    }
}

Log::~Log() {}

void Log::setLogLevel(char c) {
    for (uint32_t i = 0; i < sizeof(_logLevel); i++)
        if (_logLevel[i] == c) {
            _level = (Log::LogLevel)i;
            break;
        }
}

bool Log::enabled(LogLevel level) {
    if (level >= _level) {
        return true;
    }
    return false;
}

void Log::disable() { _enabled = false; }

void Log::enable() { _enabled = true; }

void Log::defaultOutput() { _logFunction = serialLog; }

void Log::writer(LogFunction function) { _logFunction = function; }

LogFunction Log::writer() { return _logFunction; }

void Log::log(char level, const char* file, uint32_t lineNbr,
    const char* function, const char* fmt, ...) {
    if (_line == 0) {
        ::printf("%s:%d %s:%u\n", __FILE__, __LINE__, file,
            (unsigned int)lineNbr);
        //		_sema.release();
        return;
    }

    va_list args;
    va_start(args, fmt);
    static char logLine[256];
    vsnprintf(logLine, sizeof(logLine) - 1, fmt, args);
    va_end(args);

    string_format(*_line, "%+10.10s %c | %8s | %s | %15s:%4d | %s",
        _application, level, time(), "stm32", file, lineNbr,
        logLine);
    logger.flush();
}

void Log::flush() {
    if (_logFunction)
        _logFunction((char*)_line->c_str(), _line->size());
    *_line = "";
}

void Log::level(LogLevel l) { _level = l; }

Log::LogLevel Log::level() { return _level; }

//_________________________________________ EMBEDDED





