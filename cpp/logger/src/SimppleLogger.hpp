#ifndef SIMPPLELOGGER_HPP_
#define SIMPPLELOGGER_HPP_

// Required by the macro, forcing it in the include header
#include <sstream>
#include <map>
#include <vector>

#if __cplusplus >= 201103L
// Used only in C++11 mode for std::hash and std::function
#include <functional>
#endif

#include <iostream>

/**
 * Logs a message to a specified logger with the TRACE level.
 *
 * @param logger the logger to be used.
 * @param message the message string to log.
 */
#define SPPLELOG_TRACE(logger, message) { \
        if (logger->getLevel() <= pplelog::TRACE) {\
           std::ostringstream oss_; \
           oss_ << message; \
           logger->log(pplelog::TRACE, oss_.str()); }}

/**
 * Logs a message to a specified logger with the DEBUG level.
 *
 * @param logger the logger to be used.
 * @param message the message string to log.
 */
#define SPPLELOG_DEBUG(logger, message) { \
        if (logger->getLevel() <= pplelog::DEBUG) {\
           std::ostringstream oss_; \
           oss_ << message; \
           logger->log(pplelog::DEBUG, oss_.str()); }}

/**
 * Logs a message to a specified logger with the INFO level.
 *
 * @param logger the logger to be used.
 * @param message the message string to log.
 */
#define SPPLELOG_INFO(logger, message) { \
        if (logger->getLevel() <= pplelog::INFO) {\
           std::ostringstream oss_; \
           oss_ << message; \
           logger->log(pplelog::INFO, oss_.str()); }}

/**
 * Logs a message to a specified logger with the WARN level.
 *
 * @param logger the logger to be used.
 * @param message the message string to log.
 */
#define SPPLELOG_WARN(logger, message) { \
        if (logger->getLevel() <= pplelog::WARN) {\
           std::ostringstream oss_; \
           oss_ << message; \
           logger->log(pplelog::WARN, oss_.str()); }}

/**
 * Logs a message to a specified logger with the ERROR level.
 *
 * @param logger the logger to be used.
 * @param message the message string to log.
 */
#define SPPLELOG_ERROR(logger, message) { \
        if (logger->getLevel() <= pplelog::ERROR) {\
           std::ostringstream oss_; \
           oss_ << message; \
           logger->log(pplelog::ERROR, oss_.str()); }}

/**
 * Logs a message to a specified logger with the FATAL level.
 *
 * @param logger the logger to be used.
 * @param message the message string to log.
 */
#define SPPLELOG_FATAL(logger, message) { \
        if (logger->getLevel() <= pplelog::FATAL) {\
           std::ostringstream oss_; \
           oss_ <<  message; \
           logger->log(pplelog::FATAL, oss_.str()); }}


namespace pplelog {

/**
 * List of possible levels associated to a logger
 */
enum levels{
    ALWAYS_LOG = -5,
    TRACE,
    DEBUG = 0,
    INFO,
    WARN,
    ERROR,
    FATAL,
    NEVER_LOG = 1000
};

/**
 * Return the friendly description of the logging level.
 * @param level Level to be "decoded".
 * @return A const string containing the level readable code.
 */
const std::string &getLevelDescription(levels level);

#if __cplusplus < 201103L
    typedef void (*logCallback_t)(std::string, pplelog::levels, std::string);
#else
    typedef std::function<void(std::string, pplelog::levels, std::string)> logCallback_t;
#endif


class LevelTracker;
class SimppleLogger;

levels getDefaultLevel();
void setDefaultLevel(levels newlevel);
void setLogLevel(const std::string &name, levels level);
void setParent(const std::string &base, const std::string &parent);
SimppleLogger *getLogger(const std::string &s);
SimppleLogger *getLogger(const std::string &s, const std::string &parent);

logCallback_t getDefaultLogCallback();
logCallback_t getCurrentLogCallback();
void setLogCallback(logCallback_t newLogCallback);

}; // namespace pplelog

class pplelog::LevelTracker {
public:
    LevelTracker();
    void setParent(LevelTracker &parent);
    operator levels();
    void setLevel(levels &changeLevel);
private:
    void deepLevelChanger(levels *precursor, levels *future);
    levels localLevel;
    LevelTracker *parent;
    levels *assignedLevel;
    std::vector<LevelTracker *> children;
};

class pplelog::SimppleLogger {
public:
    levels getLevel() const;
    void setLogLevel(levels level);
    void log(levels level, std::string s_);
    const std::string &getName() const;

    void setParent(const std::string &parent);
    void setParent(const SimppleLogger *parent);

private:
    std::string name;
    size_t hash;
    SimppleLogger(const std::string &s);
    ~SimppleLogger();

    static void setParent(size_t hash_base, size_t hash_parent);
    LevelTracker *partner;

// Friends
    friend levels getDefaultLevel();
    friend void setDefaultLevel(levels newlevel);
    friend void setLogLevel(const std::string &name, levels level);
    friend SimppleLogger *getLogger(const std::string &s);
    friend SimppleLogger *getLogger(const std::string &s, const std::string &parent);
    friend void setParent(const std::string &base, const std::string &parent);
};

#endif /* SIMPPLELOGGER_HPP_ */
