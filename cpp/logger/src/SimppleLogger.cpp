#include "SimppleLogger.hpp"

namespace pplelog {

typedef std::pair<LevelTracker, SimppleLogger*> loggerElement_t;
typedef std::map<size_t, loggerElement_t> loggerMap_t;

static const std::string STR_TRACE = "TRACE";
static const std::string STR_DEBUG = "DEBUG";
static const std::string STR_INFO  = "INFO";
static const std::string STR_WARN  = "WARN";
static const std::string STR_ERROR = "ERROR";
static const std::string STR_FATAL = "FATAL";
static const std::string STR_DEFAULT = " -- ";

const std::string &getLevelDescription(levels level) {
	switch (level) {
	case(TRACE):
		return STR_TRACE;
	case(DEBUG):
		return STR_DEBUG;
	case(INFO):
		return STR_INFO;
	case(WARN):
		return STR_WARN;
	case(ERROR):
		return STR_ERROR;
	case(FATAL):
		return STR_FATAL;
	default:
		return STR_DEFAULT;
	}
}


class SingleRootLogger {
public:

    static loggerMap_t &getLoggerMap() {
    	checkRootExistence();
        return rootLogger->globalLoggerMap;
    }

    static levels defaultLevel;

    //* Should be assigned at all times
    logCallback_t logCallback;

    static void defaultCoutLog(std::string name, levels level, std::string str) {
    	std::cout << "[" << name << "] |" << getLevelDescription(level) << "|:"
    			  << str << std::endl;
    }

    static SingleRootLogger *rootLogger;

    inline static void checkRootExistence() {
        if (rootLogger == NULL) {
            rootLogger = new SingleRootLogger();
        }
    }

private:
    loggerMap_t globalLoggerMap;
    SingleRootLogger() { logCallback = defaultCoutLog; };
    ~SingleRootLogger() { };
};

SingleRootLogger *SingleRootLogger::rootLogger = NULL;
levels SingleRootLogger::defaultLevel;

logCallback_t getDefaultLogCallback() {
	return SingleRootLogger::defaultCoutLog;
}

logCallback_t getCurrentLogCallback() {
	SingleRootLogger::checkRootExistence();
	return SingleRootLogger::rootLogger->logCallback;
}

void setLogCallback(const logCallback_t newLogCallback) {
	SingleRootLogger::checkRootExistence();
	SingleRootLogger::rootLogger->logCallback = newLogCallback;
}

#if __cplusplus < 201103L
// We define an internal string hash function
static size_t strhash(const std::string &name) {
	int ret = 0;
	char *ptr = reinterpret_cast<char *>(&ret);
	for (unsigned int i=0; i<name.size() ; ++i) {
		ptr[i%sizeof(int)] ^= name[i];
	}
	return ret;
}
#else
// We use the C++11 provided hash function
static std::hash<std::string> strhash;

#endif

LevelTracker::LevelTracker() :
		localLevel(SingleRootLogger::defaultLevel),
		parent(NULL),
		assignedLevel(&(SingleRootLogger::defaultLevel)) {
	// Empty constructor
}

void LevelTracker::setParent(LevelTracker &parent) {
	this->parent = &parent;
	this->assignedLevel = parent.assignedLevel;
	parent.children.push_back(this);
}

LevelTracker::operator levels() {
	return *assignedLevel;
}

void LevelTracker::setLevel(levels &changeLevel) {
	levels *oldLevel = this->assignedLevel;
	localLevel = changeLevel;
	levels *newLevel = &localLevel;
	assignedLevel = newLevel;
	for (std::vector<LevelTracker *>::iterator it = this->children.begin();
			it != this->children.end(); ++it) {
		(*it)->deepLevelChanger(oldLevel, newLevel);
	}
}

void LevelTracker::deepLevelChanger(levels *precursor, levels *future) {
	if (this->assignedLevel == precursor) {
		this->assignedLevel = future;
		for (std::vector<LevelTracker *>::iterator it = this->children.begin();
				it != this->children.end(); ++it) {
			(*it)->deepLevelChanger(precursor, future);
		}
	}
}

levels getDefaultLevel() {
	return SingleRootLogger::defaultLevel;
}

void setDefaultLevel(levels newlevel) {
	SingleRootLogger::defaultLevel = newlevel;
}

levels SimppleLogger::getLevel() const {
	return *(this->partner);
}

static void setLogLevel(size_t hash, levels level) {
    loggerMap_t &globalLoggerMap = SingleRootLogger::getLoggerMap();
	loggerElement_t &elem = globalLoggerMap[hash];
	elem.first.setLevel(level);
}

void setLogLevel(const std::string &name, levels level) {
	int hash = strhash(name);
	setLogLevel(hash, level);
}

void SimppleLogger::setLogLevel(levels level) {
	pplelog::setLogLevel(this->hash, level);
}

SimppleLogger *getLogger(const std::string &s) {
    loggerMap_t &globalLoggerMap = SingleRootLogger::getLoggerMap();
	int hash = strhash(s);
	loggerElement_t &elem = globalLoggerMap[hash];
	SimppleLogger *retlog;
	if (elem.second == 0) {
	    // Actually create it
		retlog = new SimppleLogger(s);
		globalLoggerMap[hash] = loggerElement_t(elem.first, retlog);
		retlog->partner = &(elem.first);
		return retlog;
	} else {
		return (elem.second);
	}
}

SimppleLogger *getLogger(const std::string &s, const std::string &parent) {
	SimppleLogger *retLogger = getLogger(s);
	retLogger->setParent(parent);
	return retLogger;
}

void SimppleLogger::setParent(size_t hash_base, size_t hash_parent) {
    loggerMap_t &globalLoggerMap = SingleRootLogger::getLoggerMap();
	loggerElement_t &elem_base = globalLoggerMap[hash_base];
	loggerElement_t &elem_parent = globalLoggerMap[hash_parent];

	elem_base.first.setParent(elem_parent.first);
}

void setParent(const std::string &base, const std::string &parent) {
	SimppleLogger::setParent(strhash(base), strhash(parent));
}

void SimppleLogger::setParent(const std::string &parent) {
	setParent(this->hash, strhash(parent));
}

void SimppleLogger::setParent(const SimppleLogger *parent) {
	setParent(this->hash, parent->hash);
}

void SimppleLogger::log(levels level, std::string s_) {
	SingleRootLogger::rootLogger->logCallback(this->name, level, s_);
}

const std::string &SimppleLogger::getName() const {
	return name;
}

SimppleLogger::SimppleLogger(const std::string &s):
        name(s), hash(strhash(s)), partner(NULL) {
	// Unique private constructor, initializes internal values

	// Factory method should already have updated the static map
}

SimppleLogger::~SimppleLogger() {
    // Destructor does nothing

    // Prevents the compiler creating other implicit constructors
}

}; // namespace pplelog
