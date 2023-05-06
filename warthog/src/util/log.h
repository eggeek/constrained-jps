/**
 * @file
 *
 * Simple log utility. Can work both on preprocessor time or at compile time
 *
 * All this function works only if:
 * @li the macro @c DEBUG is active OR;
 * @li the macro @c NDEBUG is not active;
 *
 * Otherwise, they will become just empty (or can be easily optimized away)
 *
 * @date Oct 1, 2018
 * @author koldar
 */

#ifndef LOG_H_
#define LOG_H_

#include <iostream>
#include "macros.h"
#include "file_utils.h"

#ifndef QUICK_LOG
#define QUICK_LOG 0
#endif

#if QUICK_LOG <= 0
/**
 * always log an entry via std cio
 *
 * Does no impact performances whatsoever
 *
 * @code
 * 	debug("hello", person->name, "!");
 * @endcode
 *
 * @param[in] ... the entries to put on stream
 */
#define debug(p, ...) _abstractLog("DEBUG", p, __VA_ARGS__)
#else
#define debug(...) ;
#endif

#if QUICK_LOG <= 1
/**
 * always log an entry via std cio
 *
 * Does no impact performances whatsoever
 *
 * @code
 * 	debug("hello", person->name, "!");
 * @endcode
 *
 * @param[in] ... the entries to put on stream
 */
#define trace(p, ...) _abstractLog("TRACE", p, __VA_ARGS__)
#else
#define trace(...) ;
#endif

#if QUICK_LOG <= 4
/**
 * always log an entry via std cio
 *
 * Does no impact performances whatsoever
 *
 * @code
 * 	debug("hello", person->name, "!");
 * @endcode
 *
 * @param[in] ... the entries to put on stream
 */
#define info(p, ...) _abstractLog("INFO ", p, __VA_ARGS__)
#else
#define info(...) ;
#endif

#if QUICK_LOG <= 5
/**
 * always log an entry via std cio
 *
 * Does no impact performances whatsoever
 *
 * @code
 * 	debug("hello", person->name, "!");
 * @endcode
 *
 * @param[in] ... the entries to put on stream
 */
#define user(p, ...) _abstractLog("USER ", p, __VA_ARGS__)
#else
#define user(...) ;
#endif

#if QUICK_LOG <= 6
/**
 * always log an entry via std cio
 *
 * Does no impact performances whatsoever
 *
 * @code
 * 	debug("hello", person->name, "!");
 * @endcode
 *
 * @param[in] ... the entries to put on stream
 */
#define warning(p, ...) _abstractLog("WARN ", p, __VA_ARGS__)
#else
#define warning(...) ;
#endif

#if QUICK_LOG <= 7
/**
 * always log an entry via std cio
 *
 * Does no impact performances whatsoever
 *
 * @code
 * 	debug("hello", person->name, "!");
 * @endcode
 *
 * @param[in] ... the entries to put on stream
 */
#define error(...) _abstractLog("ERROR", true, __VA_ARGS__)
#else
#define error(...) ;
#endif

#if QUICK_LOG <= 8
/**
 * always log an entry via std cio
 *
 * Does no impact performances whatsoever
 *
 * @code
 * 	debug("hello", person->name, "!");
 * @endcode
 *
 * @param[in] ... the entries to put on stream
 */
#define critical(...) _abstractLog("CRTCL", true, __VA_ARGS__)
#else
#define critical(...) ;
#endif

/**
 * like ::log but will log only if at runtime the @c expr will evaluate to true
 *
 * Will impact performances (even if by little) even if the log is turned off
 *
 * @code
 * clog(true)("hello ", this->name);
 * @endcode
 *
 * @param[in] expr the expression to be evaluated
 * @param[in] ... input for ::log
 */
#define clog(expr) _clog1(expr)

#define _clog1(expr) if (expr) { _clog2
#define _clog2(...)  debug(__VA_ARGS__); }

/**
 * Condition Statement Log utility allows to perform some previous statement before logging
 *
 * This logging macro is useful when your logging needs some local variable
 *
 * @code
 * cslog(true)(int a = 5)("a is %d", a);
 * @endcode
 *
 * @param[in] expr the condition to check if the log is enabled
 */
#define cslog(expr) _cslog1(expr)
#define _cslog1(expr) if (expr) { _cslog2
#define _cslog2(...) __VA_ARGS__; _cslog3
#define _cslog3(...) debug(__VA_ARGS__); }

#define _LOG_OP(context, index, x) x
#define _LOG_COMB(context, x, y) x << " " << y

#define _debug(...) FOR_EACH(, _LOG_OP, _LOG_COMB, __VA_ARGS__)

#if defined(DEBUG) || !defined(NDEBUG)
/**
 * Perform some work if debug has been enabled
 *
 *
 */
#define DO_ON_DEBUG_IF(expr) if (expr)
#define DO_ON_DEBUG if (true)

/**
 * always log an entry via std cio
 *
 * Does no impact performances whatsoever
 *
 * @code
 * 	debug("hello", person->name, "!");
 * @endcode
 *
 * @param[in] level the name of the log level
 * @param[in] ... the entries to put on stream
 */
#define _abstractLog(level, p, ...) if (p) std::cerr <<  "[" << level << "] " << getBaseName(__FILE__) << "@" << __func__ << ":" << __LINE__ << " " << _debug(__VA_ARGS__) << std::endl

#else

#define DO_ON_DEBUG_IF(expr) if(false)
#define DO_ON_DEBUG if (false)
#define _abstractLog(level, ...) ;

#endif


#endif /* LOG_H_ */
