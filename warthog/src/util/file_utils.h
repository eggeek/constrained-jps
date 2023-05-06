/*
 * file_utils.h
 *
 *  Created on: Oct 1, 2018
 *      Author: koldar
 */

#ifndef DYNAMIC_PATH_FINDING_FILE_UTILS_H_
#define DYNAMIC_PATH_FINDING_FILE_UTILS_H_

#include <string>

/**
 * @brief get the number of bytes the file occipies on the filesyste,
 *
 * @param name the name fo the file to aqnalyze
 * @return size_t number of bytes @c name occupies
 */
size_t getBytesOfFile(const std::string& name);

/**
 * Check if a file exists
 *
 * @param[in] name the filename to check
 * @return
 *  @li true fi the file exists;
 *  @li false otherwise;
 */
bool isFileExists(const std::string& name);

/**
 * Get the basename of a file given its absolute path
 *
 * @pre
 *  @li filepath is an absolute path (like the one returneed by __FILE__);
 *
 * @param[in] filepath the absolute path to handle
 * @return a pointer in @c filepath where the basename of the file starts
 */
const char* getBaseName(const char* filepath);

/**
 * @brief Get the basename of a file given its path
 *
 * @pre
 *  @li filepath does not end with "/"
 *
 * @param filepath  the path to handle
 * @return const char* a pointer of the given path
 */
const char* getBaseName(const std::string& filepath);

/**
 * @brief Get the basename of a file given its path
 *
 * @pre
 *  @li filepath does not end with "/"
 *
 * @param filepath  the path to handle
 * @return a copy of the basename path
 */
std::string getBaseNameAsString(const std::string& filepath);

/**
 * @brief Get the basename of a file given its path
 *
 * @pre
 *  @li filepath does not end with "/"
 *
 * @param filepath  the path to handle
 * @return a copy of the basename path
 */
std::string getBaseNameAsString(const char* filepath);


#endif /* DYNAMIC_PATH_FINDING_FILE_UTILS_H_ */
