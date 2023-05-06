#ifndef VEC_IO_H
#define VEC_IO_H

#include <cstdio>
#include <cstring>
#include <stdexcept>
#include <vector>
#include <iostream>
#include <unordered_set>
#include <unordered_map>
#include "log.h"

template<typename T>
std::ostream & operator<<(std::ostream& str, const std::unordered_set<T>& v) {
	str << "[ ";
	for (auto it=v.begin(); it != v.end(); ++it) {
		str << *it << " ";
	}
	str << "]";
	return str;
}

template <typename K, typename V>
std::ostream& operator <<(std::ostream& stream, const std::unordered_map<K,V>& map) {
	stream << "{ ";
	for (auto el : map) {
		stream << "[" << el.first << ", " << el.second << "], ";
	}
	stream << " }";
	return stream;
}

template<typename T>
std::ostream & operator<<(std::ostream& str, const std::vector<T>& v) {
	str << "[ ";
	for (auto it=v.cbegin(); it != v.cend(); ++it) {
		str << *it << " ";
	}
	str << "]";
	return str;
}

/**
 * store a vector instance into a file
 *
 * @pre
 *  @li @c file open with "wb";
 *
 * @param[inout] file the file to write into
 * @param[in] v the vector to save into the file
 */
template<class T>
void save_vector(std::FILE*file, const std::vector<T>&v){
	int s = v.size();
	if(std::fwrite(&s, sizeof(s), 1, file) != 1)
		throw std::runtime_error("std::fwrite failed");
	if(std::fwrite(&v[0], sizeof(T), v.size(), file) != v.size())
		throw std::runtime_error("std::fwrite failed");
}

/**
 * Load a vector from a file
 *
 * @pre
 *  @li @c file open with "rb";
 *
 * @param[inout] file the file to read
 * @return a vector instance which has been just read from @c file
 */
template<class T>
std::vector<T>load_vector(std::FILE*file){
	int s;
	if(std::fread(&s, sizeof(s), 1, file) != 1)
		throw std::runtime_error("std::fread failed");
	std::vector<T>v(s);

	size_t stuffRead = std::fread(&v[0], sizeof(T), s, file);
	if((int)stuffRead != s) {
		error("we were expecting to read ", s, " but we read", stuffRead, "elements instead");
		throw std::runtime_error("std::fread failed");
	}

	return v; // NVRO
}

/**
 * Convert an array of bytes to a vector of T.
 */
template<class T>
std::vector<T> load_vector(const char* &ss) {
        // unsigned char temp;
        int s, i = 0, st = sizeof(T);

        memcpy(&s, ss, sizeof(int));
        ss += sizeof(int);

        std::vector<T> v(s);

        while (i < s) {
          T c;

          memcpy(&c, ss, st);
          ss += st;
          v.at(i) = c;
          i++;
        }

        return v;
}

#endif
