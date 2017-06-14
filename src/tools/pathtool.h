//
// Path file manipulation tool
//

#ifndef PBRT_EXTLIB_PATHTOOL_H
#define PBRT_EXTLIB_PATHTOOL_H

#include "extractors/pathio.h"
#include <unistd.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <sys/mman.h>
#include <bits/mman.h>
#include <bits/mman-linux.h>
#include <cstring>
#include <cerrno>

#define MAP_HUGETLB	0x40000		/* Create huge page mapping.  */
#define MAP_POPULATE	0x08000		/* Populate (prefault) pagetables.  */
#define MAP_NORESERVE	0x04000		/* Don't check for reservations.  */

// Regex match
#include <regex>

// Path manipulation structs
struct vertex_entry {
    uint32_t type;  // 4
    std::array<float,3> v;     // 12
    std::array<float,3> n;     // 12
    std::array<float,3> bsdf;  // 12 (RGBSpectrum)
    float pdf_in;   // 4
    float pdf_out;  // 4
    // = 48 bytes
};

struct path_entry {

    uint32_t regexlen;                         // 4 o
    uint32_t pathlen;                          // 4 o
    char* regex;                               // regexlen o
    char* path;                                // pathlen o
    vertex_entry *vertices;         // 48 * pathlen o
    // = 8 + regexlen + (49 * pathlen) bytes

/*
    // Default constructor
    path_entry() : regexlen(0), pathlen(0) {}

    // Copy constructor
    path_entry(const path_entry& e) : regexlen(e.regexlen), pathlen(e.pathlen) {
      regex = new char[e.regexlen];
      path = new char[e.pathlen];
      vertices = new vertex_entry[e.pathlen];
      std::copy(e.regex, e.regex+e.regexlen, regex);
      std::copy(e.path, e.path+e.pathlen, path);
      std::copy(e.vertices, e.vertices+e.pathlen, vertices);
    }
*/
};

class PathFile {
  public:
    // Container typedefs

    template <typename T = pbrt::path_entry, typename A = std::allocator<T>>
    class pathconst_iterator {
      public:
        typedef std::size_t size_type;
        typedef typename A::difference_type difference_type;
        typedef typename A::value_type value_type;
        typedef typename A::reference const_reference;
        typedef typename A::pointer const_pointer;
        typedef std::forward_iterator_tag iterator_category;

        pathconst_iterator (void *startptr, size_type offset = 0) {
          pathfile = startptr;
          cpos = (T*)startptr;
          *this += offset;
        }

        // TODO: make iterator copy construcible ?
        pathconst_iterator (const pathconst_iterator &it) : pathfile(it.pathfile), cpos(it.cpos) {}
        ~pathconst_iterator() {}

        pathconst_iterator& operator=(const pathconst_iterator& it) {
          pathfile = it.pathfile;
          cpos = it.cpos; // ?
        }
        // TODO: full iterator content compare (ForwardIterator requirements)
        bool operator==(const pathconst_iterator &it) const { return cpos == it.cpos; }
        bool operator!=(const pathconst_iterator&it) const { return cpos != it.cpos; }
        bool operator<(const pathconst_iterator &it) const { return cpos < it.cpos; }
        bool operator>(const pathconst_iterator &it) const { return cpos > it.cpos; }
        bool operator<=(const pathconst_iterator &it) const { return cpos <= it.cpos; }
        bool operator>=(const pathconst_iterator &it) const { return cpos >= it.cpos; }

        pathconst_iterator& operator++() {
          cpos = (pbrt::path_entry*)((char*)cpos + 2*sizeof(uint32_t) + cpos->regexlen + cpos->pathlen * (1 + sizeof(pbrt::vertex_entry)));
          return *this;
        }

        pathconst_iterator operator++(int n) {
          for (int i = 0; i < n; ++i) {
            cpos = (pbrt::path_entry*)((char*)cpos + 2*sizeof(uint32_t) + cpos->regexlen + cpos->pathlen * (1 + sizeof(pbrt::vertex_entry)));
          }
          return *this;
        }

        pathconst_iterator& operator+=(size_type n) {
          for (int i = 0; i < n; ++i) {
            cpos = (pbrt::path_entry*)((char*)cpos + 2*sizeof(uint32_t) + cpos->regexlen + cpos->pathlen * (1 + sizeof(pbrt::vertex_entry)));
          }
          return *this;
        }

        // Bidirectionnal iterator operators, path table needed
        // const_iterator& operator--(); //optional
        // const_iterator operator--(int); //optional
        // const_iterator operator+(size_type) const; //optional
        // friend const_iterator operator+(size_type, const const_iterator&); //optional
        // const_iterator& operator-=(size_type); //optional
        // const_iterator operator-(size_type) const; //optional
        // difference_type operator-(const_iterator) const; //optional

        const_reference operator*()  {
          cpath.regexlen = cpos->regexlen;
          cpath.pathlen = cpos->pathlen;
          cpath.regex.resize(cpath.regexlen);
          memcpy((void*)(cpath.regex.data()), (void*)((char*)(cpos) + 8), cpath.regexlen);
          cpath.path.resize(cpath.pathlen);
          memcpy((void*)(cpath.path.data()), (void*)((char*)(cpos) + 8 + cpath.regexlen), cpath.pathlen);
          cpath.vertices.resize(cpath.pathlen);
          memcpy((void*)cpath.vertices.data(), (void*)((char*)(cpos)+8+cpath.regexlen+cpath.pathlen), cpath.pathlen*sizeof(vertex_entry));
          return cpath;
        }
        const_pointer operator->() const {
          return cpos;
        }

      private:
        // Iterator attrs
        //pbrt::path_entry cpath;
        void *pathfile;
        pbrt::path_entry *cpos; // Current path position in file
        pbrt::path_entry cpath;

    };

    // Container typedefs
    typedef std::size_t size_type;
    typedef PathFile::pathconst_iterator<pbrt::path_entry> const_iterator;

    PathFile(const std::string &filename) : fd(open(filename.c_str(), O_RDONLY)) {
      fstat(fd, &stats);
      std::cout << "Executing mmap with args: " << stats.st_size << " fd " << fd << std::endl;
      if((filemap = (int8_t*)mmap64(nullptr, stats.st_size, 0x1, 0xC001, fd, 0)) == (int8_t *)(-1)) { // 0x0C001
        std::cerr << "mmap error "  << std::strerror(errno) << std::endl;
        exit(EXIT_FAILURE);
      }
      // Read header and move map ptr to first path
      char *pathcountptr = strstr((char*)filemap, "Path file; n = ");
      pathcount = strtol(pathcountptr+15, nullptr, 10);

      // Place pointer to first path
      std::cout << "Loaded file, " << pathcount << " paths." << std::endl;
      first_path = (int8_t*)memchr(filemap, '\n', 80) + 1;
      std::cout << "First path pathlength = " << reinterpret_cast<pbrt::path_entry*>(first_path)->pathlen << std::endl;

      find_lastpath();
      int avg_len = average_length();
      std::cout << "Average path length: " << avg_len << std::endl;
    }

    // iterator methods
    const_iterator begin() const {
      return PathFile::const_iterator(first_path);
    }

    // past the end iterator
    const_iterator end() const {
      return PathFile::const_iterator(last_pos);
    }

    size_type size() const { return pathcount; }

    int average_length() const {
      int totallength = 0;
      int pcount = 0;
      for(const pbrt::path_entry &p : *this) {
        totallength += p.pathlen;
        ++pcount;
      }
      return pcount == 0 ? 0 : totallength/pcount;
    }

    bool eof() const {
      return current_pos >= pathcount;
    }

    ~PathFile() {
      munmap(filemap, stats.st_size);
      close(fd);
    }

  private:
    void find_lastpath() {
      currpath_ptr = first_path;
      current_path.pathlen = ((path_entry*)first_path)->pathlen;
      current_path.regexlen = ((path_entry*)first_path)->regexlen;
      current_pos = 0;

      while(!eof()) {
        NextPath();
      }

      last_pos = currpath_ptr; // last path
      currpath_ptr = first_path;
      current_path.pathlen = ((path_entry*)first_path)->pathlen;
      current_path.regexlen = ((path_entry*)first_path)->regexlen;
      current_pos = 0;
    }

    void NextPath() {
      // Suppose que le pathptr est correctement placé
      if(currpath_ptr && current_pos < pathcount)
        currpath_ptr = currpath_ptr + 2*sizeof(uint32_t) + current_path.regexlen + current_path.pathlen * (1 + sizeof(pbrt::vertex_entry));

      memcpy(&current_path, currpath_ptr, 2*sizeof(uint32_t));
      current_path.regex = (char*)currpath_ptr+current_path.regexlen+current_path.pathlen;
      current_path.path = (char*)current_path.regex + current_path.regexlen;
      current_path.vertices = (vertex_entry*)(current_path.path + current_path.pathlen);
      ++current_pos;
    }


    const int fd;
    struct stat stats;
    size_t pathcount;
    int8_t *filemap;
    int8_t *first_path;
    int8_t *currpath_ptr;
    path_entry current_path;
    int current_pos;
    int8_t *last_pos;
};


// Path select functions

// Length
static bool isLength(const path_entry &path, int length) {
  return path.pathlen == length;
}

// Regex match
static bool regMatch(const pbrt::path_entry &path, const std::string &regexstr) {
  std::regex reg(regexstr);
  return !strcmp(regexstr.c_str(), path.regex.c_str()) | std::regex_match(path.path, reg);
}

// Vertices around a sphere of radius r
static bool sphereSearch(const pbrt::path_entry &path, float r, float pos[3]) {
  for (int i = 0; i < path.pathlen; ++i) {
    float v[3] = {path.vertices[i].v[0], path.vertices[i].v[1], path.vertices[i].v[2]};
    float sum = 0.f;
    bool boundcheck = true;
    for (int j = 0; j < 3; ++j) {
      sum += (v[i] - pos[i])*(v[i] - pos[i]);
    }

    if(sum < r*r)
      return true;
  }

  return false;
}



#endif //PBRT_EXTLIB_PATHTOOL_H
