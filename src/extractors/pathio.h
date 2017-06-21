//
// Created by stardami on 6/6/17.
//

#include <array>
#include <iterator>
#include "pbrt.h"
#ifndef PBRT_EXTRACTORS_PATHIO_H
#define PBRT_EXTRACTORS_PATHIO_H

/*
 * Reminder: path file structure
 * - Header:
 *    - Number of paths
 *    - Path/Expr/Vertices/Normals table offsets in file
 * - Path table; for each path
 *    - Number of vertices
 *    - Path/Expr/Normals first inde
 * - Expr table: size total_vertices bytes, char format
 * - Vertex/Normal table: total_vertices * 3 * 4 bytes each
 */
namespace pbrt {

struct vertex_entry {
    uint32_t type;  // 4
    std::array<Float,3> v;     // 12
    std::array<Float,3> n;     // 12
    std::array<Float,3> bsdf;  // 12 (RGBSpectrum)
    Float pdf_in;   // 4
    Float pdf_out;  // 4
                    // = 48 bytes
};

struct path_entry {
    uint32_t regexlen;                          // 4 o
    uint32_t pathlen;                           // 4 o
    std::string regex;                          // regexlen o
    std::string path;                           // pathlen o
    std::vector<vertex_entry> vertices;         // 48 * pathlen o
    // = 8 + regexlen + (49 * pathlen) bytes

    // Copy constructor
    path_entry() {}
    path_entry(const path_entry& e) : regexlen(e.regexlen), pathlen(e.pathlen), regex(e.regex), path(e.path),
                                      vertices(e.vertices) {}

};

// Move to path_entry ?
static size_t path_size(const path_entry &p) {
  return 8 + p.regexlen + p.pathlen * (1 + sizeof(vertex_entry));
}

inline std::istream &operator>>(std::istream &is, vertex_entry &v) {
    return is.read((char*)&v, sizeof(vertex_entry));
}

inline std::istream &operator>>(std::istream &is, path_entry &entry) {
    is.read((char*)&entry.regexlen, sizeof(uint32_t));
    is.read((char*)&entry.pathlen, sizeof(uint32_t));
    entry.regex.reserve(entry.regexlen);
    entry.path.reserve(entry.pathlen);
    is.read(&entry.regex[0], entry.regexlen);
    is.read(&entry.path[0], entry.pathlen);

    entry.vertices.resize(entry.pathlen);
    is.read((char*)&entry.vertices[0], sizeof(vertex_entry) * entry.pathlen);

    return is;
}

inline std::ostream &operator<<(std::ostream &os, const vertex_entry &v) {
  return os.write((char*)&v, sizeof(vertex_entry));
}

inline std::ostream &operator<<(std::ostream &os, const path_entry &entry) {
  os.write((char*)(&entry.regexlen), sizeof(uint32_t));
  os.write((char*)(&entry.pathlen), sizeof(uint32_t));
  os.write(entry.regex.c_str(), entry.regexlen);
  os.write(entry.path.c_str(), entry.pathlen);
  os.write((char*)(&entry.vertices[0]), sizeof(vertex_entry) * entry.pathlen);

  return os;
}


inline std::ostringstream &operator<<(std::ostringstream &os, const path_entry &entry) {
  os << "path r [ \"" + entry.regex + "\", e \"" + entry.path + "\" ] v [ ";
  std::for_each(entry.vertices.begin(), entry.vertices.end(), [&](const vertex_entry &v) {
      os << "{";
      os << " v: " << v.v[0] << " " <<v.v[1]<<" "<<v.v[2];
      os << " n: " << v.n[0] << " " << v.n[1] << " " << v.n[2];
      os << " f: " << v.bsdf[1] << " " << v.bsdf[2] << " " << v.bsdf[3];
      os << " pdf_in: " << v.pdf_in << " pdf_out: " << v.pdf_out;
      os << " },";
  });
  os << "]";
  return os;
}

}


#endif //PBRT_V3_PATHIO_H
