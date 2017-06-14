//
// Path file manipulation tool
//
// Error/Usage fct from imgtool.cpp

#include "extractors/pathio.h"
#include <cstdio>
#include <string>
#include <cstring>
#include <iostream>
#include <vector>
#include "pbrt.h"
#include <fstream>
#include "tools/pathtool.h"
#include <sstream>
#include <fstream>
#include <cstdarg>

namespace pbrt {


void bin_to_txt(int argc, char *argv[]);
void path_grep(int argc, char *argv[]);
void print_stats(int argc, char *argv[]);
void align_check(int argc, char *argv[]);
void mmap_test(int argc, char *argv[]);

static void usage(const char *msg = nullptr, ...) {
  if (msg) {
    va_list args;
    va_start(args, msg);
    fprintf(stderr, "pathtool: ");
    vfprintf(stderr, msg, args);
    fprintf(stderr, "\n");
  }
  fprintf(stderr, R"(usage: pathtool <command> [options] <filenames...>

commands: cat, aligncheck, spherefilter, regexfilter, lengthfilter

cat option:
    --outfile          Output file name

lengthfilter option:
    syntax: pathtool lengthfilter <length> <filename>

regexfilter option:
    syntax: pathtool regexfilter <regex> <filename>

spherefilter option:
    syntax: pathtool spherefilter <radius> <x> <y> <z> filename

)");
  exit(1);
}

void align_check(int argc, char *argv[]) {
  if(argc != 3) {
    usage("Error: no file provided");
  }

  FILE *fp = fopen(argv[2], "r");

  if(!fp) {
    perror("Input file error:");
    exit(EXIT_FAILURE);
  }

  // Read header
  char header[80];
  char *pathcountptr;
  if(!fgets(header, 79, fp) || !(pathcountptr = strstr(header, "Path file; n = "))) {
    perror("Error finding header");
    exit(EXIT_FAILURE);
  }

  int64_t pathcount = strtol(pathcountptr+15, nullptr, 10); // 15 = length of header string "Path file.."
  std::cout << "Header found, reported path count: " << pathcount << std::endl;

  char buf[10];
  int failcount = 0;
  // For now just check for path alignement
  for (int64_t i = 0; i < pathcount; ++i) {
    // Check for path header consistency
    /*fread(buf, 6, 1, fp);
    if(strncmp("Path:", buf, 5)) {
      std::cout << "Error: inconsistent path " << i << ". Header string \"" << buf << "\"" << std::endl;
      ++failcount;
    }
     */

    // FIXME: path/reglen saved in uint8_t instead of uint32_t
    uint32_t reglen; // = (uint32_t)(buf[5] - '0');
    uint32_t pathlen; // = (uint32_t)(buf[6] - '0');
    fread(&reglen, 4, 1, fp);
    fread(&pathlen, 4, 1, fp);

    // Skip the approximed path length (regxp+path string bytes + pathlen vertex entries)
    long int offset = reglen + pathlen * (1 + sizeof(vertex_entry));

    fseek(fp, offset, SEEK_CUR);
    // TODO: path coherence check (normalized vectors, regexp/expr check, plausible path, pdf values..)

    if(!(i % (1024*512))){
      std::cout << "[" << (i/(float)pathcount)*100.f << "%] \t" << i << " paths checked out of " << pathcount << std::endl;
    }
  }

  if(!failcount) {
    std::cout << "Check finished, no alignement error found" << std::endl;
  } else {
    std::cout << "Check finished, " << failcount << " errors found." << std::endl;
  }

}

void bin_to_txt(int argc, char *argv[]) {
  if(argc == 2)
    usage("no file provided");

  // TODO: Vérifier intégrité du fichier avant conversion

  char *infile = nullptr;
  char *outfile = nullptr;
  bool aligncheck = false;

  // Get opts
  if(argc == 5 && !strcmp(argv[2], "--outfile")) {
    outfile = argv[3];
    infile = argv[4];
  } else if (argc == 4 && !strcmp(argv[2], "--tostdout")) {
    outfile = 0; // To stdout
    infile = argv[3];
  } else usage("Invalid argument");

  FILE *fi = nullptr;
  FILE *fo = nullptr;

  if(!(fi = fopen(infile, "r"))) {
    perror("Input fopen:");
    exit(EXIT_FAILURE);
  }

  if(!(fo = !outfile ? stdout : fopen(outfile, "w"))) {
    perror("Output file error:");
    exit(EXIT_FAILURE);
  }

  // Skip header and read file
  char header[80];
  char *pathcountptr;
  if(!fgets(header, 79, fi) || !(pathcountptr = strstr(header, "Path file; n = "))) {
    perror("Error finding header");
    exit(EXIT_FAILURE);
  }

  int64_t pathcount = strtol(pathcountptr, nullptr, 10);
  std::cout << "Header found, reported path count: " << pathcount << std::endl;

  std::cout << header << std::endl;
  int cpt = 0;
  char buf[10]; // tmpbuf
  while(!pathcount--) {
    path_entry path;

    // Check for path header consistency
    fread(buf, 7, 1, fi);
    if(strncmp("Path:", buf, 5)) {
      std::cout << "Error: inconsistent path " << cpt << ". Header string \"" << buf << "\"" << std::endl;
    }

    // FIXME: path/reglen saved in uint8_t instead of uint32_t
    path.regexlen = (uint32_t) (buf[5] - '0');
    path.pathlen = (uint32_t) (buf[6] - '0');

    //fread(&path.regexlen, 4, 1, fi);
    //fread(&path.pathlen, 4, 1, fi);
    path.regex.resize(path.regexlen);
    path.path.resize(path.pathlen);
    fread(&path.regex[0], 1, path.regexlen, fi);
    fread(&path.path[0], 1, path.pathlen, fi);
    path.vertices.resize(path.pathlen);
    fread(&path.vertices[0], sizeof(vertex_entry), path.pathlen, fi);

    std::ostringstream str;
    str << path;
    /*
    std::cout << "Path/regex size (real/announced): " << "(" << path.pathlen << "," << path.path.size() << ")" <<
    " (" << path.regexlen << "," << path.regex.size() << "). " << str.str() << std::endl;
    */
    ++cpt;

    fwrite(str.str().c_str(), sizeof(char), str.str().size(), fo);
    fputc('\n', fo);
  }

  fclose(fi);
  fflush(fo);
  fclose(fo);
}

// PathFile version of cat
void bin_to_txt2(int argc, char *argv[]) {
  std::string filename(argv[3]);
  std::ofstream out(argv[2]);

  PathFile file(filename);

  for(const pbrt::path_entry &p : file) {
    std::ostringstream str;
    str << p;
    out << str.str() << std::endl;
  }
}

} // namespace pbrt


void mmap_test(int argc, char* argv[]) {
  std::string filename(argv[2]);
  PathFile p(filename);

  for (const pbrt::path_entry &paths : p) {
    std::cout << "Path length " << paths.pathlen << std::endl;
  }

  std::cout << "Average path length: " << p.average_length() << std::endl;

  std::string test;
  std::cin >> test;
}

void filter_by_length(int argc, char* argv[]) {
  if(argc != 4) {
    pbrt::usage();
  }

  // TODO: except/file name checks
  int length = std::atoi(argv[2]);
  std::string filename(argv[3]);

  std::cout << "Filename " << filename << "; legnth = " << length << std::endl;

  PathFile file(filename);
  int n = 0;
  std::for_each(file.begin(), file.end(), [&](const pbrt::path_entry &p) { n += p.pathlen == length ? 1 : 0; });

  std::cout << "Total paths of length " << length << " : " << n << std::endl;
}

void filter_by_location(int argc, char* argv[]) {
  if(argc != 7) {
    pbrt::usage();
  }

  float radius = std::stof(argv[5]);
  float pos[3] = {std::stof(argv[2]), std::stof(argv[3]), std::stof(argv[4])};
  std::string filename(argv[6]);

  std::cout << "Matching paths passing sphere of center " << pos[0] << ", " << pos[1] << ", " << pos[2] << " and radius " << radius << std::endl;
  PathFile file(filename);
  std::vector<pbrt::path_entry> resultpaths;
  std::copy_if(file.begin(), file.end(), std::back_inserter(resultpaths),
               [&](const pbrt::path_entry &p) { return sphereSearch(p, radius, pos); } );

  std::cout << "Number of paths matching : " << resultpaths.size() << std::endl;
}

void filter_by_regex(int argc, char* argv[]) {
  if(argc != 4) {
    pbrt::usage();
  }

  std::string filename(argv[3]);
  std::string regex(argv[2]);

  PathFile file(filename);
  std::vector<pbrt::path_entry> resultpaths;
  std::copy_if(file.begin(), file.end(), std::back_inserter(resultpaths),
               [&](const pbrt::path_entry &p)
               {
                   /*
                   if(regMatch(p, regex)) {
                     std::cout << "Matching path regex = " << p.regex << "; expr = " << p.path << std::endl;
                     return true;
                   } else {
                     std::cout << "Unmatched path regex = " << p.regex << "; expr = " << p.path << std::endl;
                     return false;
                   }
                   */
                   return regMatch(p, regex);
               } );

  std::cout << "Number of paths matching : " << resultpaths.size() << std::endl;

}


int main(int argc, char* argv[]) {

  if(argc == 1) {
    pbrt::usage("");
    return 1;
  }

  if(!strcmp(argv[1], "cat")) {
    pbrt::bin_to_txt(argc, argv);
  } else if(!strcmp(argv[1], "aligncheck")) {
    pbrt::align_check(argc, argv);
  } else if (!strcmp(argv[1], "mmaptest")) {
    mmap_test(argc, argv);
  } else if (!strcmp(argv[1], "lengthfilter")) {
    filter_by_length(argc, argv);
  } else if (!strcmp(argv[1], "regexfilter")) {
    filter_by_regex(argc, argv);
  } else if (!strcmp(argv[1], "spherefilter")) {
    filter_by_location(argc, argv);
  } else if (!strcmp(argv[1], "cat2")) {
    pbrt::bin_to_txt2(argc, argv);
  } else {
    pbrt::usage("");
  }

  return 0;
}
