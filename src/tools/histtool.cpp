//
// Path file histogram tool
//
// Error/Usage fct from imgtool.cpp

#include "tools/pathtool.h"
#include <vector>
#include <iterator>
#include <fstream>
#include <ostream>
#include <string>
#include <iomanip>
#include <cstdarg>

/**
 * Input values file structure
 *
 * [Field]
 * ..values selected for histogram sets..
 *
 * Exemple for a filtering by path length
 * [ELength]
 * 1 2 3 4 5 7 9
 *
 * Will generate an histogram showing paths of a given length + other paths
 *
 */

// Manipulation enum
enum EType {
    RMatch,
    RLength,
    ERegex,
    ELength,
    Expr,
    NumTypes,
    EUndef = -1,
};

static const char *TypeCodes[] = {
    "RegMatch",
    "RegLength",
    "Regexpr",
    "ExprLength",
    "Expression"
};

static void usage(const char *msg = nullptr, ...) {
  if (msg) {
    va_list args;
    va_start(args, msg);
    fprintf(stderr, "pathtool: ");
    vfprintf(stderr, msg, args);
    fprintf(stderr, "\n");
  }
  fprintf(stderr, R"(usage: pathtool <command> [options] <filenames...>

commands: makehistogram

makehistogram option:
    syntax: histtool makehistogram inputvalues.txt infile [outfile.txt]

)");
  exit(1);
}


template <typename T>
T getField(const pbrt::path_entry &p, const EType &e) {
  switch(e) {
    case RLength:
      return p.regexlen;
    case ELength:
      return p.pathlen;
    default:
      return 0;
  }
}

ssize_t FindIf(const EType &e, const std::vector<int> values, const pbrt::path_entry &p) {
  int field = getField<int>(p, e);
  auto it = std::find(values.begin(), values.end(), field);
  return std::distance(values.begin(), it);
}

ssize_t FindIf(const EType &e, const std::vector<std::string> values, const pbrt::path_entry &p) {
  std::vector<std::string>::const_iterator it;

  if(e == EType::RMatch) {
    it = std::find_if(values.begin(), values.end(), [&](const std::string &regex) {
        std::regex r(regex);
        return std::regex_match(p.path, r);
    });
  } else if (e == EType::Expr){
    it = std::find(values.begin(), values.end(), p.path);
  } else {
    it = values.end(); // FIXME: exception ?
  }

  return std::distance(values.begin(), it);
}

template <typename T>
std::vector<int> HistogramGenerator(const EType &e, const std::vector<T> values, const std::string &pathfile) {
  std::vector<int> hist;
  hist.resize(values.size() + 1);
  std::fill(hist.begin(), hist.end(), 0);
  PathFile file(pathfile);

  for(const pbrt::path_entry &p : file) {
      ssize_t dist = FindIf(e, values, p);
      ++hist[dist];
  };

  return hist;
}

EType typeParser(std::ifstream &is) {
  std::string str;
  std::getline(is, str);

  for (int i = 0; i < EType::NumTypes; ++i) {
    if(str.find(TypeCodes[i]) == 0) { // if code found in first word
      return (EType)i;
    }
  }

  return EType::EUndef;
}

template <typename T>
void simple_histogram_output(std::ostream &os, const std::vector<T> &labels, const std::vector<int> &values) {
  os << "Histogram:" << std::endl;

  // Find maximum length for labels
  const int width = 20;

  for(int i = 0; i < labels.size(); ++i) {
    os << std::left << std::setw(width) << labels[i] << " : ";
    os << values[i] << std::endl;
  }

  // Unsorted paths
  os << std::left << std::setw(width) << "Other " << " : ";
  os << values[labels.size()] << std::endl;
}

void histogram_generator(int argc, char* argv[]) {
  std::ifstream paramfile;
  std::string infile(argv[3]);
  std::ofstream out;

  // input file parsing
  paramfile.open(argv[2], std::ios::in);
  // Type parsing
  EType type = typeParser(paramfile);
  if(type == EUndef) {
    usage("Undefined parameter type");
  }

  // TODO: handle multiple input types and better parsing
  std::vector<int> values;
  std::copy(std::istream_iterator<int>(paramfile), std::istream_iterator<int>(), std::back_inserter(values));

  // Generate histogram from pathfile
  std::vector<int> histogram = HistogramGenerator(type, values, infile);


  if(argc == 5) {
    out.open(argv[4], std::ios::out);
    simple_histogram_output(out, values, histogram);
  } else {
    simple_histogram_output(std::cout, values, histogram);
  }

  exit(0);
}

int main(int argc, char* argv[]) {

  if(argc == 1) {
    usage("");
    return 1;
  }

  if(!strcmp(argv[1], "makehistogram")) {
    histogram_generator(argc, argv);
  } else {
    usage("");
  }

  return 0;
}
