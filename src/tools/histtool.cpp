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
    ELengthIval,
    Expr,
    NumTypes,
    EUndef = -1,
};

static const char *TypeCodes[] = {
    "RegMatch",
    "RegLength",
    "Regexpr",
    "ExprLength",
    "ExprLengthInterval",
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

ssize_t path_select(const std::vector<std::string> &values, const pbrt::path_entry &p) {
  auto it = std::find(values.begin(), values.end(), p.path);
  return std::distance(values.begin(), it);
}

ssize_t reg_select(const std::vector<std::string> &values, const pbrt::path_entry &p) {
  auto it = std::find_if(values.begin(), values.end(), [&](const std::string &regex) {
      std::regex r(regex);
      return std::regex_match(p.path, r);
  });
  return std::distance(values.begin(), it);
}

ssize_t length_select(const std::vector<int> &values, const pbrt::path_entry &p) {
  auto it = std::find(values.begin(), values.end(), p.pathlen);
  return std::distance(values.begin(), it);
}

ssize_t lengthival_select(const std::vector<int> &values, const pbrt::path_entry &p) {
  for (auto it = values.begin(); it < (values.end() - 1); ++it) {
    if(p.pathlen >= *it && p.pathlen < *(it+1))
      return std::distance(values.begin(), it);
  }
  return std::distance(values.begin(), values.end()); // = values.size()
}

void fun_dispatcher(const EType &e, std::function<ssize_t(const std::vector<std::string>&, const pbrt::path_entry&)> &fun) {
  if (e == EType::RMatch) {
    fun = reg_select;
  } else if (e == EType::Expr) {
    fun = path_select;
  }
}

void fun_dispatcher(const EType &e, std::function<ssize_t(const std::vector<int>&, const pbrt::path_entry&)> &fun) {
  if(e == EType::ELengthIval) {
    fun = lengthival_select;
  } else if (e == EType::ELength) {
    fun = length_select;
  }
}

template <typename T>
std::vector<int> HistogramGenerator(const EType &e, const std::vector<T> values, const std::string &pathfile) {
  std::vector<int> hist;
  hist.resize(values.size() + 1);
  std::fill(hist.begin(), hist.end(), 0);
  PathFile file(pathfile);

  std::function<ssize_t(const std::vector<T>&, const pbrt::path_entry&)> f;
  fun_dispatcher(e, f);

  for(const pbrt::path_entry &p : file) {
      ++hist[f(values, p)];
  }

  return hist;
}

EType typeParser(std::ifstream &is) {
  std::string str;
  std::getline(is, str);

  for (int i = 0; i < EType::NumTypes; ++i) {
    if(str.compare(TypeCodes[i]) == 0) { // if code found in first word
      return (EType)i;
    }
  }

  return EType::EUndef;
}


void simple_histogram_output(std::ostream &os, const std::vector<std::string> &labels, const std::vector<int> &values) {
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
  std::vector<int> histogram;

  // input file parsing
  paramfile.open(argv[2], std::ios::in);

  // Type parsing
  EType type = typeParser(paramfile);
  if(type == EUndef) {
    usage("Undefined parameter type");
  }

  // TODO: one vector ?
  std::vector<std::string> strvalues;
  std::vector<int> intvalues;

  if(type == ELength || type == ELengthIval) {
    // TODO: handle multiple input types and better parsing
    std::copy(std::istream_iterator<int>(paramfile), std::istream_iterator<int>(), std::back_inserter(intvalues));
    // Conversion for labels
    std::for_each(intvalues.begin(), intvalues.end(), [&](int v) { strvalues.push_back(std::to_string(v)); });

    histogram = HistogramGenerator(type, intvalues, infile);
  } else {
    std::copy(std::istream_iterator<std::string>(paramfile), std::istream_iterator<std::string>(),
              std::back_inserter(strvalues));
    histogram = HistogramGenerator(type, strvalues, infile);
  }


  if(argc == 5) {
    out.open(argv[4], std::ios::out);
    simple_histogram_output(out, strvalues, histogram);
  } else {
    simple_histogram_output(std::cout, strvalues, histogram);
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
