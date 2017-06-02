//
// Created by stardami on 5/16/17.
//

#include <fstream>
#include "pbrt.h"
#include "paramset.h"
#include "pathoutput.h"
#include "extractor.h"
#include "fileutil.h"

namespace pbrt {

void PathOutputTile::AddSample(const Point2f &pFilm, std::shared_ptr<Container> container) {
  VLOG(2) << "New path sample";
  ProfilePhase p(Prof::AddPathSample);
  std::vector<PathEntry> entries = container->GetPaths();
  tilepaths.insert(tilepaths.end(), entries.begin(), entries.end());
}


std::unique_ptr<PathOutputTile> PathOutput::GetPathTile() {
  return std::unique_ptr<PathOutputTile>(new PathOutputTile());
}

void PathOutput::MergePathTile(std::unique_ptr<PathOutputTile> tile) {
  VLOG(1) << "Merging path tile " << tile->pixelBounds;
  ProfilePhase _(Prof::MergePathTile);
  std::lock_guard<std::mutex> lock(mutex);

  paths.insert(paths.end(), tile->tilepaths.begin(), tile->tilepaths.end());
}

static bool WritePathBinary(const std::string &filename, const std::vector<PathEntry> &entries) {
  return 1;
}

static bool WritePathTextFile(const std::string &filename, const std::vector<PathEntry> &entries) {
  std::basic_ofstream<char> f(filename);

  f << "Path file; n = " << entries.size() << std::endl;
  for(const PathEntry &entry: entries) {
    f << "path expr [ \"" + entry.pathexpr + "\" ] ";
    f << "vertices [ ";
    std::for_each(entry.vertices.begin(), entry.vertices.end(),
                  [&](const Point3f &p) { f << StringPrintf("[ %f, %f, %f ] ", p.x, p.y, p.z); });
    f << " ] normals [ ";
    std::for_each(entry.normals.begin(), entry.normals.end(), [&](const Normal3f &n) { f << n << " "; });
    f << "]" << std::endl;
  }

  f.close();
  return 0;
}

void PathOutput::WriteFile() {
    WritePathTextFile(filename, paths);
}

PathOutput *CreatePathOutput(const ParamSet &params) {
  // Intentionally use FindOneString() rather than FindOneFilename() here
  // so that the rendered image is left in the working directory, rather
  // than the directory the scene file lives in.
  std::string filename = params.FindOneString("filename", "");
  if (filename == "") filename = "pbrt_pathextract.txt";

  return new PathOutput(filename);
}
}