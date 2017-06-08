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
  std::vector<path_entry> entries = container->GetPaths();
  tilepaths.insert(tilepaths.end(), entries.begin(), entries.end());
}


std::unique_ptr<PathOutputTile> PathOutput::GetPathTile() {
  return std::unique_ptr<PathOutputTile>(new PathOutputTile());
}

void PathOutput::MergePathTile(std::unique_ptr<PathOutputTile> tile) {
  VLOG(1) << "Merging path tile " << tile->pixelBounds;
  ProfilePhase _(Prof::MergePathTile);
  std::lock_guard<std::mutex> lock(mutex);

  // Path addition during rendering disabled (currently: slowing down rendering in text mode due to formatting)
  // AppendPaths(tile->tilepaths);
  paths.insert(paths.end(), tile->tilepaths.begin(), tile->tilepaths.end());
}

void PathOutput::AppendPaths(const std::vector<path_entry> &entries, bool binarymode) {
  ProfilePhase p(Prof::PathWriteOutput);
  int avg = 0;
  f << "Path file; n = " << entries.size() << std::endl;

  for(const path_entry &entry: entries) {
    avg += entry.vertices.size();
    f << "Path:";
    if(!binarymode) {
      std::ostringstream str;
      str << entry;
      f << str.str() << "\n";
    } else {
      f << entry;
    }
  }

  f << "Average path length: ";
  f << ((!entries.size()) ? 0 : avg/entries.size()) << std::endl;
  f.close();

}

void PathOutput::WriteFile() {
    ProfilePhase p(Prof::PathWriteOutput);
    // Check extension for binary/text mode
    AppendPaths(paths, !HasExtension(filename, ".txtdump"));
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