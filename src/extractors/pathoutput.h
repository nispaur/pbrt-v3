//
//

#ifndef PBRT_EXTRACTOR_PATHOUTPUT_H
#define PBRT_EXTRACTOR_PATHOUTPUT_H

#include "pbrt.h"
#include "core/geometry.h"
#include "core/parallel.h"
#include "core/memory.h"
namespace pbrt {

class PathOutput {
  public:
    PathOutput(const std::string &filename) : filename(filename) {}

    std::unique_ptr<PathOutputTile> GetPathTile();
    void MergePathTile(std::unique_ptr<PathOutputTile> tile);

    void WriteFile();
  private:
    std::mutex mutex;
    std::vector<PathEntry> paths;
    const std::string filename;
};

class PathOutputTile {
  public:
    void AddSample(const Point2f &pFilm, std::shared_ptr<Container> container);
  private:
    MemoryArena arena; // Path storage arena
    // Path entries
    const Bounds2i pixelBounds;
    std::vector<PathEntry> tilepaths;
    friend class PathOutput;
};



PathOutput *CreatePathOutput(const ParamSet &params);

} // namespace pbrt

#endif //PBRT_EXTRACTOR_PATHOUTPUT_H
