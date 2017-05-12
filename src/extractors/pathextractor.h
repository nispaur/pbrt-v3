//
//

#ifndef PBRT_EXTRACTOR_PATH_H
#define PBRT_EXTRACTOR_PATH_H

#include <regex>
#include "pbrt.h"
#include "extractors/extractor.h"

namespace pbrt {

enum class VertexInteraction { Camera, Light, Diffuse, Specular };
struct PathVertex {
    VertexInteraction type;

    union {
        SurfaceInteraction si;
        Interaction ei; // TODO: Full EndpointInteraction support
        MediumInteraction mi; // TODO: handle medium interactions vertices
    };

    PathVertex() : ei() {}

    static inline PathVertex FromBDPTVertex(const Vertex &v);

    const Point3f &p() const { return GetInteraction().p; }

    const Interaction &GetInteraction() const {
      return si;
    }

    friend std::ostream &operator<<(std::ostream &os, const PathVertex &v) {
      return os << v.ToString();
    }

    std::string ToString() const {
      std::string s = std::string("[v type: ");
      switch (type) {
        case VertexInteraction::Camera:
          s += "camera";
          break;
        case VertexInteraction::Light:
          s += "light";
          break;
        case VertexInteraction::Diffuse:
          s += "diffuse interaction";
          break;
        case VertexInteraction::Specular:
          s += "specular interaction";
          break;
      }
      s += StringPrintf("; p: [ %f, %f, %f ]", p().x, p().y, p().z);
      s += std::string(" ]");
      return s;
    }
};

struct Path {
    Path(int length) {
      vertices.reserve(length);
    }

    Path() {};

    // Point2f pOrigin;
    Spectrum L;
    std::vector<PathVertex> vertices;

    std::string GetPathExpression() const {
      std::string s = "";
      for(const PathVertex &v : vertices) {
        switch (v.type) {
          case VertexInteraction::Camera:
            s += "E";
            break;
          case VertexInteraction::Light:
            s +=  "L";
            break;
          case VertexInteraction::Diffuse:
            s +=  "D";
            break;
          case VertexInteraction::Specular:
            s +=  "S";
            break;
        }
      }

      return s;
    }

    bool isValidPath(const std::regex &pathPattern) const;

    friend std::ostream &operator<<(std::ostream &os, const Path &p) {
      return os << p.ToString();
    }

    std::string ToString() const {
      std::string s = std::string("[Path expr: \"");
      s += GetPathExpression() + "\" ;";
      s += " vertices --> ";

      for(int i = 0; i < vertices.size(); ++i) {
        s += StringPrintf(" p%d: [ %f, %f, %f ] ", i, vertices[i].p().x, vertices[i].p().y, vertices[i].p().z);
      }
      s += std::string(" ]");
      return s;
    }

};

class PathExtractorContainer : public Container {
  public:
    PathExtractorContainer(Point2f pFilm, const std::regex &r) :
            pFilm(pFilm),
            regex(r) {};

    void Init(const RayDifferential &r, int depth, const Scene &Scene);

    void ReportData(const Spectrum &L) {
      auto p = paths.find({s_state,t_state});
      if(p != paths.end())
        paths.at({s_state, t_state}).L = L;
    }

    Spectrum ToSample() const;

    void AddSplat(const Point2f &pSplat, Film *film);

    // TODO: If possible, check path regex before adding vertices to the container
    void BuildPath(const Vertex *lightVertices, const Vertex *cameraVertices, int s, int t);

  private:
    const Point2f pFilm;
    const std::regex regex;
    // path state
    int s_state, t_state;
    std::map<std::pair<int,int>,Path> paths;
};



class PathExtractor : public ExtractorFunc {
  public:
    PathExtractor(const std::string pathExpression) :
      r(std::regex(pathExpression, std::regex::nosubs|std::regex::optimize)) {};

    std::shared_ptr<Container> GetNewContainer(Point2f p) const {
      return std::shared_ptr<Container>(new PathExtractorContainer(p, r));
    }

    std::unique_ptr<PathExtractorContainer> GetNewPathExtractorContainer(Point2f p) const {
      return std::unique_ptr<PathExtractorContainer>(new PathExtractorContainer(p, r));
    }

  private:
    const std::regex r;

};


Extractor *CreatePathExtractor(const ParamSet &params, const Point2i fullResolution,
                               const Float diagonal, const std::string imageFilename);

}

#endif //PBRT_EXTRACTOR_PATH_H
