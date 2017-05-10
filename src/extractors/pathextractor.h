//
//

#ifndef PBRT_EXTRACTOR_PATH_H
#define PBRT_EXTRACTOR_PATH_H

#include <regex>
#include "integrators/bdpt.h"
#include "pbrt.h"

namespace pbrt {

// Vertex structure
struct Vertex;

enum class VertexInteraction { Camera, Light, Diffuse, Specular };
struct PathVertex {
    VertexInteraction type;

    union {
        SurfaceInteraction si;
        EndpointInteraction ei;
        MediumInteraction mi; // TODO: handle medium interactions vertices
    };

    PathVertex() : ei() {}


    static inline PathVertex FromBDPTVertex(Vertex v);

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
    Point2f pOrigin;
    std::vector<PathVertex> vertices;

    std::string GetPathExpression() const {
      std::string s = "";
      for(PathVertex v : vertices) {
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
      // TODO: lower log error level to info
      LOG(FATAL) << "Empty path";
      return "";
    }

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
    PathExtractorContainer(Point2f pFilm) : pFilm(pFilm) {};

    void Init(const RayDifferential &r, int depth, const Scene &Scene);
    // BDPT Camera-complete subpath
    void ReportData(Vertex *cameraVertices, int t);
    // t == 1 / s > 0 light-camera connect
    void ReportData(Vertex *lightVertices, Vertex cameraVertex, int s, VertexInteraction mainPath);
    void ReportData(Vertex *lightVertices, Vertex *cameraVertices, int s, int t);

    // FIXME: this.
    void ReportData(const SurfaceInteraction &isect) {}

    void ReportData(Spectrum beta) {
      L = beta;
    }

    bool isValidPath(const std::regex &pathPattern) const;

    Spectrum ToSample() const {
      return L;
    }

    Path path;
  private:
    // TODO: If possible, check path regex before adding vertices to the container
    void BuildPath(Vertex *lightVertices, Vertex *cameraVertices, int s, int t);

    const Point2f pFilm;
    Spectrum L;
};

class PathExtractor : public ExtractorFunc {
  public:
    PathExtractor(const std::string pathExpression) :
            pathExpression(pathExpression) {};

    std::shared_ptr<Container> GetNewContainer(Point2f p) const {
      return std::shared_ptr<Container>(new PathExtractorContainer(p));
    }

    std::unique_ptr<PathExtractorContainer> GetNewPathExtractorContainer(Point2f p) const {
      return std::unique_ptr<PathExtractorContainer>(new PathExtractorContainer(p));
    }


  private:
    const std::string pathExpression;

};

}

#endif //PBRT_EXTRACTOR_PATH_H
