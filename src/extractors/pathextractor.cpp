//
// Created by stardami on 5/3/17.
//

#include "extractors/pathextractor.h"
#include "integrators/bdpt.h"
#include "pbrt.h"
#include "paramset.h"
#include "filters/box.h"
#include <regex>
#include <algorithm>

namespace pbrt {

void PathExtractorContainer::Init(const RayDifferential &r, int depth, const Scene &Scene) {
  // Path tracing bounce init
  if(depth == 0) {
    path_integrator = true;
    // Path setup
    current_path = Path();

    // Eye vertex setup
    PathVertex eye(r.o, VertexInteraction::Camera);
    current_path.vertices.push_back(eye);
  } else {
    // New potential bounce

  }
}

void PathExtractorContainer::ReportData(const SurfaceInteraction &isect) {
  // New surface interaction = confirmed bounce
  current_path.vertices.push_back(PathVertex(isect));
};

void PathExtractorContainer::BuildPath(const Vertex *lightVertices, const Vertex *cameraVertices, int s, int t) {
  ProfilePhase p(Prof::PathExtractorBuildPath);
  // Light->Camera order
  // Light to camera vertices
  current_path = Path(s+t);

  std::for_each(lightVertices, lightVertices+s, [&](const Vertex &v) {
      current_path.vertices.push_back(PathVertex::FromBDPTVertex(v)); });

  // Camera to light vertices, must be added in reverse order
  for (int i = t - 1; i >= 0; --i) {
    current_path.vertices.push_back(PathVertex::FromBDPTVertex(cameraVertices[i]));
  }

  // Save last path state even if not valid
  s_state = s;
  t_state = t;

  // TODO: constructor for Path
  //if(!path.isValidPath(regex))
  //  return;

  // paths[{s,t}] = path;
}

Spectrum PathExtractorContainer::ToSample() const {
  Spectrum L(0.f);
  for (const auto &path : paths) {
      VLOG(2) << "New matching path" << path.second << "\n";
      L += path.second.L;
  }
  return L;
}

void PathExtractorContainer::AddSplat(const Point2f &pSplat, Film *film) {
  // TODO: group w/ ToSample
  auto path = paths.find({s_state, t_state});
  if(path != paths.end()) {
    film->AddSplat(pSplat, path->second.L);

    paths.erase(path); // Remove path from sampled paths vector
  }
}

void PathExtractorContainer::ReportData(const Spectrum &L) {
  // Pseudo endpoint vertex
  if(path_integrator) {
    // Remove last vertex if no intersection occured
    if(current_path.vertices.back().type == VertexInteraction::Undef)
      current_path.vertices.pop_back();
    /*
    PathVertex fakelight;
    fakelight.type = VertexInteraction::Light;
    current_path.vertices.push_back(fakelight);
    */
    // FIXME: Reverse path for regex compatibility
    std::reverse(current_path.vertices.begin(), current_path.vertices.end());
  }

  current_path.L = L;
  if(!(L.IsBlack() && t_state != 1) && current_path.isValidPath(regex)) {
    VLOG(2) << "New matching path" << current_path << "\n";
    paths[{s_state, t_state}] = current_path;
  } else if (!L.IsBlack()) {
    VLOG(3) << "Incorrect path" << current_path << "\n";
  } else {
    VLOG(2) << "Ignored path (No radiance)" << current_path << "\n";
  }

  current_path = Path();
}

void PathExtractorContainer::ReportData(BxDFType T) {
  current_path.vertices.back().type = (T & BSDF_SPECULAR) != 0 ? VertexInteraction::Specular : VertexInteraction::Diffuse;
}


bool Path::isValidPath(const std::regex &pathPattern) const {
  ProfilePhase p(Prof::PathExtractorRegexTest);
  const std::string pathExpr = GetPathExpression();
  std::smatch base_match;

  return std::regex_match(pathExpr, base_match, pathPattern);
}


PathVertex PathVertex::FromBDPTVertex(const Vertex &v) {
  switch(v.type) {
    case VertexType::Light:
      return PathVertex(v.ei, VertexInteraction::Light);
    case VertexType::Camera:
      return PathVertex(v.ei, VertexInteraction::Camera);
    case VertexType::Surface:
      return PathVertex(v.si, v.delta ? VertexInteraction::Specular : VertexInteraction::Diffuse);
    default:
      LOG(FATAL) << "BDPT Vertex type not supported (Medium interaction) ?";
  }
}


Extractor *CreatePathExtractor(const ParamSet &params, const Point2i &fullResolution,
                               Float diagonal, const std::string &imageFilename) {
  std::string filename = params.FindOneString("outputfile", "");
  if (filename == "") filename = "pextract_" + imageFilename;

  std::string regex = params.FindOneString("regex", "");

  return new Extractor(new PathExtractor(regex), new Film(
          fullResolution,
          Bounds2f(Point2f(0, 0), Point2f(1, 1)),
          std::unique_ptr<Filter>(CreateBoxFilter(ParamSet())),
          diagonal, filename, 1.f));
}

}