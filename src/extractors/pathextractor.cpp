//
// Created by stardami on 5/3/17.
//

#include "extractors/pathextractor.h"
#include "integrators/bdpt.h"
#include "pbrt.h"
#include "paramset.h"
#include "filters/box.h"
#include <regex>

namespace pbrt {

void PathExtractorContainer::Init(const RayDifferential &r, int depth, const Scene &Scene) {
  validPath = false;
}

void PathExtractorContainer::BuildPath(const Vertex *lightVertices, const Vertex *cameraVertices, int s, int t) {
  // Light->Camera order
  // Light to camera vertices
  Path path;
  for (int i = 0; i < s; ++i) {
    path.vertices.push_back(PathVertex::FromBDPTVertex(lightVertices[i]));
  }
  // Camera to light vertices, must be added in reverse order
  for (int i = t - 1; i >= 0; --i) {
    path.vertices.push_back(PathVertex::FromBDPTVertex(cameraVertices[i]));
  }

  // TODO: constructor for Path
  validPath = true;
  path.s = s_state = s;
  path.t = t_state = t;
  paths[{s,t}] = path;
}

Spectrum PathExtractorContainer::ToSample() const {
  Spectrum L(0.f);
  for (std::map<std::pair<int,int>,Path>::const_iterator it=paths.begin(); it!=paths.end(); ++it) {
    if (it->first.second != 1 && it->second.isValidPath(regex)) {
      VLOG(2) << "New matching path" << it->second << "\n";
      L += it->second.L;
    }
  }
  return L;
}

void PathExtractorContainer::AddSplat(const Point2f &pSplat, Film *film) {
  // TODO: group w/ ToSample
  Path path = paths[{s_state, t_state}];
  if(path.isValidPath(regex) && t_state == 1 && validPath) {
    film->AddSplat(pSplat, path.L);
  }
}


bool Path::isValidPath(const std::regex &pathPattern) const {
  ProfilePhase p(Prof::PathExtractorRegexTest);
  std::string pathExpr = GetPathExpression();
  std::smatch base_match;

  return std::regex_match(pathExpr, base_match, pathPattern);
}


PathVertex PathVertex::FromBDPTVertex(const Vertex &v) {
  PathVertex pv;
  if(v.type == VertexType::Light || v.type == VertexType::Camera) {
    pv.type = v.type == VertexType::Light ? VertexInteraction::Light : VertexInteraction::Camera;
    pv.ei = v.ei;
  } else if (v.type == VertexType::Surface) {
    pv.type = v.delta ? VertexInteraction::Specular : VertexInteraction::Diffuse;
    pv.si = v.si;
  } else {
    LOG(FATAL) << "BDPT Vertex type not supported (Medium interaction) ?";
  }

  return pv;
}

Extractor *CreatePathExtractor(const ParamSet &params, const Point2i fullResolution,
                                 const Float diagonal, const std::string imageFilename) {
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