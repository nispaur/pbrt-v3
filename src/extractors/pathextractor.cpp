//
// Created by stardami on 5/3/17.
//

#include "extractors/pathextractor.h"
#include <regex>

namespace pbrt {

void PathExtractorContainer::Init(const RayDifferential &r, int depth, const Scene &Scene) {

}

void PathExtractorContainer::ReportData(Vertex *cameraVertices, int t) {
  BuildPath(nullptr, cameraVertices, 0, t);
}

void PathExtractorContainer::ReportData(Vertex *mainVertices, Vertex lastVertex,
                                        int nvertices, VertexInteraction mainPath) {
  if(mainPath == VertexInteraction::Light) {
    BuildPath(mainVertices, &lastVertex, nvertices, 1);
  } else {
    BuildPath(&lastVertex, mainVertices, 1, nvertices);
  }
}

void PathExtractorContainer::ReportData(Vertex *lightVertices, Vertex *cameraVertices, int s, int t) {
  BuildPath(lightVertices, cameraVertices, s, t);
}

void PathExtractorContainer::BuildPath(Vertex *lightVertices, Vertex *cameraVertices, int s, int t) {
  // Light->Camera order
  // Light to camera vertices
  for (int i = 0; i < s; ++i) {
    path.vertices.push_back(PathVertex::FromBDPTVertex(lightVertices[i]));
  }
  // Camera to light vertices, must be added in reverse order
  for (int i = t - 1; i >= 0; --i) {
    path.vertices.push_back(PathVertex::FromBDPTVertex(cameraVertices[i]));
  }
}

bool PathExtractorContainer::isValidPath(const std::regex &pathPattern) const {
  ProfilePhase p(Prof::PathExtractorRegexTest);
  std::string pathExpr = path.GetPathExpression();
  std::smatch base_match;

  return std::regex_match(pathExpr, base_match, pathPattern);
}


PathVertex PathVertex::FromBDPTVertex(Vertex v) {
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

}