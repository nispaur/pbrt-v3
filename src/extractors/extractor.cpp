//
// Created by nispaur on 4/21/17.
//

#include "filters/box.h"
#include "scene.h"
#include "paramset.h"
#include "interaction.h"
#include "extractors/extractor.h"
#include "spectrum.h"

namespace pbrt {


void NContainer::Init(const RayDifferential &r, int depth, const Scene &scene) {
    this->depth = depth;
}

void NContainer::ReportData(const SurfaceInteraction &isect) {
    if(depth == 0) {
      n = Faceforward(isect.n, isect.wo);
    }
}

Spectrum NContainer::ToRGBSpectrum() const {
    Float rgb[3] = {(n.x*0.5f)+0.5f, (n.y*.5f)+0.5f, (n.z*.5f)+.5f};
    return RGBSpectrum::FromRGB(rgb);
}


Container *NormalExtractor::GetNewContainer(Point2f p) const {
    return new NContainer(p);
}

void ZContainer::Init(const RayDifferential &r, int depth, const Scene &scene) {
    zfar = scene.WorldBound().pMax.z;
    znear = scene.WorldBound().pMin.z;
    this->depth = depth;
}

void ZContainer::ReportData(const SurfaceInteraction &isect) {
    if(depth == 0) {
        distance = (isect.p.z-znear)/(zfar-znear);
    }
}

Spectrum ZContainer::ToRGBSpectrum() const {
    return Spectrum(distance);
}


void AlbedoContainer::ReportData(const SurfaceInteraction &isect) {
  Point2f samples;
  if(depth == 0 && isect.bsdf)
    rho = isect.bsdf->rho(isect.wo, 0, &samples, BxDFType(BSDF_REFLECTION | BSDF_DIFFUSE));
}

void AlbedoContainer::Init(const RayDifferential &r, int depth, const Scene &Scene) {
  this->depth = depth;
}

Extractor *CreateNormalExtractor(const ParamSet &params, const Film *imagefilm) {
  std::string filename = params.FindOneString("outputfile", "");
  if (filename == "") filename = "normal.exr";

  return new Extractor(new NormalExtractor(), new Film(
          imagefilm->fullResolution,
          Bounds2f(Point2f(0, 0), Point2f(1, 1)),
          std::unique_ptr<Filter>(CreateBoxFilter(ParamSet())),
          imagefilm->diagonal, filename, 1.f));
}

Extractor *CreateZExtractor(const ParamSet &params, const Film *imagefilm) {
  std::string filename = params.FindOneString("outputfile", "");
  if (filename == "") filename = "depth.exr";

  return new Extractor(new ZExtractor(), new Film(
          imagefilm->fullResolution,
          Bounds2f(Point2f(0, 0), Point2f(1, 1)),
          std::unique_ptr<Filter>(CreateBoxFilter(ParamSet())),
          imagefilm->diagonal, filename, 1.f));
}


Extractor *CreateAlbedoExtractor(const ParamSet &params, const Film *imagefilm) {
  std::string filename = params.FindOneString("outputfile", "");
  if (filename == "") filename = "albedo.exr";

  return new Extractor(new AlbedoExtractor(), new Film(
          imagefilm->fullResolution,
          Bounds2f(Point2f(0, 0), Point2f(1, 1)),
          std::unique_ptr<Filter>(CreateBoxFilter(ParamSet())),
          imagefilm->diagonal, filename, 1.f));
}

}