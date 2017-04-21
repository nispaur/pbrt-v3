//
// Created by nispaur on 4/21/17.
//

#ifndef PBRT_V3_EXTRACTOR_H
#define PBRT_V3_EXTRACTOR_H


#include "reflection.h"
#include "geometry.h"
#include "film.h"
#include "pbrt.h"

namespace pbrt {


// Container class

class Container {
  public:
    Container() {};

    virtual void Init(const RayDifferential &r, int depth, const Scene &scene) =0;
    virtual void ReportData(const SurfaceInteraction &isect) =0;
    virtual void ReportData(const RayDifferential &r) {};
    virtual Spectrum ToRGBSpectrum() const = 0;

    ~Container() {}
};

class DummyContainer : public Container {
  public:
    DummyContainer(Point2f pFilm) {};

    void Init(const RayDifferential &r, int depth, const Scene &scene) {};
    void ReportData(const SurfaceInteraction &isect) {};
    Spectrum ToRGBSpectrum() const { return Spectrum(0.f); }
};

class NContainer : public Container {
  public:
    NContainer(Point2f pFilm) : p(pFilm) {};

    void Init(const RayDifferential &r, int depth, const Scene &scene);
    void ReportData(const SurfaceInteraction &isect);
    Spectrum ToRGBSpectrum() const;

  private:
    const Point2f p;

    Normal3f n;
    int depth;
};

class ZContainer : public Container {
  public:
    ZContainer(Point2f pFilm) : p(pFilm) {};

    void Init(const RayDifferential &r, int depth, const Scene &scene);
    void ReportData(const SurfaceInteraction &isect);
    Spectrum ToRGBSpectrum() const;

  private:
    const Point2f p;
    Float zfar;
    Float znear;
    Float distance;
    int depth;
};


class AlbedoContainer : public Container {
  public:
    AlbedoContainer(Point2f pFilm) : p(pFilm) {};

    void Init(const RayDifferential &r, int depth, const Scene &Scene);
    void ReportData(const SurfaceInteraction &isect);
    Spectrum ToRGBSpectrum() const { return rho; }

  private:
    const Point2f p;
    Spectrum rho;
    int depth;
};

// Functor class

class ExtractorFunc {
  public:
    virtual Container *GetNewContainer(Point2f p) const = 0;
};

class DummyExtractor : public ExtractorFunc {
  public:
    Container *GetNewContainer(Point2f p) { return new DummyContainer(p); }
};

class NormalExtractor : public ExtractorFunc {
  public:
    Container *GetNewContainer(Point2f p) const;
};

class ZExtractor : public ExtractorFunc {
  public:
    Container *GetNewContainer(Point2f p) const {
      return new ZContainer(p);
    }
};

class AlbedoExtractor : public ExtractorFunc {
  public:
    Container *GetNewContainer(Point2f p) const {
      return new AlbedoContainer(p);
    }
};

// Extractor main class
class Extractor {
  public:
    Extractor(const ExtractorFunc *f, Film *film) : f(f), film(film) {};

    const ExtractorFunc *f;
    Film *film;
};

// API Methods

Extractor *CreateNormalExtractor(const ParamSet &params, const Film *imagefilm);
Extractor *CreateZExtractor(const ParamSet &params, const Film *imagefilm);
Extractor *CreateAlbedoExtractor(const ParamSet &params, const Film *imagefilm);

}


#endif //PBRT_V3_EXTRACTOR_H
