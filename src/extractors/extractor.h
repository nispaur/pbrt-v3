//
// Created by nispaur on 4/21/17.
//

#ifndef PBRT_V3_EXTRACTOR_H
#define PBRT_V3_EXTRACTOR_H


#include "geometry.h"
#include "film.h"
#include "pbrt.h"

namespace pbrt {


// Container class

class Container {
  public:
    Container() {};

    virtual void Init(const RayDifferential &r, int depth) =0;
    virtual void ReportData(const SurfaceInteraction &isect) =0;
    virtual void ReportData(const RayDifferential &r) {};
    virtual Spectrum ToRGBSpectrum() const = 0;

    ~Container() {}
};

class NContainer : public Container {
  public:
    NContainer(Point2f pFilm) : p(pFilm) {};

    void Init(const RayDifferential &r, int depth);
    void ReportData(const SurfaceInteraction &isect);
    Spectrum ToRGBSpectrum() const;

  private:
    const Point2f p;

    Normal3f n;
    int depth;
};

// Functor class

class ExtractorFunc {
  public:
    virtual Container *GetNewContainer(Point2f p) const = 0;
};

class NormalExtractor : public ExtractorFunc {
  public:
    Container *GetNewContainer(Point2f p) const;
};


// Extractor main class
class Extractor {
  public:
    Extractor(const ExtractorFunc *f, Film *film) : f(f), film(film) {};

    const ExtractorFunc *f;
    Film *film;
};

}


#endif //PBRT_V3_EXTRACTOR_H
