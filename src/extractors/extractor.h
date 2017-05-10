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
    virtual Spectrum ToSample() const = 0;

    virtual ~Container() {}
};

// Functor class

class ExtractorFunc {
  public:
    virtual std::shared_ptr<Container> GetNewContainer(Point2f p) const = 0;
};

// Extractor main class
class Extractor {
  public:
    Extractor(const ExtractorFunc *f, Film *film) : f(f), film(film) {};

    const ExtractorFunc *f;
    Film *film;
};

class Containers {
  public:
    Containers() {};

    inline void Add(std::shared_ptr<Container> container) {
      containers.push_back(container);
    }

    void Init(const RayDifferential &r, int depth, const Scene &scene);

    template <typename T>
    void ReportData(const T &value) {
      for(std::shared_ptr<Container> c : containers)
        c->ReportData(value);
    }

    Spectrum ToSample(int id) const {
      CHECK_LT(id, containers.size());
      return containers[id]->ToSample();
    }
  private:
    std::vector<std::shared_ptr<Container>> containers;
};

// Extractor Manager

class ExtractorTileManager {
  public:

    void Add(std::unique_ptr<FilmTile> tile) {
      filmtiles.push_back(std::move(tile));
    }

    void AddSamples(const Point2f &pFilm, std::unique_ptr<Containers> container, Float sampleWeight = 1.f);

    std::unique_ptr<FilmTile> GetTile(int id) {
      CHECK_LT(id, filmtiles.size());
      return std::move(filmtiles[id]);
    }
  private:
    std::vector<std::unique_ptr<FilmTile>> filmtiles;
};

class ExtractorManager {
  public:
    ExtractorManager() {};

    void Add(Extractor *extractor) {
      extractors.push_back(extractor);
    }

    std::unique_ptr<Containers> GetNewContainer(Point2f p);
    std::unique_ptr<ExtractorTileManager> GetNewExtractorTile(const Bounds2i &sampleBounds);
    void MergeTiles(std::unique_ptr<ExtractorTileManager> tiles);
    void WriteOutput();

    private:
      std::vector<Extractor*> extractors;
};

// Albedo Extractor

class AlbedoContainer : public Container {
  public:
    AlbedoContainer(Point2f pFilm, const BxDFType &t, bool integrate, int nbSamples) :
            p(pFilm), bxdftype(t), integrate(integrate), nSamples(nbSamples) {};

    void Init(const RayDifferential &r, int depth, const Scene &Scene);
    void ReportData(const SurfaceInteraction &isect);
    Spectrum ToSample() const {
      return rho;
    }

  private:
    const Point2f p;
    const BxDFType bxdftype;
    const bool integrate;
    const int nSamples;
    Spectrum rho;
    int depth;
    std::vector<Point2f> wi;
    std::vector<Point2f> wo;
};


class AlbedoExtractor : public ExtractorFunc {
  public:
    AlbedoExtractor(const BxDFType &type, bool integrate, int nbSamples) :
            type(type), integrateAlbedo(integrate), nbSamples(nbSamples) {}

    std::shared_ptr<Container> GetNewContainer(Point2f p) const {
      return std::shared_ptr<Container>(new AlbedoContainer(p, type, integrateAlbedo, nbSamples));
    }

  private:
    const BxDFType type;
    const bool integrateAlbedo; // Defines if the albedo should be in closed form or sampled
    const int nbSamples;
};

// Normal extractor

class NContainer : public Container {
  public:
    NContainer(Point2f pFilm) : p(pFilm) {};

    void Init(const RayDifferential &r, int depth, const Scene &scene);
    void ReportData(const SurfaceInteraction &isect);
    Spectrum ToSample() const;

  private:
    const Point2f p;

    Normal3f n;
    int depth;
};

class NormalExtractor : public ExtractorFunc {
  public:
    std::shared_ptr<Container> GetNewContainer(Point2f p) const;
};

// Depth Extractor

class ZContainer : public Container {
  public:
    ZContainer(Point2f pFilm) : p(pFilm) {};

    void Init(const RayDifferential &r, int depth, const Scene &scene);
    void ReportData(const SurfaceInteraction &isect);
    Spectrum ToSample() const;

  private:
    const Point2f p;
    Float zfar;
    Float znear;
    Float distance;
    int depth;
};

class ZExtractor : public ExtractorFunc {
  public:
    std::shared_ptr<Container> GetNewContainer(Point2f p) const {
      return std::shared_ptr<Container>(new ZContainer(p));
    }
};


// API Methods

Extractor *CreateNormalExtractor(const ParamSet &params, const Point2i fullResolution,
                                 const Float diagonal, const std::string imageFilename);
Extractor *CreateZExtractor(const ParamSet &params, const Point2i fullResolution,
                            const Float diagonal, const std::string imageFilename);
Extractor *CreateAlbedoExtractor(const ParamSet &params, const Point2i fullResolution,
                                 const Float diagonal, const std::string imageFilename);

}


#endif //PBRT_V3_EXTRACTOR_H
