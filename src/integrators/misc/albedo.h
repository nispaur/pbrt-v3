

#ifndef PBRT_INTEGRATORS_ALBEDO_H
#define PBRT_INTEGRATORS_ALBEDO_H


#include "pbrt.h"
#include "integrator.h"
#include "scene.h"

namespace pbrt {

// AlbedoIntegrator Declarations
    class AlbedoIntegrator : public SamplerIntegrator {
    public:
        // AlbedoIntegrator Public Methods
        AlbedoIntegrator(std::shared_ptr<const Camera> camera,
                          std::shared_ptr<Sampler> sampler,
                          const Bounds2i &pixelBounds)
                : SamplerIntegrator(camera, sampler, pixelBounds) {}
        Spectrum Li(const RayDifferential &ray, const Scene &scene,
                    Sampler &sampler, MemoryArena &arena, int depth) const;
    };

    AlbedoIntegrator *CreateAlbedoIntegrator(
            const ParamSet &params, std::shared_ptr<Sampler> sampler,
            std::shared_ptr<const Camera> camera);

}  // namespace pbrt
#endif //PBRT_INTEGRATORS_ALBEDO_H
