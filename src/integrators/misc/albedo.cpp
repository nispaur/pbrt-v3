//
// Created by stardami on 4/6/17.
//

#include "albedo.h"
#include "interaction.h"
#include "camera.h"
#include "film.h"
#include "paramset.h"
#include "reflection.h"

namespace pbrt {

// AlbedoIntegrator Method Definitions
    Spectrum AlbedoIntegrator::Li(const RayDifferential &ray, const Scene &scene,
                                  Sampler &sampler, MemoryArena &arena,
                                  int depth) const {
        // Find closest ray intersection or return background radiance
        SurfaceInteraction isect;
        if (!scene.Intersect(ray, &isect)) {
            return Spectrum(0.);
        }

        // Compute emitted and reflected light at ray intersection point

        // Initialize common variables for albedo integrator
        Vector3f wo = isect.wo;

        // Compute scattering functions for surface interaction
        isect.ComputeScatteringFunctions(ray, arena);
        if (!isect.bsdf)
            return Spectrum(0.);

        // TODO: Sampling, currently works only for closed-form solutions (lambertian diffuse)
        Point2f samples;
        Spectrum rho = isect.bsdf->rho(wo, 0, &samples, BxDFType(BSDF_REFLECTION | BSDF_DIFFUSE));

        return rho;
    }
    
    AlbedoIntegrator *CreateAlbedoIntegrator(
            const ParamSet &params, std::shared_ptr<Sampler> sampler,
            std::shared_ptr<const Camera> camera) {
        int np;
        const int *pb = params.FindInt("pixelbounds", &np);
        Bounds2i pixelBounds = camera->film->GetSampleBounds();
        if (pb) {
            if (np != 4)
                Error("Expected four values for \"pixelbounds\" parameter. Got %d.",
                      np);
            else {
                pixelBounds = Intersect(pixelBounds,
                                        Bounds2i{{pb[0], pb[2]},
                                                 {pb[1], pb[3]}});
                if (pixelBounds.Area() == 0)
                    Error("Degenerate \"pixelbounds\" specified.");
            }
        }
        return new AlbedoIntegrator(camera, sampler, pixelBounds);
    }
}
