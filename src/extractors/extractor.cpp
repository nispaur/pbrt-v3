//
// Created by nispaur on 4/21/17.
//

#include "interaction.h"
#include "extractors/extractor.h"
#include "spectrum.h"

namespace pbrt {


void NContainer::Init(const RayDifferential &r, int depth) {
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
}