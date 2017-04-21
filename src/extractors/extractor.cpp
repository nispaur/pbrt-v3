//
// Created by nispaur on 4/21/17.
//

#include <core/interaction.h>
#include "extractors/extractor.h"

namespace pbrt {


void NContainer::Init(const RayDifferential &r, int depth) {
    this->depth = depth;
}

void NContainer::ReportData(const SurfaceInteraction &isect) {
    if(depth == 0)
        n = isect.shading.n;
}

Spectrum NContainer::ToRGBSpectrum() const {
    Float rgb[3] = {n.x,n.y,n.z};
    return RGBSpectrum::FromRGB(rgb);
}


Container *NormalExtractor::GetNewContainer(Point2f p) const {
    return new NContainer(p);
}
}