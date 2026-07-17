function fMm=focal_pixels_to_mm(fPx,cfg)
%FOCAL_PIXELS_TO_MM Inverse using the configured equivalent output pitch.
pitch=[cfg.outputPixelPitchXmm;cfg.outputPixelPitchYmm];
fMm=fPx.*pitch;
end
