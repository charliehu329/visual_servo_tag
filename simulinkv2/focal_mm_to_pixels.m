function [fxPx,fyPx,calibrationInfo]=focal_mm_to_pixels(fMm,cfg)
%FOCAL_MM_TO_PIXELS Convert optical focal length [mm] to pixels.
fxPx=fMm./cfg.outputPixelPitchXmm;
fyPx=fMm./cfg.outputPixelPitchYmm;
calibrationInfo=struct('method','IMX415_2x2_binning_pitch','isPlaceholder',true, ...
    'pixelPitchXmm',cfg.outputPixelPitchXmm,'pixelPitchYmm',cfg.outputPixelPitchYmm);
end
