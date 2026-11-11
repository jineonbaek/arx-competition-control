"""Compare rectified observed axial depth with the CAD render of the same camera."""
import numpy as np

# shortcut: initial screening thresholds, not a measured grasp-accuracy guarantee; tune on labelled scene trials.
POLICY=dict(min_pixels=100,min_valid_depth_ratio=.5,min_mask_coverage=.7,
            min_render_precision=.6,max_median_abs_depth_mm=10.,max_p90_abs_depth_mm=25.)

def assess_depth_fit(mask, observed_m, rendered_m, *, max_depth_m=1.5):
    if not np.isfinite(max_depth_m) or max_depth_m<=.07:
        raise ValueError('Invalid camera depth range')
    mask,observed,rendered=map(np.asarray,(mask,observed_m,rendered_m))
    if (observed.ndim!=2 or mask.shape!=observed.shape or rendered.shape!=observed.shape
            or not all(np.isfinite(a).all() for a in (mask,observed,rendered))
            or np.any(observed<0) or np.any(rendered<0)):
        raise ValueError('Invalid rectified mask/depth/render geometry')
    mask=mask>0;silhouette=rendered>0
    valid=(observed>.07)&(observed<max_depth_m)
    overlap=mask&silhouette
    paired=overlap&valid
    count=int(paired.sum());total=int(mask.sum())
    residual=np.abs(observed[paired]-rendered[paired])*1000
    result=dict(version=1,status='insufficient_depth',coordinate_frame='rectified_color_optical',
        depth_unit='m',max_depth_m=float(max_depth_m),mask_pixels=total,paired_pixels=count,
        valid_depth_ratio=float((mask&valid).sum()/max(1,total)),
        mask_coverage=float(overlap.sum()/max(1,total)),
        render_precision=float(overlap.sum()/max(1,int(silhouette.sum()))),
        median_abs_depth_mm=float(np.median(residual)) if count else None,
        p90_abs_depth_mm=float(np.percentile(residual,90)) if count else None,
        policy=dict(POLICY),physical_accuracy_validated=False,reasons=[])
    if count<POLICY['min_pixels'] or result['valid_depth_ratio']<POLICY['min_valid_depth_ratio']:
        result['reasons'].append('CAD 与识别区域内有效深度不足')
        return result
    for metric,threshold in [('mask_coverage','min_mask_coverage'),('render_precision','min_render_precision')]:
        if result[metric]<POLICY[threshold]:result['reasons'].append(metric+' below screening threshold')
    for metric,threshold in [('median_abs_depth_mm','max_median_abs_depth_mm'),('p90_abs_depth_mm','max_p90_abs_depth_mm')]:
        if result[metric]>POLICY[threshold]:result['reasons'].append(metric+' above screening threshold')
    result['status']='rejected' if result['reasons'] else 'consistent'
    return result
