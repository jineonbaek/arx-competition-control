"""Saved-frame CNOS reranking only. No robot, camera, HTTP, or actuator imports."""
import argparse
import hashlib
import json
import os
from pathlib import Path
from types import SimpleNamespace
import sys
import time
import subprocess

os.environ['PYOPENGL_PLATFORM'] = 'egl'
import numpy as np
from PIL import Image, ImageDraw
import torch

HERE = Path(__file__).resolve().parent
RUNS = Path('/home/arx/arx-competition-control/agent/runs')
CASES = [
    ('head_2025', 'run-20261114T122512Z-47ca13b0', 'fixed_0', [501, 0, 621, 104]),
    ('head_2027', 'run-20261114T122715Z-fc8c74c9', 'fixed_0', [501, 0, 621, 104]),
    ('head_2030', 'run-20261114T123009Z-d385a2e4', 'fixed_0', [501, 0, 621, 104]),
    ('head_203035', 'run-20261114T123035Z-73c29386', 'fixed_0', [501, 0, 621, 104]),
    ('head_2031', 'run-20261114T123130Z-22907676', 'fixed_0', [501, 0, 621, 104]),
    ('wrist_2032', 'run-20261114T123130Z-22907676', 'fixed_3', [278, 30, 411, 161]),
    ('wrist_203252', 'run-20261114T123130Z-22907676', 'fixed_7', [242, 54, 445, 344]),
    ('head_2151_absent', 'run-20261114T135124Z-0a11945f', 'fixed_0', None),
]


def write_json(path, value):
    path.write_text(json.dumps(value, indent=2, ensure_ascii=False) + '\n')


def overlap_label(box, truth):
    if truth is None:
        return 'non_battery'
    a, b = np.array(box), np.array(truth)
    intersection = np.prod(np.maximum(0, np.minimum(a[2:], b[2:]) - np.maximum(a[:2], b[:2])))
    area, target_area = np.prod(a[2:] - a[:2]), np.prod(b[2:] - b[:2])
    # Offline annotation convention only; never an online acceptance criterion.
    return 'battery_visible_part' if intersection / max(1, area) >= .65 and area <= 1.8 * target_area else 'non_battery_or_mixed'


def prepare_masks(output):
    from transformers import Sam2Model, Sam2Processor
    root = Path('/home/arx/arx-vision-demo/vision/demo/models/sam2')
    processor = Sam2Processor.from_pretrained(root, local_files_only=True)
    model = Sam2Model.from_pretrained(root, local_files_only=True).cuda().eval()
    manifest = []
    with torch.inference_mode():
        for name, run, call, truth in CASES:
            source = RUNS / run / 'battery-observations' / call
            report = json.loads((source / 'recognition.json').read_text())
            image = Image.open(source / 'rgb.png').convert('RGB')
            scene = output / name
            scene.mkdir(parents=True, exist_ok=True)
            image.save(scene / 'rgb.png')
            candidates = [dict(row, source='saved_dino') for row in report['candidates']]
            if truth is not None:
                candidates.append(dict(candidate_id=-3, bbox_xyxy=truth, score=None, source='manual_battery_probe'))
            if name.startswith('head_'):
                candidates += [dict(candidate_id=-1, bbox_xyxy=[0, 320, 180, 480], score=None, source='manual_gripper_negative'),
                               dict(candidate_id=-2, bbox_xyxy=[530, 300, 640, 480], score=None, source='manual_gripper_negative')]
            rows, masks = [], []
            for row in candidates:
                box = row['bbox_xyxy']
                inputs = processor(images=image.convert('L').convert('RGB'), input_boxes=[[box]], return_tensors='pt').to('cuda')
                result = model(**inputs)
                decoded = processor.post_process_masks(result.pred_masks.cpu(), inputs['original_sizes'].cpu())[0]
                index = int(result.iou_scores.reshape(-1).argmax())
                mask = decoded.reshape(-1, image.height, image.width)[index].numpy().astype(bool)
                if not mask.any():
                    row['skipped'] = 'empty_segmentation'
                    continue
                masks.append(mask)
                rows.append(dict(row, annotation=overlap_label(box, truth), sam_score=float(result.iou_scores.reshape(-1)[index])))
            np.savez_compressed(scene / 'masks.npz', masks=np.asarray(masks))
            write_json(scene / 'candidates.json', rows)
            manifest.append(dict(name=name, run=run, call=call, truth_box=truth,
                                 original_selected=report.get('selection'), source_rgb_sha256=hashlib.sha256((source/'rgb.png').read_bytes()).hexdigest()))
            print('MASKS', name, len(rows), flush=True)
    write_json(output / 'manifest.json', manifest)


def render_templates(upstream, mesh_path, output):
    import pyrender
    import trimesh
    from src.poses.pyrender import render
    poses = np.load(upstream / 'src/poses/predefined_poses/obj_poses_level0.npy').copy()
    poses[:, :3, 3] *= .0004
    mesh = trimesh.load_mesh(mesh_path)
    groups = {'full': mesh, 'end_half': mesh.slice_plane([0, 0, mesh.bounds[1, 2] / 2], [0, 0, 1])}
    for name, part in groups.items():
        destination = output / name
        destination.mkdir(parents=True, exist_ok=True)
        if len(list(destination.glob('*.png'))) == len(poses):
            continue
        part = part.copy()
        part.apply_translation(-part.bounds.mean(axis=0))
        part.apply_scale(.001)
        part.visual.face_colors = np.tile([102, 102, 102, 255], (len(part.faces), 1))
        render(pyrender.Mesh.from_trimesh(part, smooth=False), str(destination), poses,
               (480, 640), np.array([[572.4114, 0, 325.2611], [0, 573.57043, 242.04899], [0, 0, 1]]),
               light_itensity=1.0)
        print('RENDER', name, len(poses), flush=True)
    write_json(output / 'mesh.json', dict(path=str(mesh_path), sha256=hashlib.sha256(mesh_path.read_bytes()).hexdigest(),
                                       bounds_mm=mesh.bounds.tolist(), views=42, material='uniform gray',
                                       end_half='CAD clipped at object Z >= max_Z/2; synthetic partial-object ablation, not a drone occlusion simulation'))


def evaluate(args):
    from src.model.dinov2 import CustomDINOv2
    from src.model.detector import CNOS
    from src.model.loss import PairwiseSimilarity
    backbone = torch.hub.load(str(args.dino_source), 'dinov2_vitl14', source='local', pretrained=False)
    backbone.load_state_dict(torch.load(args.weights, map_location='cpu', weights_only=True))
    backbone = backbone.cuda().eval()
    descriptor = CustomDINOv2('dinov2_vitl14', backbone, 'x_norm_clstoken', 224, 8, 640).cuda().eval()
    matcher = CNOS(None, descriptor, SimpleNamespace(),
                   SimpleNamespace(metric=PairwiseSimilarity(), aggregation_function='avg_5', confidence_thresh=-float('inf'), max_num_instances=10000),
                   SimpleNamespace(), 5, str(args.output / 'matcher_logs'), False)
    reference = []
    reference_names = ['full', 'end_half']
    with torch.inference_mode():
        for name in reference_names:
            features = []
            for path in sorted((args.output / 'templates' / name).glob('*.png')):
                image = Image.open(path).convert('RGBA')
                mask = np.asarray(image)[:, :, 3] > 0
                y, x = np.nonzero(mask)
                boxes = torch.tensor([[x.min(), y.min(), x.max()+1, y.max()+1]], device='cuda')
                rgb = torch.tensor(np.array(image.convert('RGB')) / 255, device='cuda', dtype=torch.float32).permute(2, 0, 1)[None]
                crop = descriptor.rgb_proposal_processor(rgb, boxes)
                mean = torch.tensor([.485, .456, .406], device='cuda')[None, :, None, None]
                std = torch.tensor([.229, .224, .225], device='cuda')[None, :, None, None]
                features.append(descriptor.compute_features((crop - mean) / std, 'x_norm_clstoken'))
            reference.append(torch.cat(features))
        matcher.ref_data = {'descriptors': torch.stack(reference)}
        torch.save(matcher.ref_data['descriptors'].cpu(), args.output / 'reference_features.pt')
        scenes = []
        for case in json.loads((args.output / 'manifest.json').read_text()):
            scene = args.output / case['name']
            rows = json.loads((scene / 'candidates.json').read_text())
            masks = np.load(scene / 'masks.npz')['masks']
            image = Image.open(scene / 'rgb.png').convert('RGB')
            boxes = [row['bbox_xyxy'] for row in rows]
            proposals = SimpleNamespace(masks=torch.tensor(masks, device='cuda'), boxes=torch.tensor(boxes, device='cuda'))
            torch.cuda.synchronize(); start = time.perf_counter()
            features = descriptor(np.array(image), proposals)
            _, _, scores, distribution = matcher.find_matched_proposals(features)
            torch.cuda.synchronize(); elapsed = time.perf_counter() - start
            # Verify every proposal survived; no confidence threshold or NMS is applied.
            assert len(scores) == len(rows)
            expected = matcher.matching_config.metric(features, matcher.ref_data['descriptors']).topk(5, dim=-1).values.mean(-1)
            torch.testing.assert_close(distribution, expected)
            values = distribution.cpu().tolist()
            for row, value in zip(rows, values):
                row.update(cnos_full=value[0], cnos_end_half=value[1], cnos_combined=max(value))
            original = [r for r in rows if r['source'] == 'saved_dino']
            rankings = {key: sorted(original, key=lambda r:r[key], reverse=True) for key in ['cnos_full','cnos_end_half','cnos_combined']}
            result = dict(case, candidates=rows, rankings={key:[r['candidate_id'] for r in ranking] for key, ranking in rankings.items()},
                          top_labels={key:ranking[0]['annotation'] for key,ranking in rankings.items()},
                          battery_in_original_candidates=any(r['annotation']=='battery_visible_part' for r in original),
                          warm_matching_seconds=elapsed)
            write_json(scene / 'scores.json', result)
            scene_rows = sorted(rows, key=lambda r:r['cnos_combined'], reverse=True)
            sheet = Image.new('RGB', (900, 155 * ((len(rows)+2)//3)), 'white')
            draw = ImageDraw.Draw(sheet)
            for i, row in enumerate(scene_rows):
                x, y = (i % 3)*300, (i // 3)*155
                crop = image.crop(row['bbox_xyxy']); crop.thumbnail((140, 105))
                sheet.paste(crop, (x, y))
                draw.text((x, y+107), f"ID {row['candidate_id']} {row['annotation']}", fill='black')
                draw.text((x, y+122), f"full {row['cnos_full']:.3f} end {row['cnos_end_half']:.3f}", fill='black')
                draw.text((x, y+137), str(row['source']), fill='black')
            sheet.save(scene / 'ranking.png')
            scenes.append(result)
            print('RESULT',case['name'],result['top_labels'],round(elapsed,3),flush=True)
    write_json(args.output / 'results.json', dict(backbone='dinov2_vitl14', upstream_commit=subprocess.check_output(['git', '-C', str(args.upstream), 'rev-parse', 'HEAD'], text=True).strip(),
              weights_sha256=hashlib.sha256(args.weights.read_bytes()).hexdigest(), scenes=scenes,
              proposal_source='saved Grounding-DINO boxes + fresh local SAM2 masks; not original CNOS SAM proposal generator',
              no_acceptance_threshold=True, no_motion=True))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--upstream', type=Path, required=True)
    parser.add_argument('--dino-source', type=Path)
    parser.add_argument('--weights', type=Path)
    parser.add_argument('--mesh', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--prepare-only', action='store_true')
    args = parser.parse_args()
    sys.path.insert(0, str(args.upstream))
    args.output.mkdir(parents=True, exist_ok=True)
    torch.set_num_threads(4)
    if not (args.output / 'manifest.json').exists():
        prepare_masks(args.output)
    render_templates(args.upstream, args.mesh, args.output / 'templates')
    if not args.prepare_only:
        evaluate(args)


if __name__ == '__main__':
    main()
