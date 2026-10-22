#!/usr/bin/env python3
"""Build a YOLO "bear" training set from frames saved with save_frames.py (offline utility).

Run with the export venv (needs ultralytics):

  1) Auto-label (run again after each new flight; only NEW runs are labelled, runs you
     already reviewed are skipped): pretrained YOLO already boxes the bear (it just calls it "surfboard",
     "fire hydrant", ...). Every box is saved as class 0 = bear, and a preview image is written
     for you to check.
       ~/yolo_export_venv/bin/python bear_dataset.py prelabel

     Runs whose folder name contains "nobear" get empty labels (no YOLO run).

  2) Review: open ~/grizzlyguard_frames/_review and DELETE every preview that is wrong:
       - a box that is not on a bear (tree, shadow, ...)
       - a bear with no box, or a box that only covers part of the bear
     Keep previews that are right (including correct "NO BEAR" ones).

  3) Build: only frames whose preview you kept are used. Whole runs go to validation.
       ~/yolo_export_venv/bin/python bear_dataset.py build --val-runs run05 run06_nobear

Output: ~/grizzlyguard_dataset/{images,labels}/{train,val}/ and bear.yaml for `yolo train`.
Frames and dataset stay outside the Git repository.
"""
import argparse
import shutil
import sys
from pathlib import Path

import cv2

HOME = Path.home()
REVIEW = '_review'


def runs_in(frames: Path):
    return sorted(p for p in frames.iterdir() if p.is_dir() and not p.name.startswith('_'))


def images_in(run: Path):
    return sorted(p for p in run.iterdir() if p.suffix.lower() in ('.png', '.jpg', '.jpeg'))


def prelabel(a):
    from ultralytics import YOLO

    frames = Path(a.frames).expanduser()
    review = frames / REVIEW
    review.mkdir(parents=True, exist_ok=True)
    chosen = set(a.runs or [])
    all_runs = runs_in(frames)
    unknown = chosen - {r.name for r in all_runs}
    if unknown:
        sys.exit(f'--runs not found: {sorted(unknown)}')
    model = YOLO(a.model)

    totals = {'frames': 0, 'boxes': 0, 'no_box': 0}
    for run in runs_in(frames):
        imgs = images_in(run)
        if not imgs:
            continue
        labels = run / 'labels'
        if chosen and run.name not in chosen:
            continue
        if labels.exists() and not chosen:
            print(f'{run.name}: already labelled, skipping (your review is kept)')
            continue
        # (Re)label this run: clear only its own labels and previews.
        shutil.rmtree(labels, ignore_errors=True)
        for old in review.glob(f'{run.name}__*'):
            old.unlink()
        labels.mkdir()
        no_bear_run = 'nobear' in run.name.lower()
        print(f'{run.name}: {len(imgs)} frames' + ('  (no-bear run: empty labels)' if no_bear_run else ''))
        for img_path in imgs:
            img = cv2.imread(str(img_path))
            if img is None:
                continue
            h, w = img.shape[:2]
            boxes = []
            if not no_bear_run:
                r = model.predict(img, conf=a.conf, agnostic_nms=True, verbose=False)[0]
                for (x1, y1, x2, y2), c, s in zip(r.boxes.xyxy.tolist(), r.boxes.cls.tolist(),
                                                  r.boxes.conf.tolist()):
                    if (x2 - x1) * (y2 - y1) > a.max_area * w * h:
                        continue   # whole-frame boxes are background, not a bear
                    boxes.append((x1, y1, x2, y2, r.names[int(c)], s))
            with open(labels / f'{img_path.stem}.txt', 'w') as f:
                for x1, y1, x2, y2, _, _ in boxes:
                    f.write(f'0 {(x1 + x2) / 2 / w:.6f} {(y1 + y2) / 2 / h:.6f} '
                            f'{(x2 - x1) / w:.6f} {(y2 - y1) / h:.6f}\n')

            vis = img.copy()
            for x1, y1, x2, y2, name, s in boxes:
                cv2.rectangle(vis, (int(x1), int(y1)), (int(x2), int(y2)), (0, 0, 255), 2)
                cv2.putText(vis, f'bear? ({name} {s:.2f})', (int(x1), max(14, int(y1) - 5)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 255), 1, cv2.LINE_AA)
            if not boxes:
                cv2.putText(vis, 'NO BEAR', (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.9,
                            (0, 255, 255), 2, cv2.LINE_AA)
                totals['no_box'] += 1
            cv2.imwrite(str(review / f'{run.name}__{img_path.stem}.jpg'), vis)
            totals['frames'] += 1
            totals['boxes'] += len(boxes)

    print(f"\n{totals['frames']} frames, {totals['boxes']} boxes, {totals['no_box']} with no box.")
    print(f'Now review the previews in {review} and delete every wrong one.')


def build(a):
    frames = Path(a.frames).expanduser()
    out = Path(a.out).expanduser()
    review = frames / REVIEW
    if not review.is_dir():
        sys.exit('Run "prelabel" and review the previews first.')
    run_names = {r.name for r in runs_in(frames)}
    unknown = set(a.val_runs) - run_names
    if unknown:
        sys.exit(f'--val-runs not found: {sorted(unknown)}. Runs are: {sorted(run_names)}')
    if out.exists():
        shutil.rmtree(out)

    counts = {s: {'images': 0, 'boxes': 0, 'empty': 0, 'runs': set()} for s in ('train', 'val')}
    rejected = 0
    for run in runs_in(frames):
        split = 'val' if run.name in a.val_runs else 'train'
        for img_path in images_in(run):
            label = run / 'labels' / f'{img_path.stem}.txt'
            if not label.exists():
                continue
            if not (review / f'{run.name}__{img_path.stem}.jpg').exists():
                rejected += 1          # preview deleted during review -> wrong label
                continue
            for kind, src in (('images', img_path), ('labels', label)):
                dst = out / kind / split
                dst.mkdir(parents=True, exist_ok=True)
                shutil.copy2(src, dst / f'{run.name}__{src.name}')
            n = sum(1 for line in open(label) if line.strip())
            c = counts[split]
            c['images'] += 1
            c['boxes'] += n
            c['empty'] += n == 0
            c['runs'].add(run.name)

    for s, c in counts.items():
        print(f"{s:5}: {c['images']} images ({c['empty']} without bears), {c['boxes']} bear boxes, "
              f"runs: {', '.join(sorted(c['runs'])) or '-'}")
    print(f'Skipped {rejected} frames you rejected in review.')
    if counts['train']['images'] == 0 or counts['val']['images'] == 0:
        sys.exit('Need images in both train and val. Check --val-runs.')

    yaml_path = out / 'bear.yaml'
    yaml_path.write_text(f'path: {out}\ntrain: images/train\nval: images/val\nnames:\n  0: bear\n')
    print(f'\nDataset ready: {yaml_path}')


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    sub = ap.add_subparsers(dest='cmd', required=True)
    p = sub.add_parser('prelabel', help='auto-label frames and write review previews')
    p.add_argument('--frames', default=str(HOME / 'grizzlyguard_frames'))
    p.add_argument('--model', default='yolo26n.pt')
    p.add_argument('--conf', type=float, default=0.15, help='min score of the pretrained guess')
    p.add_argument('--max-area', type=float, default=0.6, help='ignore boxes bigger than this frame fraction')
    p.add_argument('--runs', nargs='+', help='only (re)label these runs; resets their review')
    p.set_defaults(func=prelabel)
    b = sub.add_parser('build', help='make train/val dataset from reviewed frames')
    b.add_argument('--frames', default=str(HOME / 'grizzlyguard_frames'))
    b.add_argument('--out', default=str(HOME / 'grizzlyguard_dataset'))
    b.add_argument('--val-runs', nargs='+', required=True, help='run folders used only for validation')
    b.set_defaults(func=build)
    a = ap.parse_args()
    a.func(a)


if __name__ == '__main__':
    main()