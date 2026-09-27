"""Calibration intrinsèque de la caméra (mire d'échiquier) — utilisable sans écran sur le Raspberry Pi.

  1) Capture (une image toutes les N secondes, seulement si l'échiquier est bien détecté) :
       ros2 run robot_vision camera_calibration capture --source picamera2 --out ~/calib --count 25
     Pendant la capture, inclinez et déplacez la mire dans tout le champ (bords et coins compris),
     à plusieurs distances. Ne touchez pas à la mise au point.

  2) Calcul :
       ros2 run robot_vision camera_calibration calibrate --images ~/calib --out camera.yaml \\
            --cols 9 --rows 6 --square 0.025

  --cols/--rows = nombre de coins INTÉRIEURS (un échiquier de 10x7 cases a 9x6 coins).
  Erreur de reprojection : < 0.5 px = bon, > 1 px = à refaire.
"""
import argparse
import glob
import os
import sys
import time

import cv2
import numpy as np
import yaml

from .frame_source import open_source

FLAGS = cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE


def cmd_capture(args) -> int:
    out = os.path.expanduser(args.out)
    os.makedirs(out, exist_ok=True)
    src = open_source(args.source, args.device, args.width, args.height, 30.0, args.lens_position)
    saved, last = 0, 0.0
    print(f'Capture de {args.count} images dans {out} (Ctrl+C pour arrêter)...')
    try:
        while saved < args.count:
            frame = src.read()
            if frame is None:
                continue
            found = False
            if time.time() - last >= args.interval:
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                found, _ = cv2.findChessboardCorners(gray, (args.cols, args.rows), FLAGS)
                if found:
                    cv2.imwrite(os.path.join(out, f'img_{saved:03d}.png'), frame)
                    saved += 1
                    last = time.time()
                    print(f'  [{saved}/{args.count}] échiquier détecté -> image enregistrée')
                else:
                    last = time.time() - args.interval + 0.5  # nouvel essai dans 0.5 s
                    print('  échiquier non détecté : mieux éclairé / entièrement visible ?')
            if args.display:
                cv2.imshow('capture (q pour quitter)', frame)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
            else:
                time.sleep(0.05)
    except KeyboardInterrupt:
        pass
    finally:
        src.close()
    print(f'{saved} images enregistrées.')
    return 0


def cmd_calibrate(args) -> int:
    files = sorted(glob.glob(os.path.join(os.path.expanduser(args.images), '*.png'))
                   + glob.glob(os.path.join(os.path.expanduser(args.images), '*.jpg')))
    if len(files) < 10:
        print(f'Seulement {len(files)} images : il en faut au moins 10 (idéalement 20-30).')
        return 1
    objp = np.zeros((args.cols * args.rows, 3), np.float32)
    objp[:, :2] = np.mgrid[0:args.cols, 0:args.rows].T.reshape(-1, 2) * args.square
    obj_pts, img_pts, size = [], [], None
    crit = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 40, 0.001)
    for f in files:
        gray = cv2.cvtColor(cv2.imread(f), cv2.COLOR_BGR2GRAY)
        size = gray.shape[::-1]
        ok, corners = cv2.findChessboardCorners(gray, (args.cols, args.rows), FLAGS)
        if ok:
            corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), crit)
            obj_pts.append(objp)
            img_pts.append(corners)
    print(f'{len(obj_pts)}/{len(files)} images exploitables')
    if len(obj_pts) < 10:
        print('Trop peu d\'images avec échiquier détecté.')
        return 1
    rms, K, dist, _, _ = cv2.calibrateCamera(obj_pts, img_pts, size, None, None)
    print(f'Erreur de reprojection RMS : {rms:.3f} px  ({"bon" if rms < 0.5 else "acceptable" if rms < 1.0 else "MAUVAIS : refaire"})')
    print('Matrice K :\n', K, '\nDistorsion :', dist.ravel())
    data = {
        'image_width': int(size[0]), 'image_height': int(size[1]),
        'camera_matrix': [float(x) for x in K.reshape(-1)],
        'dist_coeffs': [float(x) for x in dist.reshape(-1)],
        'calibrated': True, 'rms_reprojection_px': float(rms),
    }
    with open(args.out, 'w', encoding='utf-8') as fh:
        yaml.safe_dump(data, fh, sort_keys=False)
    print(f'Écrit : {args.out}  (à copier dans robot_vision/config/camera.yaml)')
    return 0


def main(argv=None) -> int:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = p.add_subparsers(dest='cmd', required=True)

    c = sub.add_parser('capture', help='capture automatique d\'images de l\'échiquier')
    c.add_argument('--source', default='picamera2', choices=['picamera2', 'opencv'])
    c.add_argument('--device', default='0')
    c.add_argument('--width', type=int, default=640)
    c.add_argument('--height', type=int, default=480)
    c.add_argument('--lens-position', type=float, default=2.0, dest='lens_position')
    c.add_argument('--out', default='~/calib')
    c.add_argument('--count', type=int, default=25)
    c.add_argument('--interval', type=float, default=2.5)
    c.add_argument('--cols', type=int, default=9)
    c.add_argument('--rows', type=int, default=6)
    c.add_argument('--display', action='store_true')
    c.set_defaults(func=cmd_capture)

    k = sub.add_parser('calibrate', help='calcule la matrice caméra et la distorsion')
    k.add_argument('--images', default='~/calib')
    k.add_argument('--out', default='camera.yaml')
    k.add_argument('--cols', type=int, default=9)
    k.add_argument('--rows', type=int, default=6)
    k.add_argument('--square', type=float, default=0.025, help='côté d\'une case en mètres')
    k.set_defaults(func=cmd_calibrate)

    args = p.parse_args(argv)
    return args.func(args)


if __name__ == '__main__':
    sys.exit(main())
