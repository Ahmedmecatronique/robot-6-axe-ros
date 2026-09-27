#!/usr/bin/env python3
"""Génère firmware/Core/Inc/robot_config_generated.h à partir de robot_description/config/robot.yaml.

robot.yaml est la SOURCE UNIQUE de vérité (longueurs, limites, réducteurs, PID...) : après l'avoir
modifié, relancer ce script puis recompiler le firmware.

    python tools/gen_firmware_config.py
"""
import math
import os
import sys

import yaml

ROOT = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
YAML_PATH = os.path.join(ROOT, 'ros2_ws', 'src', 'robot_description', 'config', 'robot.yaml')
OUT_PATH = os.path.join(ROOT, 'firmware', 'Core', 'Inc', 'robot_config_generated.h')


def farr(values):
    return '{ ' + ', '.join(f'{v:.9g}f' for v in values) + ' }'


def iarr(values):
    return '{ ' + ', '.join(str(int(v)) for v in values) + ' }'


def main() -> int:
    with open(YAML_PATH, 'r', encoding='utf-8') as f:
        cfg = yaml.safe_load(f)
    rb, hw = cfg['robot'], cfg['hardware']
    lim = rb['limits']
    n = len(rb['joint_names'])
    assert n == 6, 'le firmware suppose 6 axes'
    for key in ('gear_ratio', 'encoder_ppr', 'encoder_sign', 'dir_sign', 'home_dir', 'home_speed_deg_s'):
        assert len(hw[key]) == n, f'hardware.{key} doit contenir {n} valeurs'
    assert sorted(hw['home_order']) == list(range(n)), 'home_order doit être une permutation de 0..5'

    steps_rev = hw['motor_full_steps_per_rev'] * hw['microsteps']
    two_pi = 2.0 * math.pi
    steps_per_rad = [steps_rev * g / two_pi for g in hw['gear_ratio']]
    counts_per_rad = [p * 4 * g / two_pi for p, g in zip(hw['encoder_ppr'], hw['gear_ratio'])]
    q_min = [math.radians(v) for v in lim['position_min_deg']]
    q_max = [math.radians(v) for v in lim['position_max_deg']]
    v_max = [math.radians(v) for v in lim['velocity_max_deg_s']]
    a_max = [math.radians(v) for v in lim['acceleration_max_deg_s2']]
    home_pos = [lo if d < 0 else hi for lo, hi, d in zip(q_min, q_max, hw['home_dir'])]
    home_speed = [math.radians(v) for v in hw['home_speed_deg_s']]
    pid = hw['pid']

    # Contrôle de cohérence : la vitesse max articulaire doit pouvoir être produite par le générateur de pas
    for i in range(n):
        rate = v_max[i] * steps_per_rad[i]
        if rate > hw['max_step_rate_hz']:
            print(f'ATTENTION axe {i + 1}: vitesse max {lim["velocity_max_deg_s"][i]}°/s demande {rate:.0f} pas/s '
                  f'> max_step_rate_hz ({hw["max_step_rate_hz"]}) -> vitesse plafonnée par le firmware',
                  file=sys.stderr)

    lines = [
        '/* FICHIER GÉNÉRÉ par tools/gen_firmware_config.py à partir de robot.yaml — NE PAS MODIFIER À LA MAIN. */',
        '#ifndef ROBOT_CONFIG_GENERATED_H',
        '#define ROBOT_CONFIG_GENERATED_H',
        '',
        '#include <stdint.h>',
        '',
        f'#define NUM_AXES 6',
        f'#define CFG_MAX_STEP_RATE_HZ {float(hw["max_step_rate_hz"]):.1f}f',
        f'#define CFG_WATCHDOG_MS {int(hw["watchdog_ms"])}',
        f'#define CFG_DEADBAND_RAD {math.radians(pid["deadband_deg"]):.9g}f',
        f'#define CFG_FERR_MAX_RAD {math.radians(pid["following_error_max_deg"]):.9g}f',
        '',
        '/* pas moteur (avec micro-pas) par radian ARTICULAIRE, réducteur inclus */',
        f'static const float CFG_STEPS_PER_RAD[NUM_AXES] = {farr(steps_per_rad)};',
        '/* incréments encodeur (x4) par radian ARTICULAIRE ; l\'encodeur est sur l\'arbre moteur */',
        f'static const float CFG_ENC_COUNTS_PER_RAD[NUM_AXES] = {farr(counts_per_rad)};',
        f'static const int8_t CFG_ENC_SIGN[NUM_AXES] = {iarr(hw["encoder_sign"])};',
        f'static const int8_t CFG_DIR_SIGN[NUM_AXES] = {iarr(hw["dir_sign"])};',
        '',
        '/* limites articulaires (rad), vitesses (rad/s), accélérations (rad/s^2) */',
        f'static const float CFG_Q_MIN[NUM_AXES] = {farr(q_min)};',
        f'static const float CFG_Q_MAX[NUM_AXES] = {farr(q_max)};',
        f'static const float CFG_V_MAX[NUM_AXES] = {farr(v_max)};',
        f'static const float CFG_A_MAX[NUM_AXES] = {farr(a_max)};',
        '',
        '/* homing : sens de recherche, vitesse, angle au fin de course, ordre */',
        f'static const int8_t CFG_HOME_DIR[NUM_AXES] = {iarr(hw["home_dir"])};',
        f'static const float CFG_HOME_SPEED[NUM_AXES] = {farr(home_speed)};',
        f'static const float CFG_HOME_POS[NUM_AXES] = {farr(home_pos)};',
        f'static const uint8_t CFG_HOME_ORDER[NUM_AXES] = {iarr(hw["home_order"])};',
        '',
        '/* PID de position (rad -> rad/s) */',
        f'static const float CFG_KP[NUM_AXES] = {farr(pid["kp"])};',
        f'static const float CFG_KI[NUM_AXES] = {farr(pid["ki"])};',
        f'static const float CFG_KD[NUM_AXES] = {farr(pid["kd"])};',
        '',
        '#endif /* ROBOT_CONFIG_GENERATED_H */',
        '',
    ]
    with open(OUT_PATH, 'w', encoding='utf-8', newline='\n') as f:
        f.write('\n'.join(lines))
    print(f'Écrit : {OUT_PATH}')
    for i in range(n):
        print(f'  axe {i + 1}: {steps_per_rad[i]:9.1f} pas/rad, {counts_per_rad[i]:9.1f} inc/rad, '
              f'vitesse max -> {v_max[i] * steps_per_rad[i]:6.0f} pas/s')
    return 0


if __name__ == '__main__':
    sys.exit(main())
