from typing import Dict, Any
from .ik.hexapodSolver import solveHexapodParams

def get_walk_sequence(
    dimensions: Dict[str, float],
    params: Dict[str, Any] = {
        "tx": 0,
        "tz": 0,
        "rx": 0,
        "ry": 0,
        "legStance": 0,
        "hipStance": 25,
        "stepCount": 5,
        "hipSwing": 25,
        "liftSwing": 40,
    },
    gait_type: str = "tripod",
    walk_mode: str = "walking",
) -> Dict[str, Dict[str, list]]:
    hip_stance = params["hipStance"]
    rx = params["rx"]
    ry = params["ry"]
    tx = params["tx"]
    tz = params["tz"]
    leg_stance = params["legStance"]

    raw_ik_params = {
        "tx": tx,
        "ty": 0,
        "tz": tz,
        "legStance": leg_stance,
        "hipStance": hip_stance,
        "rx": rx,
        "ry": ry,
        "rz": 0,
    }

    ik_solver = solveHexapodParams(dimensions, raw_ik_params, True)[0]

    if not ik_solver["foundSolution"] or ik_solver["hasLegsOffGround"]:
        return None

    hip_swing = params["hipSwing"]
    lift_swing = params["liftSwing"]
    step_count = params["stepCount"]

    a_hip_swing, a_lift_swing = abs(hip_swing), abs(lift_swing)

    hip_swings = get_hip_swing_rotate(a_hip_swing) if walk_mode == "rotating" else get_hip_swing_forward(a_hip_swing)

    if gait_type == "ripple":
        return ripple_sequence(ik_solver["pose"], a_lift_swing, hip_swings, step_count)
    else:
        return tripod_sequence(ik_solver["pose"], a_lift_swing, hip_swings, step_count)


def tripod_sequence(pose, a_lift_swing, hip_swings, step_count) -> Dict[str, Dict[str, list]]:
    forward_alpha_seqs, lift_beta_seqs, lift_gamma_seqs = build_tripod_sequences(
        pose, a_lift_swing, hip_swings, step_count
    )

    double_step_count = 2 * step_count

    tripod_a = tripod_a_sequence(forward_alpha_seqs, lift_gamma_seqs, lift_beta_seqs, double_step_count)
    tripod_b = tripod_b_sequence(forward_alpha_seqs, lift_gamma_seqs, lift_beta_seqs, double_step_count)

    return {**tripod_a, **tripod_b}


def tripod_a_sequence(forward_alpha_seqs, lift_gamma_seqs, lift_beta_seqs, double_step_count) -> Dict[str, Dict[str, list]]:
    sequence = {}
    leg_positions = ["leftFront", "rightMiddle", "leftBack"]

    for leg_position in leg_positions:
        forward = forward_alpha_seqs[leg_position]
        gamma_lift_up = lift_gamma_seqs[leg_position]
        beta_lift_up = lift_beta_seqs[leg_position]

        gamma_seq = gamma_lift_up + gamma_lift_up[::-1] + fill_array(gamma_lift_up[0], double_step_count)
        beta_seq = beta_lift_up + beta_lift_up[::-1] + fill_array(beta_lift_up[0], double_step_count)

        sequence[leg_position] = {
            "alpha": forward + forward[::-1],
            "gamma": gamma_seq,
            "beta": beta_seq,
        }

    return sequence


def tripod_b_sequence(forward_alpha_seqs, lift_gamma_seqs, lift_beta_seqs, double_step_count) -> Dict[str, Dict[str, list]]:
    sequence = {}
    leg_positions = ["rightFront", "leftMiddle", "rightBack"]

    for leg_position in leg_positions:
        forward = forward_alpha_seqs[leg_position]
        gamma_lift_up = lift_gamma_seqs[leg_position]
        beta_lift_up = lift_beta_seqs[leg_position]

        gamma_seq = fill_array(gamma_lift_up[0], double_step_count) + gamma_lift_up + gamma_lift_up[::-1]
        beta_seq = fill_array(beta_lift_up[0], double_step_count) + beta_lift_up + beta_lift_up[::-1]

        sequence[leg_position] = {
            "alpha": forward[::-1] + forward,
            "gamma": gamma_seq,
            "beta": beta_seq,
        }

    return sequence


def ripple_sequence(start_pose, a_lift_swing, hip_swings, step_count) -> Dict[str, Dict[str, list]]:
    sequences = {}
    for position, pose in start_pose.items():
        alpha, beta, gamma = pose["alpha"], pose["beta"], pose["gamma"]

        beta_lift = build_sequence(beta, a_lift_swing, step_count)
        gamma_lift = build_sequence(gamma, -a_lift_swing / 2, step_count)

        delta = hip_swings[position]
        fw1 = build_sequence(alpha - delta, delta, step_count)
        fw2 = build_sequence(alpha, delta, step_count)

        half_delta = delta / 2
        bk1 = build_sequence(alpha + delta, -half_delta, step_count)
        bk2 = build_sequence(alpha + half_delta, -half_delta, step_count)
        bk3 = build_sequence(alpha, -half_delta, step_count)
        bk4 = build_sequence(alpha - half_delta, -half_delta, step_count)

        sequences[position] = build_ripple_leg_sequence(
            position, beta_lift, gamma_lift, fw1, fw2, bk1, bk2, bk3, bk4
        )

    return sequences


def build_ripple_leg_sequence(position, b_lift, g_lift, fw1, fw2, bk1, bk2, bk3, bk4) -> Dict[str, list]:
    step_count = len(fw1)
    rev_g_lift = g_lift[::-1]
    rev_b_lift = b_lift[::-1]
    b0 = b_lift[0]
    g0 = g_lift[0]
    b_n = fill_array(b0, step_count)
    g_n = fill_array(g0, step_count)

    alpha_seq = [fw1, fw2, bk1, bk2, bk3, bk4]
    beta_seq = [b_lift, rev_b_lift, b_n, b_n, b_n, b_n]
    gamma_seq = [g_lift, rev_g_lift, g_n, g_n, g_n, g_n]

    modulo_map = {
        "leftBack": 0,
        "rightFront": 1,
        "leftMiddle": 2,
        "rightBack": 3,
        "leftFront": 4,
        "rightMiddle": 5,
    }

    return {
        "alpha": mod_sequence(modulo_map[position], alpha_seq),
        "beta": mod_sequence(modulo_map[position], beta_seq),
        "gamma": mod_sequence(modulo_map[position], gamma_seq),
    }


def mod_sequence(mod, seq) -> list:
    sequence = seq + seq
    return sequence[mod:mod + 6]


def build_tripod_sequences(start_pose, a_lift_swing, hip_swings, step_count) -> Dict[str, Dict[str, list]]:
    double_step_count = 2 * step_count
    leg_positions = start_pose.keys()

    forward_alpha_seqs = {}
    lift_beta_seqs = {}
    lift_gamma_seqs = {}

    for leg_position in leg_positions:
        alpha, beta, gamma = start_pose[leg_position]
        delta_alpha = hip_swings[leg_position]

        forward_alpha_seqs[leg_position] = build_sequence(alpha - delta_alpha, 2 * delta_alpha, double_step_count)
        lift_beta_seqs[leg_position] = build_sequence(beta, a_lift_swing, step_count)
        lift_gamma_seqs[leg_position] = build_sequence(gamma, -a_lift_swing / 2, step_count)

    return forward_alpha_seqs, lift_beta_seqs, lift_gamma_seqs


def build_sequence(start_val, delta, step_count) -> list:
    step = delta / step_count
    current_item = start_val
    array = []
    for _ in range(step_count):
        current_item += step
        array.append(current_item)
    return array


def get_hip_swing_forward(a_hip_swing) -> Dict[str, float]:
    return {
        "leftFront": -a_hip_swing,
        "rightMiddle": a_hip_swing,
        "leftBack": -a_hip_swing,
        "rightFront": a_hip_swing,
        "leftMiddle": -a_hip_swing,
        "rightBack": a_hip_swing,
    }


def get_hip_swing_rotate(a_hip_swing) -> Dict[str, float]:
    return {
        "leftFront": a_hip_swing,
        "rightMiddle": a_hip_swing,
        "leftBack": a_hip_swing,
        "rightFront": a_hip_swing,
        "leftMiddle": a_hip_swing,
        "rightBack": a_hip_swing,
    }


def fill_array(value, length) -> list:
    if length == 0:
        return []
    return [value] * length
