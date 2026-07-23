from pxr import Gf
import math

def solve_two_bone_ik(shoulder_pos, target_pos, u1_rest, u2_rest, pole_hint):
    """Return (R1, R2) as Gf.Rotation: R1 = shoulder world rotation,
    R2 = elbow LOCAL rotation (applied in the frame after R1)."""
    L1 = u1_rest.GetLength()
    L2 = u2_rest.GetLength()
    d_vec = target_pos - shoulder_pos
    d = d_vec.GetLength()
    d = max(abs(L1 - L2) + 1e-6, min(L1 + L2 - 1e-6, d))
    d_dir = d_vec.GetNormalized()

    cos_alpha = (L1 * L1 + d * d - L2 * L2) / (2 * L1 * d)
    cos_alpha = max(-1.0, min(1.0, cos_alpha))
    alpha = math.degrees(math.acos(cos_alpha))

    pole_dir = (pole_hint - shoulder_pos).GetNormalized()
    pole_perp = (pole_dir - d_dir * Gf.Dot(pole_dir, d_dir))
    if pole_perp.GetLength() < 1e-8:
        pole_perp = Gf.Vec3d(0, 0, 1) - d_dir * Gf.Dot(Gf.Vec3d(0, 0, 1), d_dir)
    pole_perp = pole_perp.GetNormalized()
    bend_axis = Gf.Cross(d_dir, pole_perp).GetNormalized()

    e_dir = Gf.Rotation(bend_axis, alpha).TransformDir(d_dir).GetNormalized()

    u1_hat = u1_rest.GetNormalized()
    u2_hat = u2_rest.GetNormalized()

    R1 = Gf.Rotation(u1_hat, e_dir)

    E = shoulder_pos + e_dir * L1
    f_dir = (target_pos - E).GetNormalized()
    # R2 must be expressed in the SHOULDER's local frame (it composes as
    # R1 applied AFTER R2 when walking down the hierarchy: world = R1(R2(u2))),
    # so the target direction has to be un-rotated by R1 first.
    target_dir_local = R1.GetInverse().TransformDir(f_dir)
    R2 = Gf.Rotation(u2_hat, target_dir_local)
    return R1, R2, E
