/**
 * PointKinematics.h
 * ==================
 * Virtual control-point kinematics for a differential-drive robot.
 *
 * MATH (task §3, documented in full — do not simplify):
 *
 * Unicycle model of the robot (body frame: +x forward, +y LEFT, +z up,
 * theta = heading angle of +x_body measured CCW from +x_world):
 *
 *   x_dot = v * cos(theta)
 *   y_dot = v * sin(theta)
 *   theta_dot = omega
 *
 * Virtual control point P at distance l IN FRONT of the axle midpoint:
 *
 *   p_x = x + l*cos(theta)
 *   p_y = y + l*sin(theta)
 *
 * Differentiating (chain rule, theta depends on time):
 *
 *   [p_x_dot]   [ cos(theta)   -l*sin(theta) ] [v]
 *   [p_y_dot] = [ sin(theta)    l*cos(theta) ] [omega]
 *
 * The 2x2 matrix has determinant l (non-singular for l != 0), so the
 * INVERSE mapping from desired point velocity to body twist is exact:
 *
 *   [v]     [ cos(theta)          sin(theta)        ] [p_x_dot]
 *   [omega] [ -sin(theta)/l       cos(theta)/l     ] [p_y_dot]
 *
 * SIGN CONVENTIONS (critical, see README):
 *   - omega > 0 : counter-clockwise turn (nose swings LEFT) when viewed
 *     from above, right-hand rule around +z (up).
 *   - p_y is the LATERAL point velocity in the BODY frame here: we pass
 *     the body-frame point velocity (u_forward, u_lateral) directly, so
 *     the theta-dependent rotation is NOT applied inside this module —
 *     see the two function variants below.
 *
 * Two variants are provided because both are genuinely useful:
 *   1. bodyFrameTwist(): takes the point velocity expressed in the BODY
 *      frame. This is what the line follower uses — the line error is a
 *      body-frame lateral quantity, so no heading estimate is needed.
 *      Derivation: in the body frame the point velocity is
 *      (u_f, u_l) = (v, l*omega), hence
 *          v     = u_f
 *          omega = u_l / l
 *   2. worldFrameTwist(): takes the point velocity in the WORLD frame and
 *      applies the full inverse matrix above; requires a heading estimate
 *      theta (usable later when a reliable IMU yaw exists).
 *
 * UNITS: velocities [m/s], omega [rad/s], l [m], theta [rad].
 */

#ifndef POINT_KINEMATICS_H
#define POINT_KINEMATICS_H

struct Twist2D {
    float v;      ///< linear velocity [m/s], + = forward
    float omega;  ///< angular velocity [rad/s], + = CCW (nose left)
};

struct PointVelocityBody {
    float uForward;   ///< point velocity along +x_body [m/s]
    float uLateral;   ///< point velocity along +y_body (LEFT) [m/s]
};

struct PointVelocityWorld {
    float ux;  ///< world-frame x velocity of the control point [m/s]
    float uy;  ///< world-frame y velocity of the control point [m/s]
};

class PointKinematics {
public:
    explicit PointKinematics(float controlPointDistance)
        : l_(controlPointDistance) {}

    void setControlPointDistance(float l) { l_ = l; }
    float getControlPointDistance() const { return l_; }

    /**
     * Body-frame variant (no heading needed — used by the fast loop).
     * The control point sits l ahead of the axle, rigidly in the body
     * frame, so a body-frame point velocity maps:
     *     v = uForward,  omega = uLateral / l
     * l must be > 0; if l <= 0 the lateral command is ignored (returns
     * omega = 0) rather than dividing by zero.
     */
    Twist2D bodyFrameTwist(const PointVelocityBody& u) const;

    /**
     * World-frame variant (full inverse matrix; needs heading theta).
     *     v     =  cos(theta)*ux + sin(theta)*uy
     *     omega = (-sin(theta)*ux + cos(theta)*uy) / l
     * Kept for future use with a reliable absolute-yaw IMU. The current
     * line follower does NOT use it (no trustworthy theta available).
     */
    Twist2D worldFrameTwist(const PointVelocityWorld& u, float theta) const;

private:
    float l_;  ///< control-point distance [m], > 0
};

#endif // POINT_KINEMATICS_H
