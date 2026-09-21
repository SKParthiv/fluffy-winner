/**
 * PointKinematics.cpp — see the header for the full derivation.
 * The mathematics is intentionally NOT simplified: the world-frame
 * inverse matrix is written exactly as documented.
 */

#include "PointKinematics.h"
#include <cmath>

Twist2D PointKinematics::bodyFrameTwist(const PointVelocityBody& u) const {
    Twist2D t;
    t.v = u.uForward;
    // A non-positive control-point distance is a configuration error;
    // fail safe (no rotation) instead of dividing by zero/negative.
    t.omega = (l_ > 0.0f) ? (u.uLateral / l_) : 0.0f;
    return t;
}

Twist2D PointKinematics::worldFrameTwist(const PointVelocityWorld& u,
                                         float theta) const {
    Twist2D t;
    const float c = cosf(theta);
    const float s = sinf(theta);
    // Inverse of  [[c, -l*s], [s, l*c]]  is  [[c, s], [-s/l, c/l]]
    t.v     = c * u.ux + s * u.uy;
    t.omega = (l_ > 0.0f) ? ((-s * u.ux + c * u.uy) / l_) : 0.0f;
    return t;
}
