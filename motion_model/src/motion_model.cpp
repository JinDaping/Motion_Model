#include "motion_model/motion_model.h"

MotionModel::MotionModel() {}
MotionModel::~MotionModel() {}

// ackermann model
ACKERMANN_BACK_PARAM MotionModel::ackermann_model_(float velocity,
                                                   float angular) {
  ACKERMANN_BACK_PARAM ackermann_back_param;

  if (velocity > 0.0f) {
    ackermann_back_param->angle =
        atan(angular / velocity * axles_distance); //-PI/2~PI/2  angle
    ackermann_back_param->velocity = velocity;
  } else {
    ackermann_back_param->angle = 0;
    ackermann_back_param->velocity = 0;
  }
  return ackermann_back_param;
}

// differ model
DEFFER_BACK_PARAM MotionModel::differ_model_(float velocity, float angular) {
  DEFFER_BACK_PARAM differ_back_param;

  differ_back_param->wR = velocity / d_wheel_radius;
  differ_back_param->wL = angular / d_wheel_radius * d_axles_distance + wR;

  return differ_back_param;
}

// mecanum model
MECANUM_BACK_PARAM
MotionModel::mecanum_model_(float velocity, float angular,
                            float direction_angle,
                            float angle) { //θ=direction_angle ∅=angle
  MECANUM_BACK_PARAM mecanum_back_param;

  mecanum_back_param->w1 = (-angular * (Length + Width) +
                            velocity * (cos(direction_angle - angle) -
                                        sin(direction_angle - angle))) /
                           m_wheel_radius;
  mecanum_back_param->w2 =
      (angular * (Length + Width) + velocity*(cos(direction_angle - angle) +
                                        sin(direction_angle - angle)) / m_wheel_radius;
  mecanum_back_param->w3 =
      (angular * (Length + Width) + velocity*(cos(direction_angle - angle) -
                                        sin(direction_angle - angle)) / m_wheel_radius;
  mecanum_back_param->w4 =
      (-angular * (Length + Width) + velocity*(cos(direction_angle - angle) +
                                        sin(direction_angle - angle)) / m_wheel_radius;
  return mecanum_back_param;
}

// omni model---3 wheel
OMNI_BACK_PARAM3 MotionModel::omni_model_(float velocity, float angular,
                                          float direction_angle, float angle) {
  OMNI3_BACK_PARAM omni_back_param;
  omni_back_param->w1 =
      (-angular * o_axles_distance + velocity * cos(direction_angle - angle)) /
      o_wheel_radius;
  omni_back_param->w2 =
      (-angular * o_axles_distance +
       velocity * (-0.5 * cos(direction_angle - angle) -
                   sin(PI / 3) * sin(direction_angle - angle))) /
      o_wheel_radius;
  omni_back_param->w3 =
      (-angular * o_axles_distance +
       velocity * (-0.5 * cos(direction_angle - angle) +
                   sin(PI / 3) * sin(direction_angle - angle))) /
      o_wheel_radius;
  return omni_back_param;
}

// omni model---4 wheel
OMNI4_BACK_PARAM MotionModel::omni_model_(float velocity, float angular,
                                          float direction_angle, float angle) {
  OMNI4_BACK_PARAM omni_back_param;
  omni_back_param->w1 =
      (-angular * o_axles_distance + velocity * cos(direction_angle - angle)) /
      o_wheel_radius;
  omni_back_param->w2 =
      (-angular * o_axles_distance - velocity * sin(direction_angle - angle)) /
      o_wheel_radius;
  omni_back_param->w3 =
      (-angular * o_axles_distance - velocity * cos(direction_angle - angle)) /
      o_wheel_radius;
  omni_back_param->w4 =
      (-angular * o_axles_distance + velocity * sin(direction_angle - angle)) /
      o_wheel_radius;
  return omni_back_param;
}