#ifndef MOTION_MODEL_H
#define MOTION_MODEL_H

#include <math.h>
#define PI 3.141592654

// ackermann param
#define axles_distance 1.305 //单位m
#define wheel_distance 0.905
#define velocity_size 1.8
#define angle_size 1.0

#define ctrl_velocity_size 200.0
#define ctrl_angle_size 106.0

// differ robot param
#define d_axles_distance 0.3
#define d_wheel_radius 0.1
// mecanum param
#define Lenght 0.3
#define Width 0.1
#define m_wheel_radius 0.1
// omni model
#define o3_axles_distance 0.30
#define o3_wheel_radius 0.05
// omni model
#define o4_axles_distance 0.30
#define o4_wheel_radius 0.05

// ackermann model
typedef struct {
  float velocity;
  float angle;
} ACKERMANN_BACK_PARAM;
// differ model
typedef struct {
  float wL;
  float wR;
} DIFFER_BACK_PARAM;
// mecanum model
typedef struct {
  float w1;
  float w2;
  float w3;
  float w4;
} MECANUM_BACK_PARAM;
// omni model
typedef struct {
  float w1;
  float w2;
  float w3;
} OMNI3_BACK_PARAM;
typedef struct {
  float w1;
  float w2;
  float w3;
  float w4;
} OMNI4_BACK_PARAM;
class MotionModel {
public:
  MotionModel();
  ~MotionModel();
  ACKERMANN_BACK_PARAM MotionModel::ackermann_model_(float velocity,
                                                     float angular);
  DEFFER_BACK_PARAM MotionModel::differ_model_(float velocity, float angular);
  MECANUM_BACK_PARAM
  MotionModel::mecanum_model_(float velocity, float angular,
                              float direction_angle, float angle);
  OMNI_BACK_PARAM3 MotionModel::omni_model_(float velocity, float angular,
                                            float direction_angle, float angle);
  OMNI4_BACK_PARAM MotionModel::omni_model_(float velocity, float angular,
                                            float direction_angle, float angle);

private:
};

#endif
