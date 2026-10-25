#ifndef CANRPC_ARM_SAFETY_H
#define CANRPC_ARM_SAFETY_H
#include <math.h>
#include <stdint.h>

typedef struct {
  float position_tolerance_rad[3];
  float settle_velocity_rad_s,rebase_max_delta_rad;
  uint32_t settle_duration_ms;
} canrpc_arm_motion_config_t;
#define CANRPC_ARM_MOTION_DEFAULT_CONFIG {{0.05f,0.01f,0.01f},0.05f,0.2f,200u}
typedef struct { uint32_t started_ms,counts[3]; uint8_t active; } canrpc_arm_settle_t;

/* Preserve native calibration without adding mechanical limits. */
static inline int canrpc_arm_config_valid(const canrpc_arm_motion_config_t *c) {
  if(!c || !isfinite(c->settle_velocity_rad_s) || c->settle_velocity_rad_s<=0 ||
     !isfinite(c->rebase_max_delta_rad) || c->rebase_max_delta_rad<=0 ||
     !c->settle_duration_ms || c->settle_duration_ms>=0x80000000u) return 0;
  for(unsigned i=0;i<3u;++i) if(!isfinite(c->position_tolerance_rad[i]) || c->position_tolerance_rad[i]<=0) return 0;
  return 1;
}
static inline int canrpc_arm_joint_safe(const canrpc_arm_motion_config_t *c,const float q[3],uint8_t *axis) {
  (void)c;
  if(axis) *axis=255u;
  for(unsigned i=0;i<3u;++i) if(!isfinite(q[i])) {if(axis) *axis=(uint8_t)i;return 11;}
  return 0;
}
static inline float canrpc_arm_joint_to_native(const canrpc_arm_motion_config_t *c,unsigned axis,float q) {
  (void)c;
  return axis==1u ? -q-1.0f : (axis==2u ? q-1.884f:q);
}
static inline float canrpc_arm_native_to_joint(const canrpc_arm_motion_config_t *c,unsigned axis,float q) {
  (void)c;
  return axis==1u ? -(q+1.0f) : (axis==2u ? q+1.884f:q);
}
static inline int canrpc_arm_continuous(const canrpc_arm_motion_config_t *c,const float previous[3],const float next[3],uint8_t *axis) {
  int result=canrpc_arm_joint_safe(c,previous,axis);
  if(result) return result;
  result=canrpc_arm_joint_safe(c,next,axis);
  if(result) return result;
  for(unsigned i=0;i<3u;++i) if(!isfinite(c->rebase_max_delta_rad) || c->rebase_max_delta_rad<=0 || fabsf(next[i]-previous[i])>c->rebase_max_delta_rad) {
    if(axis) *axis=(uint8_t)i;
    return 15;
  }
  return 0;
}
static inline int canrpc_arm_at_goal(const canrpc_arm_motion_config_t *c,const float q[3]) {
  const float goal[3]={0,0,2.0f};
  for(unsigned i=0;i<3u;++i) if(!isfinite(q[i]) || fabsf(q[i]-goal[i])>c->position_tolerance_rad[i]) return 0;
  return 1;
}
static inline int canrpc_arm_stationary(const canrpc_arm_motion_config_t *c,const float velocity[3]) {
  for(unsigned i=0;i<3u;++i) if(!isfinite(velocity[i]) || fabsf(velocity[i])>c->settle_velocity_rad_s) return 0;
  return 1;
}
static inline int canrpc_arm_settled(const canrpc_arm_motion_config_t *c,canrpc_arm_settle_t *s,const float q[3],const float velocity[3],const uint32_t counts[3],uint32_t now) {
  if(!canrpc_arm_at_goal(c,q) || !canrpc_arm_stationary(c,velocity)) {s->active=0;return 0;}
  for(unsigned i=0;i<3u;++i) if(!counts[i] || counts[i]==s->counts[i]) return 0;
  for(unsigned i=0;i<3u;++i) s->counts[i]=counts[i];
  if(!s->active) {s->active=1;s->started_ms=now;}
  return (uint32_t)(now-s->started_ms)>=c->settle_duration_ms;
}
#endif
