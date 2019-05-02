#pragma once

#include <stdint.h>
#include <sys/time.h>
#include <pthread.h>

#define NUM_PWM 8

struct msg_t
{
  uint32_t id;
  float pwm[NUM_PWM];
  struct timeval t;

  pthread_mutex_t mutex;
  pthread_cond_t  condvar;
};