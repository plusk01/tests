#ifndef TEST_H
#define TEST_H

#ifdef __cplusplus
extern "C" {
#endif

void inc_global(void);
int read_global(void);

void inc_static(void);
int read_static(void);

extern int my_global;

#ifdef __cplusplus
}
#endif

#endif