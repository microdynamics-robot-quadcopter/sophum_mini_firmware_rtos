#ifndef __SOPHUM_DRIVER_LOG_H__
#define __SOPHUM_DRIVER_LOG_H__

#include <stdio.h>

#ifndef SUCCESS
#define SUCCESS 1
#endif

#ifndef ERROR
#define ERROR 0
#endif

#ifndef SOPHUM_LOG_ENABLE
#define SOPHUM_LOG_ENABLE
#endif

#ifdef SOPHUM_LOG_ENABLE
#define SOPHUM_LOG(operation, state) printf("info:[%s] line:[%d] func:[%s] operation:[%s]\n", \
((state == 1)? "SUCCESS": "ERROR"), __LINE__, __FUNCTION__, operation)

#else
#define SOPHUM_LOG(operation, state)

#endif

#endif