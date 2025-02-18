/**
 * @file l9963_platform.h
 * @brief L9963 platform dependent isolation header
 *
 * @author Antonio Capone
 * @date 2025-02-17
 * @version 1.0
 */

#ifndef __L9963_STATUS_H_
#define __L9963_STATUS_H_

typedef enum {
	L9963_OK,
	L9963_NOT_OK,
	L9963_TIMEOUT
} L9963_Status;

#endif /* __L9963_STATUS_H_ */