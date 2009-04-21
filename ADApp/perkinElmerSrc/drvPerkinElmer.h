/* drvPerkinElmer.h
 *
 * This is a driver for a simulated area detector.
 *
 * Author: Brian Tieman
 *
 * Created:  07/24/2008
 *
 */

#ifndef DRV_PERKINELMER_H
#define DRV_PERKINELMER_H

#ifdef __cplusplus
extern "C" {
#endif

int PerkinElmerConfig(const char *portName, int maxSizeX, int maxSizeY, int dataType, int maxBuffers, size_t maxMemory,
                                 int priority, int stackSize);

#ifdef __cplusplus
}
#endif
#endif
