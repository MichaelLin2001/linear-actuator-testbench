#ifndef TRIPLE_MAP_H
#define TRIPLE_MAP_H

#ifdef __cplusplus
extern "C" {
#endif
#include <stdint.h>

#define MAX_JX 64

typedef struct {
    uint8_t slaveId;
    uint8_t passage;
    uint16_t id;
} TripleKey;

int triple_map_init(const int jx_map[][6], const uint8_t id_map[][6], const uint8_t count);
int triple_map_encode(uint8_t slaveId, uint8_t passage, uint16_t id);
int triple_map_decode(int jx, uint8_t* slaveId, uint8_t* passage, uint16_t* id);

#ifdef __cplusplus
}
#endif

#endif  // TRIPLE_MAP_H
