#include "triple_map.h"
#include <stdio.h>
#include <stdint.h>
#include "jc_log.h"

static TripleKey table[MAX_JX] = {0};
static uint8_t g_slave_count = 0;
static int g_jx_map[MAX_JX][6] = {0};
static uint8_t g_id_map[MAX_JX][6] = {0};
static int table_size = 0;

int triple_map_init(const int jx_map[][6], const uint8_t id_map[][6], const uint8_t count)
{
    for (int i = 0; i < count; i++) {
        for (int j = 0; j < 6; j++) {
            int jx = jx_map[i][j];

            g_jx_map[i][j] = jx;
            if (jx >= 0) {
                table[jx].slaveId = i;
                table[jx].passage = j / 3;
                table[jx].id = id_map[i][j];
                g_id_map[i][j] = id_map[i][j];
                JC_LOG_DEBUG("triple_map_init: jx=%d, slaveId=%d, passage=%d, id=0x%02X", jx, table[jx].slaveId, table[jx].passage, table[jx].id);
            }
            table_size++;
        }
    }

    g_slave_count = count;
    return 0;
}

int triple_map_encode(uint8_t slaveId, uint8_t passage, uint16_t id)
{
    for (int i = 0; i < table_size; i++) {
        if (table[i].slaveId == slaveId && table[i].passage == ((passage - 1) / 3) && table[i].id == id) {
            return i;
        }
    }
    return -1;
}

int triple_map_decode(int jx, uint8_t* slaveId, uint8_t* passage, uint16_t* id)
{
    if (jx < 0 || jx >= table_size) {
        return -1;
    }

    for (int _slave_id = 0; _slave_id < g_slave_count; _slave_id++) {
        for (int _passage = 0; _passage < 6; _passage++) {
            if (g_jx_map[_slave_id][_passage] == jx) {
                *slaveId = _slave_id;
                *passage = _passage + 1;
                *id = g_id_map[_slave_id][_passage];

                return 0;
            }
        }
    }

    return -1;
}
