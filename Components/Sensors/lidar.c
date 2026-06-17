#include "lidar.h"
#include "vl53l0x.h"
#include "oled.h"
#include "mapping.h"
#include <stdio.h>

extern I2C_HandleTypeDef hi2c3;

static VL53L0X_Dev sensors[4];

static GPIO_TypeDef * const xshut_port[4] = {
    XSHUT1_GPIO_Port, XSHUT2_GPIO_Port, XSHUT3_GPIO_Port, XSHUT4_GPIO_Port
};

static const uint16_t xshut_pin[4] = {
    XSHUT1_Pin, XSHUT2_Pin, XSHUT3_Pin, XSHUT4_Pin
};

#define VL53L0X_DEFAULT_ADDR 0x52

static const uint8_t new_addresses[4] = {
    (LIDAR_ADDR_0 << 1), (LIDAR_ADDR_1 << 1), (LIDAR_ADDR_2 << 1), (LIDAR_ADDR_3 << 1)
};

/* --- FAST DATA SAMPLING ARRAYS --- */
/* Initialized to 8191 (Out of Range) so the robot doesn't phantom-brake on boot! */
static uint16_t s_lidar_hist[4][3] = {
    {8191, 8191, 8191},
    {8191, 8191, 8191},
    {8191, 8191, 8191},
    {8191, 8191, 8191}
};
static uint8_t s_hist_idx[4] = {0};



typedef enum {
    LIDAR_STATE_TRIGGER,
    LIDAR_STATE_WAITING
} LidarState;

static LidarState s_lidar_state = LIDAR_STATE_TRIGGER;


extern uint16_t s_lidar_raw[4];
extern uint8_t  s_lidar_hits[4];
extern int      s_is_moving;
extern Map      g_map;

static uint8_t s_current_sensor_idx = 0;


/* Ultra-fast hardware comparison macros */
#define FAST_MIN(x,y) ((x) < (y) ? (x) : (y))
#define FAST_MAX(x,y) ((x) > (y) ? (x) : (y))
/* -------------------------------------- */

void LIDAR_Init(void)
{
    char buf[32];

    OLED_Clear();
    OLED_Print(0, 0, "Booting LiDAR...");
    OLED_Update();

    for (int i = 0; i < 4; i++) {
        HAL_GPIO_WritePin(xshut_port[i], xshut_pin[i], GPIO_PIN_RESET);
    }
    HAL_Delay(100);

    for (int i = 0; i < 4; i++) {
        snprintf(buf, sizeof(buf), "LiDAR%d...", i+1);
        OLED_Print(0, 14 + (i*10), buf);
        OLED_Update();

        HAL_GPIO_WritePin(xshut_port[i], xshut_pin[i], GPIO_PIN_SET);
        HAL_Delay(50);

        sensors[i].i2c = &hi2c3;
        sensors[i].address = VL53L0X_DEFAULT_ADDR;
        sensors[i].timeout_ms = 100;

        VL53L0X_SetAddress(&sensors[i], new_addresses[i]);
        HAL_Delay(20);

        if (VL53L0X_Init(&sensors[i], &hi2c3, new_addresses[i]) == 1) {
            char addr_buf[16];
            snprintf(addr_buf, sizeof(addr_buf), "0x%02X", (new_addresses[i] >> 1));
            OLED_Print(80, 14 + (i*10), addr_buf);
        } else {
            OLED_Print(80, 14 + (i*10), "FAIL");
        }
        OLED_Update();
    }

    HAL_Delay(2000);
}

/* ═══════════════════════════════════════════════════════════════ */
/* NEW: AUTOMATED HOT-REBOOT RECOVERY                              */
/* ═══════════════════════════════════════════════════════════════ */
static void LIDAR_RecoverSensor(uint8_t index)
{
    /* 1. Hard kill the sensor's power using XSHUT */
    HAL_GPIO_WritePin(xshut_port[index], xshut_pin[index], GPIO_PIN_RESET);
    HAL_Delay(5); /* Tiny delay to let capacitors drain */

    /* 2. Wake it back up */
    HAL_GPIO_WritePin(xshut_port[index], xshut_pin[index], GPIO_PIN_SET);
    HAL_Delay(10); /* Let the sensor boot up */

    /* 3. The sensor forgot its address! Point the library back to factory default */
    sensors[index].address = VL53L0X_DEFAULT_ADDR;

    /* 4. Reprogram the custom address */
    VL53L0X_SetAddress(&sensors[index], new_addresses[index]);
    HAL_Delay(5);

    /* 5. Re-initialize the base settings */
    VL53L0X_Init(&sensors[index], &hi2c3, new_addresses[index]);
}

/* ═══════════════════════════════════════════════════════════════ */
/* Reads the hardware, filters the noise, and handles crashes      */
/* ═══════════════════════════════════════════════════════════════ */
void Process_LiDAR_Asynchronous(void)
{
    VL53L0X_Dev *current_sensor = &sensors[s_current_sensor_idx];

    if (s_lidar_state == LIDAR_STATE_TRIGGER)
    {
        /* Fire the laser and immediately step out */
        VL53L0X_StartSingleShot(current_sensor);
        s_lidar_state = LIDAR_STATE_WAITING;
    }
    else if (s_lidar_state == LIDAR_STATE_WAITING)
    {
        /* Check if it's done without halting the CPU */
        if (VL53L0X_CheckDataReady(current_sensor))
        {
            uint16_t raw_val = VL53L0X_GetDistanceResult(current_sensor);

            /* 1. FAULT DETECTION: Did motor noise cause a timeout crash? */
            if (raw_val == 65535)
            {
                LIDAR_RecoverSensor(s_current_sensor_idx);
                /* Fallback to last known safe sample */
                raw_val = s_lidar_hist[s_current_sensor_idx][(s_hist_idx[s_current_sensor_idx] + 2) % 3];
            }

            /* 2. SAMPLING: Push to the moving median array */
            s_lidar_hist[s_current_sensor_idx][s_hist_idx[s_current_sensor_idx]] = raw_val;
            s_hist_idx[s_current_sensor_idx] = (s_hist_idx[s_current_sensor_idx] + 1) % 3;

            /* 3. Extract median window to eliminate random single spikes */
            uint16_t a = s_lidar_hist[s_current_sensor_idx][0];
            uint16_t b = s_lidar_hist[s_current_sensor_idx][1];
            uint16_t c = s_lidar_hist[s_current_sensor_idx][2];
            uint16_t filtered_dist = FAST_MAX(FAST_MIN(a,b), FAST_MIN(FAST_MAX(a,b),c));

            /* Save clean distance for OLED / Broadcast variables */
            s_lidar_raw[s_current_sensor_idx] = filtered_dist;

            /* 4. DEBOUNCE FILTER: Verify if obstacle is a real wall vs reflection ghosting */
            if (filtered_dist > 40 && filtered_dist <= 250)
            {
                if (s_lidar_hits[s_current_sensor_idx] < 2) {
                    s_lidar_hits[s_current_sensor_idx]++;
                }
            } else {
                s_lidar_hits[s_current_sensor_idx] = 0;
            }

            /* 5. DYNAMIC GRID MAPPING */
            if (s_is_moving && filtered_dist > 40 && filtered_dist < 3500)
            {
                float angle = 0.0f;
                if(s_current_sensor_idx == 0) angle = 0.0f;   /* Front */
                if(s_current_sensor_idx == 1) angle = 90.0f;  /* Right */
                if(s_current_sensor_idx == 2) angle = 180.0f; /* Back  */
                if(s_current_sensor_idx == 3) angle = 270.0f; /* Left  */

                Map_UpdateLiDAR(&g_map, filtered_dist / 1000.0f, angle);
            }

            /* Move to next sensor array channel and reset step state machine */
            s_current_sensor_idx = (s_current_sensor_idx + 1) % 4;
            s_lidar_state = LIDAR_STATE_TRIGGER;
        }
    }
}
