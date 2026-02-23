/**
 * FPGA-based Out-of-Tree Controller
 */

#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>

#include "app.h"
#include "FreeRTOS.h"
#include "task.h"

#define DEBUG_MODULE "FPGA_CTRL"
#include "debug.h"

#include "controller.h"
#include "controller_pid.h"
#include "log.h"
#include "param.h"
#include "deck.h"
#include "sleepus.h"
#include "math3d.h"
#include "param_logic.h"

//--------------------------------------------------------------
// Configuration
//--------------------------------------------------------------

// Hardware pins
#define FPGA_CS_PIN       DECK_GPIO_IO1
#define FPGA_IRQ_PIN      DECK_GPIO_IO2
#define FPGA_SPI_BAUDRATE SPI_BAUDRATE_12MHZ

/* 4-byte header + 12 state words × 4 bytes */
#define TX_LEN  (4 + 12 * 4)
/* 4 control outputs × 4 bytes */
#define RX_LEN  (4 * 4)


#define FRAC_BITS 22
#define SCALE     (1 << FRAC_BITS)

/* 32-bit signed range for Q10.22: full int32 range (~ -1024 to +1024 in value) */
#define FP32_MIN  INT32_MIN
#define FP32_MAX  INT32_MAX

static bool fpgaControllerInitialized = false;
static uint8_t emptyBuffer[TX_LEN] = {0};
static uint8_t txBuffer[TX_LEN] = {0};
static uint8_t dummyBuffer[RX_LEN] = {0};

static uint64_t runTimes = 0;

void appMain() {
    DEBUG_PRINT("FPGA Controller app started.\n");
    
    while (1) {
        vTaskDelay(M2T(20000));
    }
}

//--------------------------------------------------------------
// Out-of-Tree Controller Interface
//--------------------------------------------------------------
void controllerOutOfTreeInit(void) {
    if(fpgaControllerInitialized) {
        DEBUG_PRINT("FPGA out-of-tree controller already initialized, skipping.\n");
        return;
    }
    
    DEBUG_PRINT("FPGA out-of-tree controller init...\n");
    
    pinMode(FPGA_CS_PIN, OUTPUT);
    pinMode(FPGA_IRQ_PIN, INPUT_PULLDOWN);
    digitalWrite(FPGA_CS_PIN, HIGH);
    spiBegin();
    spiBeginTransaction(FPGA_SPI_BAUDRATE);

    DEBUG_PRINT("FPGA out-of-tree controller initialized.\n");

    vTaskDelay(M2T(2000)); // Wait for FPGA to be ready

    DEBUG_PRINT("Sending dummy first transaction.\n");
    
    digitalWrite(FPGA_CS_PIN, LOW);

    uint8_t localTxBuffer[TX_LEN] = {0}; localTxBuffer[3] = 0xAA; 
    uint8_t localRxBuffer[TX_LEN];

    spiExchange(TX_LEN, localTxBuffer, localRxBuffer);
    DEBUG_PRINT("Dummy transaction sent.\n");

    controllerPidInit();

    fpgaControllerInitialized = true;   
}

bool controllerOutOfTreeTest(void) {
    return true;
}

static inline quaternion_t normalize_quat(quaternion_t q)
{
    float inv_mag = 1.0f / sqrtf(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w);
    return (quaternion_t){
        .x = q.x * inv_mag,
        .y = q.y * inv_mag,
        .z = q.z * inv_mag,
        .w = q.w * inv_mag
    };
}

static inline struct vec quat_2_rp(quaternion_t q)
{
  struct vec v;
  v.x = q.x / q.w;
  v.y = q.y / q.w;
  v.z = q.z / q.w;
  return v;
}

void float_to_32bit_fixed_at(float value, uint8_t *buf, size_t offset)
{
    double scaled = (double)value * (double)SCALE;
    int32_t fp32;

    if (scaled <= (double)FP32_MIN) {
        fp32 = FP32_MIN;
    } else if (scaled >= (double)FP32_MAX) {
        fp32 = FP32_MAX;
    } else {
        fp32 = (int32_t)(scaled < 0 ? scaled - 0.5 : scaled + 0.5);
    }

    uint32_t u = (uint32_t)fp32;
    buf[offset + 0] = (uint8_t)(u >> 24);
    buf[offset + 1] = (uint8_t)(u >> 16);
    buf[offset + 2] = (uint8_t)(u >>  8);
    buf[offset + 3] = (uint8_t)(u >>  0);
}

float fixed_32bit_to_float_at(const uint8_t *buf, size_t offset)
{
    /* 32-bit little-endian */
    uint32_t u32 = ((uint32_t)buf[offset]     << 24)
                 | ((uint32_t)buf[offset + 1] << 16)
                 | ((uint32_t)buf[offset + 2] <<  8)
                 | ((uint32_t)buf[offset + 3] <<  0);

    int32_t s32 = (int32_t)u32;
    return (float)s32 * (1.0f / (float)SCALE);
}

void stateToTxBuffer(const setpoint_t *setpoint, const state_t *state, const sensorData_t *sensors, uint8_t *buffer) {

    // ToDo difference from setpoint

    struct vec phi = quat_2_rp(normalize_quat(state->attitudeQuaternion)); // quaternion to Rodrigues parameters
    // DEBUG_PRINT("phi: (%.2f, %.2f, %.2f)\n", (double)phi.x, (double)phi.y, (double)phi.z);

    buffer[0] = 0x00;
    buffer[1] = 0x00;
    buffer[2] = 0x00;
    buffer[3] = 0xAA;

    
// x=-0.036 y=-0.203 z=0.499
    float_to_32bit_fixed_at(state->position.x - setpoint->position.x, buffer, 4);
    float_to_32bit_fixed_at(state->position.y - setpoint->position.y, buffer, 8);
    float_to_32bit_fixed_at(state->position.z - setpoint->position.z, buffer, 12); // ToDo  tmp hover at 1m height
    float_to_32bit_fixed_at(phi.x, buffer, 16);
    float_to_32bit_fixed_at(phi.y, buffer, 20);
    float_to_32bit_fixed_at(phi.z, buffer, 24);
    float_to_32bit_fixed_at(state->velocity.x, buffer, 28);
    float_to_32bit_fixed_at(state->velocity.y, buffer, 32);
    float_to_32bit_fixed_at(state->velocity.z, buffer, 36);
    float_to_32bit_fixed_at(radians(sensors->gyro.x), buffer, 40);
    float_to_32bit_fixed_at(radians(sensors->gyro.y), buffer, 44);
    float_to_32bit_fixed_at(radians(sensors->gyro.z), buffer, 48);
}

void rxBufferToControl(const uint8_t *buffer, control_t *control) {
    control->controlMode = controlModeForce;
    control->normalizedForces[0] = 0.58333335f + fixed_32bit_to_float_at(buffer, 0);
    control->normalizedForces[1] = 0.58333335f + fixed_32bit_to_float_at(buffer, 4);
    control->normalizedForces[2] = 0.58333335f + fixed_32bit_to_float_at(buffer, 8);
    control->normalizedForces[3] = 0.58333335f + fixed_32bit_to_float_at(buffer, 12);
}

void controllerOutOfTree(control_t *control,
                         const setpoint_t *setpoint,
                         const sensorData_t *sensors,
                         const state_t *state,
                         const uint32_t tick) {
    if(!fpgaControllerInitialized) {
        return;
    }

    if(!RATE_DO_EXECUTE(RATE_500_HZ, tick))
    {
        return;
    }

    runTimes++;

    stateToTxBuffer(setpoint, state, sensors, txBuffer);
  
    // uint64_t start = usecTimestamp();

    for(int i = 0; i < 50000; i++) {
        if(digitalRead(FPGA_IRQ_PIN) == HIGH) {
            break;
        }
    }
    uint8_t rxBuffer[RX_LEN] = {0};


    for(int i = 0; i < 1000; i++) {
        spiExchange(1, emptyBuffer, rxBuffer);
        if(rxBuffer[0] == 0xFF) 
        {
            break;    
        }
    }
    /* Consume the remaining 3 bytes of the 32-bit ready word (0xFF 0xFF 0xFF 0xFF) */
    spiExchange(3, emptyBuffer, dummyBuffer);

    spiExchange(RX_LEN, emptyBuffer, rxBuffer);

    digitalWrite(FPGA_CS_PIN, HIGH);
    sleepus(1);
    digitalWrite(FPGA_CS_PIN, LOW);

    spiExchange(TX_LEN, txBuffer, dummyBuffer);

    // uint64_t end = usecTimestamp();

    rxBufferToControl(rxBuffer, control);

    if(runTimes < 500) controllerPid(control, setpoint, sensors, state, tick);

    
    // if(runTimes < 200) {
    //     DEBUG_PRINT("TIME: %lld us\n", end - start);
    //     DEBUG_PRINT("Received control (run %llu): ", runTimes);
    //     for(int i = 0; i < (RX_LEN / 4); i++) {
    //         DEBUG_PRINT("  Force[%d] = %.4f %02X %02X %02X %02X \n", i,(double)fixed_32bit_to_float_at(rxBuffer, 4*i) , rxBuffer[4*i], rxBuffer[4*i+1], rxBuffer[4*i+2], rxBuffer[4*i+3]);
    //     }
    // }
}

//--------------------------------------------------------------
// Logging Variables
//--------------------------------------------------------------

// LOG_GROUP_START(fpga)
// LOG_ADD(LOG_UINT8, init, &fpgaInitialized)
// LOG_GROUP_STOP(fpga)
//--------------------------------------------------------------
// Parameters
//--------------------------------------------------------------

// PARAM_GROUP_START(fpga)
// PARAM_GROUP_STOP(fpga)
