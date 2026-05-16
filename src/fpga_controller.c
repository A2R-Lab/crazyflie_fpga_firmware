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

/* Temporary firmware-only test mode:
 * - Keep ROS/commander command decoding active.
 * - Skip FPGA SPI transactions.
 * - Clamp the PID x setpoint between received obstacle bounds.
 */
#define FPGA_BYPASS_TO_PID_TEST 0
#define CONSTRAINT_LOG_PERIOD_MS 1000

/* 4-byte header + 12 state words × 4 bytes + 4 bytes dynamic constraints */
#define TX_LEN  (4 + 12 * 4 + 4)
/* 4 control outputs × 4 bytes */
#define RX_LEN  (4 * 4)


#define FRAC_BITS 22
#define SCALE     (1 << FRAC_BITS)

/* 32-bit signed range for Q10.22: full int32 range (~ -1024 to +1024 in value) */
#define FP32_MIN  INT32_MIN
#define FP32_MAX  INT32_MAX

#define DMA_ALIGNED __attribute__((aligned(4)))

static bool fpgaControllerInitialized = false;
/* SPI uses DMA, so these buffers must stay in regular SRAM (.bss), not CCM or stack. */
static uint8_t emptyBuffer[TX_LEN] DMA_ALIGNED = {0};
static uint8_t txBuffer[TX_LEN] DMA_ALIGNED = {0};
static uint8_t rxBuffer[RX_LEN] DMA_ALIGNED = {0};
static uint8_t rxDiscardBuffer[TX_LEN] DMA_ALIGNED = {0};

static uint8_t fpgaInitCalled = 0;
static uint64_t runTimes = 0;
static bool yawSetpointInitialized = false;
static float yawSetpointDeg = 0.0f;
static float loggedU[4] = {0.0f};
static int16_t loggedU16[4] = {0};
static setpoint_t lastValidSetpoint = {0};
static bool lastValidSetpointAvailable = false;

/* Dynamic constraints and commands */
static uint32_t dynamicConstraints = 0;
static bool trajStartRequested = false;
static bool trajResetRequested = false;
static bool constraintsActive = false;
static int16_t worldMinX_mm = 0;
static int16_t worldMaxX_mm = 0;
static uint32_t lastCommandXBits = 0;
static uint32_t lastCommandYBits = 0;
static uint32_t commandSeenCount = 0;
static bool constraintsWereActive = false;
static uint32_t lastConstraintLogTick = 0;
static bool constraintLogHasRun = false;

/* Command protocol constants */
#define CMD_MAGIC_MASK  0xC0000000
#define CMD_MAGIC_VAL   0xC0000000 // Top 2 bits = 0b11
#define CMD_TYPE_MASK   0x30000000
#define CMD_TYPE_SHIFT  28

#define CMD_START_TRAJ     1
#define CMD_RESET_TRAJ     2
#define CMD_SET_CONSTRAINTS 3

static int16_t floatToInt16Saturated(const float value)
{
  const float scaled = value * 32767.0f;
  if (scaled > 32767.0f) {
    return INT16_MAX;
  }
  if (scaled < -32768.0f) {
    return INT16_MIN;
  }
  return (int16_t)lrintf(scaled);
}

static float clampFloat(const float value, const float minValue, const float maxValue)
{
  if (value < minValue) {
    return minValue;
  }
  if (value > maxValue) {
    return maxValue;
  }
  return value;
}

static void logConstraintCommandThrottled(const bool constraintCommandReceived,
                                          const int16_t relMinX,
                                          const int16_t relMaxX,
                                          const float requestedSetpointX,
                                          const float clampedSetpointX)
{
  if (!constraintCommandReceived) {
    return;
  }

  const uint32_t now = xTaskGetTickCount();
  if (constraintLogHasRun && ((now - lastConstraintLogTick) < M2T(CONSTRAINT_LOG_PERIOD_MS))) {
    return;
  }

  lastConstraintLogTick = now;
  constraintLogHasRun = true;

  DEBUG_PRINT("CONSTR: active=%u world=[%d,%d]mm rel=[%d,%d]mm pid_x=%ld->%ldmm cmd=(0x%08lX,0x%08lX) count=%lu\n",
              constraintsActive ? 1 : 0,
              (int)worldMinX_mm,
              (int)worldMaxX_mm,
              (int)relMinX,
              (int)relMaxX,
              (long)(requestedSetpointX * 1000.0f),
              (long)(clampedSetpointX * 1000.0f),
              (unsigned long)lastCommandXBits,
              (unsigned long)lastCommandYBits,
              (unsigned long)commandSeenCount);
}

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
    fpgaInitCalled++;
    runTimes = 0;
    if(fpgaControllerInitialized) {
        DEBUG_PRINT("FPGA out-of-tree controller already initialized, skipping.\n");
        return;
    }
    
    DEBUG_PRINT("FPGA out-of-tree controller init...\n");

    if (FPGA_BYPASS_TO_PID_TEST) {
        DEBUG_PRINT("FPGA PID bypass test mode enabled; SPI solver transactions disabled.\n");
        controllerPidInit();
        yawSetpointInitialized = false;
        yawSetpointDeg = 0.0f;
        lastValidSetpointAvailable = false;
        fpgaControllerInitialized = true;
        return;
    }
    
    pinMode(FPGA_CS_PIN, OUTPUT);
    pinMode(FPGA_IRQ_PIN, INPUT_PULLDOWN);
    digitalWrite(FPGA_CS_PIN, HIGH);
    spiBegin();
    spiBeginTransaction(FPGA_SPI_BAUDRATE);

    DEBUG_PRINT("FPGA out-of-tree controller initialized.\n");

    vTaskDelay(M2T(2000)); // Wait for FPGA to be ready

    DEBUG_PRINT("Sending dummy first transaction.\n");
    
    digitalWrite(FPGA_CS_PIN, LOW);

    static uint8_t initTxBuffer[TX_LEN] DMA_ALIGNED = {0};
    static uint8_t initRxBuffer[TX_LEN] DMA_ALIGNED = {0};
    initTxBuffer[3] = 0xAA;

    spiExchange(TX_LEN, initTxBuffer, initRxBuffer);
    DEBUG_PRINT("Dummy transaction sent.\n");

    controllerPidInit();
    yawSetpointInitialized = false;
    yawSetpointDeg = 0.0f;
    lastValidSetpointAvailable = false;

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
  if (fabsf(q.w) < 1e-6f) {
    v.x = 0.0f;
    v.y = 0.0f;
    v.z = 0.0f;
    return v;
  }
  v.x = q.x / q.w;
  v.y = q.y / q.w;
  v.z = q.z / q.w;
  return v;
}

static inline quaternion_t quat_mul(quaternion_t a, quaternion_t b)
{
  quaternion_t q;
  q.w = a.w*b.w - a.x*b.x - a.y*b.y - a.z*b.z;
  q.x = a.w*b.x + a.x*b.w + a.y*b.z - a.z*b.y;
  q.y = a.w*b.y - a.x*b.z + a.y*b.w + a.z*b.x;
  q.z = a.w*b.z + a.x*b.y - a.y*b.x + a.z*b.w;
  return q;
}

static inline float capAngleDeg(float angleDeg)
{
  while (angleDeg > 180.0f) {
    angleDeg -= 360.0f;
  }
  while (angleDeg < -180.0f) {
    angleDeg += 360.0f;
  }
  return angleDeg;
}

static float quat_to_yaw_deg(quaternion_t q)
{
  quaternion_t n = normalize_quat(q);
  const float sinyCosp = 2.0f * (n.w * n.z + n.x * n.y);
  const float cosyCosp = 1.0f - 2.0f * (n.y * n.y + n.z * n.z);
  return degrees(atan2f(sinyCosp, cosyCosp));
}

static float updateYawSetpointDeg(const setpoint_t *setpoint, const state_t *state)
{
  /* Keep a local yaw setpoint for modeVelocity, like the stock PID controller. */
  if (!yawSetpointInitialized) {
    yawSetpointDeg = state->attitude.yaw;
    yawSetpointInitialized = true;
  }

  if (setpoint->mode.yaw == modeVelocity) {
    yawSetpointDeg = capAngleDeg(yawSetpointDeg + setpoint->attitudeRate.yaw * (1.0f / 500.0f));
  } else if (setpoint->mode.yaw == modeAbs) {
    yawSetpointDeg = setpoint->attitude.yaw;
  } else if (setpoint->mode.quat == modeAbs) {
    yawSetpointDeg = quat_to_yaw_deg(setpoint->attitudeQuaternion);
  }

  yawSetpointDeg = capAngleDeg(yawSetpointDeg);
  return yawSetpointDeg;
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
    /* 32-bit big-endian */
    uint32_t u32 = ((uint32_t)buf[offset]     << 24)
                 | ((uint32_t)buf[offset + 1] << 16)
                 | ((uint32_t)buf[offset + 2] <<  8)
                 | ((uint32_t)buf[offset + 3] <<  0);

    int32_t s32 = (int32_t)u32;
    return (float)s32 * (1.0f / (float)SCALE);
}

void stateToTxBuffer(const setpoint_t *setpoint, const state_t *state, const sensorData_t *sensors, uint8_t *buffer) {

    const float desiredYawRad = radians(updateYawSetpointDeg(setpoint, state));
    const quaternion_t qState = normalize_quat(state->attitudeQuaternion);
    const quaternion_t qDesiredYawConj = {
        .x = 0.0f,
        .y = 0.0f,
        .z = -sinf(0.5f * desiredYawRad),
        .w = cosf(0.5f * desiredYawRad)
    };
    const quaternion_t qError = normalize_quat(quat_mul(qDesiredYawConj, qState));
    struct vec phi = quat_2_rp(qError); // Rodrigues parameters of attitude error wrt desired yaw
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

    /* Append dynamic constraints at the end of the buffer (index 52-55) */
    buffer[52] = (uint8_t)(dynamicConstraints >> 24);
    buffer[53] = (uint8_t)(dynamicConstraints >> 16);
    buffer[54] = (uint8_t)(dynamicConstraints >>  8);
    buffer[55] = (uint8_t)(dynamicConstraints >>  0);
}

void rxBufferToControl(const uint8_t *buffer, control_t *control) {
    control->controlMode = controlModeForce;
    for (int i = 0; i < 4; i++) {
      loggedU[i] = fixed_32bit_to_float_at(buffer, 4 * i);
      /* FPGA already includes hover thrust (U_HOVER) */
      control->normalizedForces[i] = loggedU[i];
      loggedU16[i] = floatToInt16Saturated(control->normalizedForces[i]);
    }
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

    // Parse external commands from setpoint multiplexing
    uint32_t x_bits, y_bits;
    bool isCommandPacket = false;
    bool constraintCommandReceived = false;
    memcpy(&x_bits, &setpoint->position.x, 4);
    memcpy(&y_bits, &setpoint->position.y, 4);

    if (((x_bits & CMD_MAGIC_MASK) == CMD_MAGIC_VAL) &&
        ((y_bits & CMD_MAGIC_MASK) == CMD_MAGIC_VAL)) {
        isCommandPacket = true;
        lastCommandXBits = x_bits;
        lastCommandYBits = y_bits;
        commandSeenCount++;

        uint8_t cmdType = (x_bits & CMD_TYPE_MASK) >> CMD_TYPE_SHIFT;
        if (cmdType == CMD_START_TRAJ) {
            trajStartRequested = true;
            trajResetRequested = false;
            DEBUG_PRINT("CMD: START TRAJECTORY\n");
        } else if (cmdType == CMD_RESET_TRAJ) {
            trajStartRequested = false;
            trajResetRequested = true;
            DEBUG_PRINT("CMD: RESET TRAJECTORY\n");
        } else if (cmdType == CMD_SET_CONSTRAINTS) {
            /* We extract the lower 16 bits of x and y as world-frame X boundaries (in mm) */
            worldMinX_mm = (int16_t)(x_bits & 0x0000FFFF);
            worldMaxX_mm = (int16_t)(y_bits & 0x0000FFFF);
            constraintCommandReceived = true;

            if (worldMinX_mm == 0 && worldMaxX_mm == 0) {
                constraintsActive = false;
                dynamicConstraints = 0;
                if (constraintsWereActive) {
                    DEBUG_PRINT("CMD: CONSTRAINTS DISABLED\n");
                }
            } else {
                constraintsActive = true;
                if (!constraintsWereActive) {
                    DEBUG_PRINT("CMD: CONSTRAINTS ENABLED\n");
                }
            }
            constraintsWereActive = constraintsActive;
        }
    }

    if (!isCommandPacket) {
        lastValidSetpoint = *setpoint;
        lastValidSetpointAvailable = true;
    }

    const setpoint_t *effectiveSetpoint = setpoint;
    if (isCommandPacket && lastValidSetpointAvailable) {
        effectiveSetpoint = &lastValidSetpoint;
    } else if (isCommandPacket) {
        DEBUG_PRINT("CMD: no cached setpoint available, using command packet as fallback\n");
    }

    /* Handle coordinate shifting: relative_boundary = world_boundary - current_setpoint */
    int16_t relMinX = 0;
    int16_t relMaxX = 0;
    if (constraintsActive) {
        float setpointX_mm = effectiveSetpoint->position.x * 1000.0f;
        relMinX = worldMinX_mm - (int16_t)setpointX_mm;
        relMaxX = worldMaxX_mm - (int16_t)setpointX_mm;

        /* Pack into the 32-bit word for the FPGA */
        dynamicConstraints = ((uint16_t)relMinX) | (((uint32_t)(uint16_t)relMaxX) << 16);
    }

    if (FPGA_BYPASS_TO_PID_TEST) {
        setpoint_t pidSetpoint = *effectiveSetpoint;
        const float requestedSetpointX = pidSetpoint.position.x;

        if (constraintsActive) {
            float minX = (float)worldMinX_mm * 0.001f;
            float maxX = (float)worldMaxX_mm * 0.001f;
            if (minX > maxX) {
                const float tmp = minX;
                minX = maxX;
                maxX = tmp;
            }
            pidSetpoint.position.x = clampFloat(pidSetpoint.position.x, minX, maxX);
        }

        logConstraintCommandThrottled(constraintCommandReceived,
                                      relMinX,
                                      relMaxX,
                                      requestedSetpointX,
                                      pidSetpoint.position.x);

        trajStartRequested = false;
        trajResetRequested = false;
        controllerPid(control, &pidSetpoint, sensors, state, tick);
        return;
    }

    stateToTxBuffer(effectiveSetpoint, state, sensors, txBuffer);

    /* Apply commands to the header word of txBuffer */
    if (trajStartRequested) txBuffer[2] |= 0x01;
    if (trajResetRequested) txBuffer[2] |= 0x02;
    if (trajStartRequested || trajResetRequested) {
        DEBUG_PRINT("TX: hdr=%02X %02X %02X %02X start=%u reset=%u constraints=0x%08lX last_cmd=(0x%08lX,0x%08lX)\n",
                    txBuffer[0],
                    txBuffer[1],
                    txBuffer[2],
                    txBuffer[3],
                    trajStartRequested ? 1 : 0,
                    trajResetRequested ? 1 : 0,
                    (unsigned long)dynamicConstraints,
                    (unsigned long)lastCommandXBits,
                    (unsigned long)lastCommandYBits);
    }
    trajStartRequested = false;
    trajResetRequested = false;
  
    // uint64_t start = usecTimestamp();

    for(int i = 0; i < 50000; i++) {
        if(digitalRead(FPGA_IRQ_PIN) == HIGH) {
            break;
        }
    }
    memset(rxBuffer, 0, sizeof(rxBuffer));


    for(int i = 0; i < 1000; i++) {
        spiExchange(1, emptyBuffer, rxBuffer);
        if(rxBuffer[0] == 0xFF) 
        {
            break;    
        }
    }
    /* Consume the remaining 3 bytes of the 32-bit ready word (0xFF 0xFF 0xFF 0xFF) */
    spiExchange(3, emptyBuffer, rxDiscardBuffer);

    spiExchange(RX_LEN, emptyBuffer, rxBuffer);

    digitalWrite(FPGA_CS_PIN, HIGH);
    sleepus(1);
    digitalWrite(FPGA_CS_PIN, LOW);

    spiExchange(TX_LEN, txBuffer, rxDiscardBuffer);

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

LOG_GROUP_START(fpga)
LOG_ADD(LOG_INT16, u1_16, &loggedU16[0])
LOG_ADD(LOG_INT16, u2_16, &loggedU16[1])
LOG_ADD(LOG_INT16, u3_16, &loggedU16[2])
LOG_ADD(LOG_INT16, u4_16, &loggedU16[3])
LOG_GROUP_STOP(fpga)
//--------------------------------------------------------------
// Parameters
//--------------------------------------------------------------

// PARAM_GROUP_START(fpga)
// PARAM_GROUP_STOP(fpga)
