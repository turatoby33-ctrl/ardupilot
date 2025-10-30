/**
 * @file GCS_ServoRelay.cpp
 * @brief Servo and relay control via MAVLink commands
 *
 * This file implements servo and relay control commands for EduCopter:
 * - DO_SET_SERVO: Set servo PWM value
 * - DO_SET_RELAY: Set relay on/off
 * - DO_REPEAT_SERVO: Pulse servo repeatedly
 * - DO_REPEAT_RELAY: Pulse relay repeatedly
 *
 * Useful for controlling:
 * - Camera gimbals
 * - Parachute deployment
 * - Cargo release mechanisms
 * - Landing gear
 * - Lights
 * - Other auxiliary equipment
 *
 * @author EduCopter Development Team
 * @date 2025
 */

#include "GCS.h"
#include "GCS_config.h"
#include <cstring>

namespace EduCopter {
namespace GCS {

#if EDUCOPTER_SERVO_RELAY_ENABLED

// External servo/relay interface (implemented by vehicle)
extern bool setServoPWM(uint8_t servoNum, uint16_t pwmUS);
extern bool setRelay(uint8_t relayNum, bool state);
extern uint16_t getServoPWM(uint8_t servoNum);
extern bool getRelayState(uint8_t relayNum);
extern bool isServoValid(uint8_t servoNum);
extern bool isRelayValid(uint8_t relayNum);

// Servo limits
static const uint16_t SERVO_PWM_MIN = 500;
static const uint16_t SERVO_PWM_MAX = 2500;
static const uint8_t MAX_SERVOS = 16;
static const uint8_t MAX_RELAYS = 6;

// Repeat servo/relay state
struct RepeatState {
    bool active;
    uint8_t channel;
    uint16_t repeatCount;
    uint16_t repeatDelay;
    uint16_t currentCount;
    uint32_t lastToggleMS;
    bool isServo;
    uint16_t servoPWM;
};

static RepeatState s_repeatState = {false, 0, 0, 0, 0, 0, false, 0};

/**
 * @brief Handle MAV_CMD_DO_SET_SERVO command
 *
 * Sets a servo to a specific PWM value.
 */
MAV_RESULT GCSChannel::handleCommandSetServo(const mavlink_command_long_t& cmd)
{
    // param1: Servo number (1-16)
    // param2: PWM value (500-2500 microseconds)

    uint8_t servoNum = static_cast<uint8_t>(cmd.param1);
    uint16_t pwm = static_cast<uint16_t>(cmd.param2);

    // Validate servo number
    if (servoNum == 0 || servoNum > MAX_SERVOS) {
        sendText(MAV_SEVERITY_WARNING, "Invalid servo number");
        return MAV_RESULT_FAILED;
    }

    if (!isServoValid(servoNum)) {
        sendText(MAV_SEVERITY_WARNING, "Servo not available");
        return MAV_RESULT_FAILED;
    }

    // Validate PWM range
    if (pwm < SERVO_PWM_MIN || pwm > SERVO_PWM_MAX) {
        sendText(MAV_SEVERITY_WARNING, "PWM out of range (500-2500)");
        return MAV_RESULT_FAILED;
    }

    // Set servo
    if (setServoPWM(servoNum, pwm)) {
        char buf[80];
        snprintf(buf, sizeof(buf), "Servo %u set to %u us", servoNum, pwm);
        sendText(MAV_SEVERITY_INFO, buf);
        return MAV_RESULT_ACCEPTED;
    } else {
        sendText(MAV_SEVERITY_ERROR, "Servo set failed");
        return MAV_RESULT_FAILED;
    }
}

/**
 * @brief Handle MAV_CMD_DO_SET_RELAY command
 *
 * Turns a relay on or off.
 */
MAV_RESULT GCSChannel::handleCommandSetRelay(const mavlink_command_long_t& cmd)
{
    // param1: Relay number (0-5)
    // param2: State (0=off, 1=on)

    uint8_t relayNum = static_cast<uint8_t>(cmd.param1);
    bool state = (cmd.param2 > 0.5f);

    // Validate relay number
    if (relayNum >= MAX_RELAYS) {
        sendText(MAV_SEVERITY_WARNING, "Invalid relay number");
        return MAV_RESULT_FAILED;
    }

    if (!isRelayValid(relayNum)) {
        sendText(MAV_SEVERITY_WARNING, "Relay not available");
        return MAV_RESULT_FAILED;
    }

    // Set relay
    if (setRelay(relayNum, state)) {
        char buf[80];
        snprintf(buf, sizeof(buf), "Relay %u: %s",
                 relayNum, state ? "ON" : "OFF");
        sendText(MAV_SEVERITY_INFO, buf);
        return MAV_RESULT_ACCEPTED;
    } else {
        sendText(MAV_SEVERITY_ERROR, "Relay set failed");
        return MAV_RESULT_FAILED;
    }
}

/**
 * @brief Handle MAV_CMD_DO_REPEAT_SERVO command
 *
 * Pulses a servo repeatedly.
 */
MAV_RESULT GCSChannel::handleCommandRepeatServo(const mavlink_command_long_t& cmd)
{
    // param1: Servo number (1-16)
    // param2: PWM value (500-2500 microseconds)
    // param3: Repeat count
    // param4: Delay between pulses (seconds)

    uint8_t servoNum = static_cast<uint8_t>(cmd.param1);
    uint16_t pwm = static_cast<uint16_t>(cmd.param2);
    uint16_t count = static_cast<uint16_t>(cmd.param3);
    uint16_t delayMS = static_cast<uint16_t>(cmd.param4 * 1000.0f);

    // Validate parameters
    if (servoNum == 0 || servoNum > MAX_SERVOS) {
        sendText(MAV_SEVERITY_WARNING, "Invalid servo number");
        return MAV_RESULT_FAILED;
    }

    if (pwm < SERVO_PWM_MIN || pwm > SERVO_PWM_MAX) {
        sendText(MAV_SEVERITY_WARNING, "PWM out of range");
        return MAV_RESULT_FAILED;
    }

    if (count == 0 || count > 100) {
        sendText(MAV_SEVERITY_WARNING, "Invalid repeat count");
        return MAV_RESULT_FAILED;
    }

    // Start repeat sequence
    s_repeatState.active = true;
    s_repeatState.isServo = true;
    s_repeatState.channel = servoNum;
    s_repeatState.servoPWM = pwm;
    s_repeatState.repeatCount = count;
    s_repeatState.repeatDelay = delayMS;
    s_repeatState.currentCount = 0;
    s_repeatState.lastToggleMS = millis();

    char buf[80];
    snprintf(buf, sizeof(buf), "Servo %u repeat: %u times, %u ms delay",
             servoNum, count, delayMS);
    sendText(MAV_SEVERITY_INFO, buf);

    return MAV_RESULT_ACCEPTED;
}

/**
 * @brief Handle MAV_CMD_DO_REPEAT_RELAY command
 *
 * Pulses a relay repeatedly.
 */
MAV_RESULT GCSChannel::handleCommandRepeatRelay(const mavlink_command_long_t& cmd)
{
    // param1: Relay number (0-5)
    // param2: Repeat count
    // param3: Delay between pulses (seconds)

    uint8_t relayNum = static_cast<uint8_t>(cmd.param1);
    uint16_t count = static_cast<uint16_t>(cmd.param2);
    uint16_t delayMS = static_cast<uint16_t>(cmd.param3 * 1000.0f);

    // Validate parameters
    if (relayNum >= MAX_RELAYS) {
        sendText(MAV_SEVERITY_WARNING, "Invalid relay number");
        return MAV_RESULT_FAILED;
    }

    if (count == 0 || count > 100) {
        sendText(MAV_SEVERITY_WARNING, "Invalid repeat count");
        return MAV_RESULT_FAILED;
    }

    // Start repeat sequence
    s_repeatState.active = true;
    s_repeatState.isServo = false;
    s_repeatState.channel = relayNum;
    s_repeatState.repeatCount = count;
    s_repeatState.repeatDelay = delayMS;
    s_repeatState.currentCount = 0;
    s_repeatState.lastToggleMS = millis();

    char buf[80];
    snprintf(buf, sizeof(buf), "Relay %u repeat: %u times, %u ms delay",
             relayNum, count, delayMS);
    sendText(MAV_SEVERITY_INFO, buf);

    return MAV_RESULT_ACCEPTED;
}

/**
 * @brief Update repeat servo/relay sequences
 *
 * Called periodically to handle timed servo/relay pulses.
 */
void GCSChannel::updateServoRelay()
{
    if (!s_repeatState.active) {
        return;
    }

    uint32_t nowMS = millis();

    // Check if it's time for next toggle
    if (nowMS - s_repeatState.lastToggleMS < s_repeatState.repeatDelay) {
        return;
    }

    s_repeatState.lastToggleMS = nowMS;

    bool isEven = (s_repeatState.currentCount % 2) == 0;

    if (s_repeatState.isServo) {
        // Pulse servo between PWM and neutral (1500)
        uint16_t pwm = isEven ? s_repeatState.servoPWM : 1500;
        setServoPWM(s_repeatState.channel, pwm);
    } else {
        // Toggle relay
        setRelay(s_repeatState.channel, isEven);
    }

    s_repeatState.currentCount++;

    // Check if sequence complete
    if (s_repeatState.currentCount >= s_repeatState.repeatCount * 2) {
        s_repeatState.active = false;

        char buf[80];
        snprintf(buf, sizeof(buf), "%s %u repeat complete",
                 s_repeatState.isServo ? "Servo" : "Relay",
                 s_repeatState.channel);
        sendText(MAV_SEVERITY_INFO, buf);
    }
}

/**
 * @brief Send servo output status
 *
 * Sends current servo PWM values.
 */
void GCSChannel::sendServoOutputRaw()
{
    if (!hasPayloadSpace(MAVLINK_MSG_ID_SERVO_OUTPUT_RAW)) {
        return;
    }

    uint16_t servo[16];
    for (uint8_t i = 0; i < 16; i++) {
        if (isServoValid(i + 1)) {
            servo[i] = getServoPWM(i + 1);
        } else {
            servo[i] = 0;
        }
    }

    mavlink_message_t msg;
    mavlink_msg_servo_output_raw_pack(
        m_mavlink.getSystemID(),
        m_mavlink.getComponentID(),
        &msg,
        millis(),
        0, // Port (main output)
        servo[0], servo[1], servo[2], servo[3],
        servo[4], servo[5], servo[6], servo[7],
        servo[8], servo[9], servo[10], servo[11],
        servo[12], servo[13], servo[14], servo[15]
    );

    sendMessage(&msg);
}

/**
 * @brief Send relay status as text
 */
void GCSChannel::sendRelayStatus()
{
    char buf[100];
    snprintf(buf, sizeof(buf), "Relay status:");
    sendText(MAV_SEVERITY_INFO, buf);

    for (uint8_t i = 0; i < MAX_RELAYS; i++) {
        if (isRelayValid(i)) {
            bool state = getRelayState(i);
            snprintf(buf, sizeof(buf), "  Relay %u: %s",
                     i, state ? "ON" : "OFF");
            sendText(MAV_SEVERITY_INFO, buf);
        }
    }
}

#else // EDUCOPTER_SERVO_RELAY_ENABLED

// Servo/relay disabled - provide stub implementations
MAV_RESULT GCSChannel::handleCommandSetServo(const mavlink_command_long_t& cmd)
{
    sendText(MAV_SEVERITY_WARNING, "Servo control not supported");
    return MAV_RESULT_UNSUPPORTED;
}

MAV_RESULT GCSChannel::handleCommandSetRelay(const mavlink_command_long_t& cmd)
{
    sendText(MAV_SEVERITY_WARNING, "Relay control not supported");
    return MAV_RESULT_UNSUPPORTED;
}

MAV_RESULT GCSChannel::handleCommandRepeatServo(const mavlink_command_long_t& cmd)
{
    return MAV_RESULT_UNSUPPORTED;
}

MAV_RESULT GCSChannel::handleCommandRepeatRelay(const mavlink_command_long_t& cmd)
{
    return MAV_RESULT_UNSUPPORTED;
}

void GCSChannel::updateServoRelay()
{
    // No-op
}

void GCSChannel::sendServoOutputRaw()
{
    // No-op
}

#endif // EDUCOPTER_SERVO_RELAY_ENABLED

} // namespace GCS
} // namespace EduCopter
