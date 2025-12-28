/**
 * enumerators for meca500 response  and codes. See programming manual
 * 
 * Author: Garrison Johnston
 */

#pragma once

#include <cstdint>
#include <unordered_map>
#include <string>
#include <iostream>

namespace meca500_hardware
{
    /**
     * Codes to indicate a success
     */
    enum class SuccessCode : uint16_t 
    {
        // homing
        HOMING_DONE              = 2002,
        ALREADY_HOMED            = 2003,
        HOMING_FAILED            = 1014,

        // activation
        MOTORS_ACTIVATED         = 2000,
        MOTORS_ALREADY_ACTIVATED = 2001,

        // deactivation
        MOTORS_DEACTIVATED        = 2004
    };

    /**
     * @brief Map of all ErrorCode values to human-readable messages.
     *
     * This map is used to provide descriptive messages for error codes returned by the robot.
     */
    const std::unordered_map<uint16_t, std::string> ErrorCode = {
        {1000, "Command buffer full"},
        {1001, "Empty or unknown command"},
        {1002, "Syntax error"},
        {1003, "Argument error"},
        {1005, "Robot not activated"},
        {1006, "Robot not homed"},
        {1007, "Joint over limit"},
        {1010, "Linear move reorientation 180"},
        {1011, "Robot in error"},
        {1012, "Linear move singularity"},
        {1013, "Activation failed"},
        {1014, "Homing failed"},
        {1032, "Homing failed outside joint limits"},
        {1016, "Destination out of reach"},
        {1022, "Not saving program"},
        {1023, "Command ignored in offline mode"},
        {1024, "Mastering needed"},
        {1025, "Error reset impossible"},
        {1026, "Deactivation required"},
        {1027, "Invalid simulation mode change"},
        {1554, "Variable modification failed"},
        {3001, "Another user connected"},
        {3002, "Firmware upgrade in progress"},
        {3003, "Command too long"},
        {3005, "Motion error"},
        {3006, "Drive communication error"},
        {3009, "Robot initialization failed"},
        {3014, "Saved program problem"},
        {3017, "No offline program"},
        {3020, "Offline program invalid"},
        {3025, "Gripper error"},
        {3026, "Maintenance check failed"},
        {3027, "Internal error"},
        {3029, "Excessive torque"},
        {3031, "Invalid text API command"},
        {3037, "Pneumatic module error"},
        {3039, "EOAT firmware outdated"},
        {3041, "Imminent collision error"},
        {3042, "Firmware update failure"},
        {3043, "EOAT communication errors"},
        {3044, "IO port communication error"},
        {3045, "Imminent collision deceleration"},
        {3047, "Drive mount failure"},
        {3049, "Work zone limit error"}
    };

    /**
     * @brief Check if a given error code exists in the map of known errors.
     * @param code The error code to check.
     * @return true if the code exists, false otherwise.
     */
    inline bool isErrorCode(uint16_t code)
    {
        return ErrorCode.count(code) > 0;
    }

    /**
     * @brief Check if a given error code exists in the map of known errors.
     * @param code The error code to check.
     * @return true if the code exists, false otherwise.
     */
    inline bool isErrorCode(SuccessCode code)
    {
        return isErrorCode(static_cast<uint16_t>(code));
    }

    /**
     * @brief Get a human-readable message for a given error code.
     * @param code The error code to query.
     * @return A descriptive string for the error, or "Unknown error code" if not found.
     */
    inline std::string getErrorMessage(uint16_t code)
    {
        auto it = ErrorCode.find(code);
        return (it != ErrorCode.end()) ? it->second : "Unknown error code";
    }

    /**
     * @brief Get a human-readable message for a given error code.
     * @param code The error code to query.
     * @return A descriptive string for the error, or "Unknown error code" if not found.
     */
    inline std::string getErrorMessage(SuccessCode code)
    {
        return getErrorMessage(static_cast<uint16_t>(code));
    }
}

