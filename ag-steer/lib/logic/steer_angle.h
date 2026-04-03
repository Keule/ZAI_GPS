/**
 * @file steer_angle.h
 * @brief Steering angle sensor driver over SPI.
 *
 * Reads the current steering angle in degrees.
 */

#pragma once

/// Initialise steering angle sensor.
void steerAngleInit(void);

/// Read current steering angle [degrees].
float steerAngleReadDeg(void);
