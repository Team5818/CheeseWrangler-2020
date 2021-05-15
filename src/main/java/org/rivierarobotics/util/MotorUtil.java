/*
 * This file is part of CheeseWrangler-2020, licensed under the GNU General Public License (GPLv3).
 *
 * Copyright (c) Riviera Robotics <https://github.com/Team5818>
 * Copyright (c) contributors
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

package org.rivierarobotics.util;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.BaseTalon;
import org.rivierarobotics.subsystems.PIDConfig;

/**
 * Utility methods relating to robot motor movement.
 */
public class MotorUtil {
    private MotorUtil() {
    }

    /**
     * Configures Motion Magic motion profiling on given CTRE
     * Talon, Victor, or Falcon controlled motors.
     *
     * <p>Uses the internal 1 kHz clock of the controller instead of the 20 ms
     * RoboRio clock. This is recommended as it removes the need to make
     * custom motion profiles, leading to faster turnaround times on subsystems.
     * As a warning, this first resets all motor settings to factory default
     * and then configures the feedback sensor based on the passed value.
     * As such is is recommended that this be the first motor configuration call
     * in any subsystem. Note that maximum velocity and acceleration will
     * not be set if <code>maxVel == 0</code>.</p>
     *
     * @param sensor the sensor attached to the controller used for loop feedback.
     * @param pidConfig the PIDF and range values to use on the controller.
     * @param maxVel maximum velocity of the profile in ticks per 100ms.
     * @param motors the motors for which Motion Magic is enabled on.
     */
    @SafeVarargs
    public static <T extends BaseTalon> void setupMotionMagic(FeedbackDevice sensor, PIDConfig pidConfig, int maxVel, T... motors) {
        int periodMs = 10;
        int timeoutMs = 10;
        for (T motor : motors) {
            motor.configFactoryDefault();
            motor.selectProfileSlot(0, 0);
            motor.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, periodMs, timeoutMs);
            motor.setStatusFramePeriod(StatusFrame.Status_10_MotionMagic, periodMs, timeoutMs);
            if (sensor == FeedbackDevice.PulseWidthEncodedPosition || sensor == FeedbackDevice.IntegratedSensor) {
                motor.setStatusFramePeriod(StatusFrameEnhanced.Status_8_PulseWidth, periodMs, timeoutMs);
                motor.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, periodMs, timeoutMs);
            }
            motor.configSelectedFeedbackSensor(sensor, 0, timeoutMs);

            motor.configNominalOutputForward(0);
            motor.configNominalOutputReverse(0);
            motor.configPeakOutputForward(pidConfig.getRange());
            motor.configPeakOutputReverse(-pidConfig.getRange());

            motor.config_kP(0, pidConfig.getP());
            motor.config_kI(0, pidConfig.getI());
            motor.config_kD(0, pidConfig.getD());
            motor.config_kF(0, pidConfig.getF());

            if (maxVel != 0) {
                motor.configMotionCruiseVelocity(maxVel);
                motor.configMotionAcceleration(maxVel);
            }
        }
    }

    /**
     * Places limits on the range of CTRE Talon, Victor, or Falcon controlled motors.
     *
     * <p>Limits are defined in ticks and apply to both power and positional control sets.
     * It is still recommended that both are limited manually if possible. Note that
     * this is a hard stop (despite being a soft limit) and does not account for velocity
     * accumulated while moving. This value does not persist after power-off.</p>
     *
     * @param forward the maximum ticks in the forward/positive direction.
     * @param reverse the minimum ticks in the reverse/backward/negative direction.
     * @param motors the motors to apply the soft limits onto.
     */
    @SafeVarargs
    public static <T extends BaseTalon> void setSoftLimits(int forward, int reverse, T... motors) {
        for (T motor : motors) {
            motor.configForwardLimitSwitchSource(LimitSwitchSource.Deactivated, LimitSwitchNormal.NormallyOpen);
            motor.configReverseLimitSwitchSource(LimitSwitchSource.Deactivated, LimitSwitchNormal.NormallyOpen);
            motor.configForwardSoftLimitThreshold(forward);
            motor.configReverseSoftLimitThreshold(reverse);
            motor.configForwardSoftLimitEnable(true);
            motor.configReverseSoftLimitEnable(true);
        }
    }
}
