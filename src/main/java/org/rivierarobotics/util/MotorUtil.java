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
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.can.BaseMotorController;
import com.ctre.phoenix.motorcontrol.can.BaseTalon;
import org.rivierarobotics.subsystems.PIDConfig;

public class MotorUtil {
    private MotorUtil() {
    }

    @SafeVarargs
    public static <T extends BaseMotorController> void setInverted(boolean invert, T... motors) {
        for (T motor : motors) {
            motor.setInverted(invert);
        }
    }

    @SafeVarargs
    public static <T extends BaseMotorController> void setupMotionMagic(FeedbackDevice sensor, PIDConfig pidConfig, int maxVel, T... motors) {
        for (T motor : motors) {
            motor.configFactoryDefault();
            motor.selectProfileSlot(0, 0);
            motor.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, 10, 10);
            motor.setStatusFramePeriod(StatusFrame.Status_10_MotionMagic, 10, 10);
            motor.configSelectedFeedbackSensor(sensor, 0, 100);

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

    @SafeVarargs
    public static <T extends BaseMotorController> void setNeutralMode(NeutralMode mode, T... motors) {
        for (T motor : motors) {
            motor.setNeutralMode(mode);
        }
    }

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
