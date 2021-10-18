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

package org.rivierarobotics.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.rivierarobotics.appjack.Logging;
import org.rivierarobotics.appjack.MechLogger;
import org.rivierarobotics.util.MotorUtil;

/**
 * Subsystem for the climb. A simple rack-and-pinion climb. Uses a relative
 * encoder. No PID because of high gear reduction ratio. Inches used instead
 * of meters for convenience. Contains an external limit switch wired into the
 * reverse limit switch of the motor (Falcon) to detect minimum safe
 * operating range. Maximum calculated off theoretical maximums. Soft limits
 * used for maximum range enforcement.
 *
 * @see Climb.Position
 */
public class Climb extends SubsystemBase implements RRSubsystem {
    private static final double TICKS_PER_INCH = 39114;
    private static final double MAX_INCHES = 34.5;
    private final WPI_TalonFX climbFalcon;
    private final MechLogger logger;

    public Climb(int motorId) {
        this.climbFalcon = new WPI_TalonFX(motorId);
        this.logger = Logging.getLogger(getClass());

        // This PID is not tuned, currently not used
        MotorUtil.setupMotionMagic(FeedbackDevice.IntegratedSensor,
                new PIDConfig(10, 0, 0, 0), 0, climbFalcon);
        climbFalcon.setSensorPhase(false);
        climbFalcon.configForwardLimitSwitchSource(LimitSwitchSource.Deactivated, LimitSwitchNormal.NormallyOpen);
        climbFalcon.configForwardSoftLimitThreshold(MAX_INCHES * TICKS_PER_INCH);
        climbFalcon.configForwardSoftLimitEnable(true);
        climbFalcon.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);
        climbFalcon.setNeutralMode(NeutralMode.Brake);
    }

    public double getTicksPerInch() {
        return TICKS_PER_INCH;
    }

    /**
     * Checks the limit switch to see if the climb is at the bottom of its
     * safe movement range. Wired into reverse limit switch port. Required as
     * the climb encoder is relative (not absolute/pulse width).
     *
     * @return if the climb is at the bottom of its safe movement range.
     */
    public boolean isAtBottom() {
        return climbFalcon.isRevLimitSwitchClosed() == 1;
    }

    /**
     * Reset the encoder to zero ticks. Default selected sensor is relative
     * (correct for the climb), not absolute.
     */
    public void resetEncoder() {
        climbFalcon.setSelectedSensorPosition(0);
    }

    public void setPositionTicks(double positionTicks) {
        logger.setpointChange(positionTicks);
        climbFalcon.set(ControlMode.MotionMagic, positionTicks);
    }

    @Override
    public double getPositionTicks() {
        return climbFalcon.getSelectedSensorPosition();
    }

    @Override
    public void setPower(double pwr) {
        logger.powerChange(pwr);
        climbFalcon.set(ControlMode.PercentOutput, pwr);
    }

    @Override
    public void periodic() {
        if (isAtBottom() && climbFalcon.get() < 0) {
            climbFalcon.stopMotor();
        }
    }

    /**
     * Defines the position of the climb for certain presets. Enum constants
     * contain percentages, stored as ticks (calculated).
     */
    public enum Position {
        ZERO(0.03),
        HALF(0.7),
        MAX(0.92);

        private final double ticks;

        Position(double pct) {
            this.ticks = pct * TICKS_PER_INCH * MAX_INCHES;
        }

        public double getTicks() {
            return ticks;
        }
    }
}
