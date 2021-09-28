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
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.rivierarobotics.appjack.Logging;
import org.rivierarobotics.appjack.MechLogger;
import org.rivierarobotics.util.MathUtil;
import org.rivierarobotics.util.MotorUtil;
import org.rivierarobotics.util.RSTab;
import org.rivierarobotics.util.RobotShuffleboard;
import org.rivierarobotics.util.ShooterConstants;

/**
 * Subsystem for flywheel. Spins at a velocity and shoots balls previously
 * pushed by ejectors from Cheese Wheel.
 */
public class Flywheel extends SubsystemBase implements RRSubsystem {
    private static final double MIN_SHOOTING_VEL = 1000.0;
    private static double targetVel = 0;
    private static double tolerance = 70;
    private final WPI_TalonFX flywheelFalcon;
    private final MechLogger logger;
    private final RSTab tab;

    public Flywheel(int id, RobotShuffleboard shuffleboard) {
        this.logger = Logging.getLogger(getClass());
        this.tab = shuffleboard.getTab("Vision");

        this.flywheelFalcon = new WPI_TalonFX(id);
        // Previous configuration, also works
        //new PIDConfig((1023.0 * 0.5) / 500, (1023.0 * 0.01) / 500, 0.0, (1023.0 * 0.75) / 15900), 0, flywheelFalcon);
        MotorUtil.setupMotionMagic(FeedbackDevice.IntegratedSensor,
            new PIDConfig(1.5, 0.0, 0.3, (1023.0 * 0.72) / 15900), 0, flywheelFalcon);
        flywheelFalcon.setInverted(false);
        flywheelFalcon.setNeutralMode(NeutralMode.Coast);
    }

    @Override
    public double getPositionTicks() {
        return flywheelFalcon.getSelectedSensorVelocity();
    }

    /**
     * Get the velocity that a ball will have at the current velocity.
     *
     * @return the ball velocity in meters per second.
     */
    public double getBallVelocity() {
        return ShooterConstants.ticksToVelocity(flywheelFalcon.getSelectedSensorVelocity());
    }

    @Override
    public void setPower(double pwr) {
        logger.powerChange(pwr);
        flywheelFalcon.set(ControlMode.PercentOutput, pwr);
    }

    /**
     * Check if the velocity is within tolerance of the target.
     *
     * @return if the velocity is within tolerance of target.
     */
    public boolean withinTolerance() {
        return MathUtil.isWithinTolerance(getPositionTicks(), targetVel, tolerance);
    }

    /**
     * Change the flywheel pre-shoot tolerance. A lower value will cause more
     * accurate shots with a longer spool-up time. Minimum 0.
     *
     * @param amount the offset amount to change the tolerance by.
     */
    public void stepTolerance(int amount) {
        tolerance = Math.max(0, tolerance + amount);
        tab.setEntry("SetTolerance", tolerance);
    }

    public static double getTolerance() {
        return tolerance;
    }

    /**
     * Set the velocity of the flywheel. Controlled by PIDF loop. Will cut
     * off current and set velocity to zero if stopped. Passed velocity is
     * automatically set as target.
     *
     * @param vel the velocity in ticks per 100ms to set.
     */
    public void setVelocity(double vel) {
        tab.setEntry("Flywheel Set Vel", vel);
        targetVel = vel;
        logger.setpointChange(vel);
        if (vel == 0) {
            flywheelFalcon.set(TalonFXControlMode.Velocity, 0.0);
            flywheelFalcon.set(TalonFXControlMode.Current, 0.0);
        } else {
            flywheelFalcon.set(TalonFXControlMode.Velocity, vel);
        }
    }

    public boolean isBelowMinShootingVel() {
        return getPositionTicks() < MIN_SHOOTING_VEL;
    }

    public MotorTemp getTemp() {
        return new MotorTemp(flywheelFalcon.getDeviceID(), flywheelFalcon.getTemperature(), "FlywheelFalcon");
    }
}
