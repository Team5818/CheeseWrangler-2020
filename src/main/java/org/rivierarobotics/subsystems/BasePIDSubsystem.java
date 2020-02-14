/*
 * This file is part of Placeholder-2020, licensed under the GNU General Public License (GPLv3).
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

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.rivierarobotics.util.ShuffleUtil;

public abstract class BasePIDSubsystem extends SubsystemBase {
    private final double pidRange, anglesOrInchesToTicks, kP, kI, kD, ticksToAngles;
    private final PIDController pidController;
    private boolean pidEnabled = false;
    private ShuffleboardTab dash;

    public BasePIDSubsystem(double kP, double kI, double kD, double pidRange) {
        this(kP, kI, kD, pidRange, 0.0, 4096.0 / 360, 360 / 4096.0);
    }

    public BasePIDSubsystem(double kP, double kI, double kD, double pidRange, double tolerance, double anglesOrInchesToTicks, double ticksToAngles) {
        this.pidController = new PIDController(kP, kI, kD, 0.005);
        this.pidRange = pidRange;
        this.anglesOrInchesToTicks = anglesOrInchesToTicks;
        this.ticksToAngles = ticksToAngles;
        pidController.setTolerance(tolerance);
        this.dash = Shuffleboard.getTab(getName());
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
    }

    private void tickPid() {
        double pidPower = Math.min(pidRange, Math.max(-pidRange, pidController.calculate(getPositionTicks())));
        if (pidEnabled) {
            setPower(pidPower);
        }
    }

    public final PIDController getPidController() {
        return pidController;
    }

    public double getAnglesOrInchesToTicks() {
        return anglesOrInchesToTicks;
    }

    public double getTicksToAngles() {
        return ticksToAngles;
    }

    public double getPosition() {
        return getPositionTicks() / anglesOrInchesToTicks;
    }

    public void setPosition(double position) {
        pidEnabled = true;
        pidController.setSetpoint(position * anglesOrInchesToTicks);
    }

    public void setManualPower(double pwr) {
        pidEnabled = false;
        setPower(pwr);
    }

    public abstract double getPositionTicks();

    public void setPositionTicks(double position) {
        pidEnabled = true;
        pidController.setSetpoint(position);
    }

    protected abstract void setPower(double pwr);

    private void displayShuffleboard() {
        ShuffleUtil.setOutEntry(dash, "Position", getPosition());
        ShuffleUtil.setOutEntry(dash, "Position Ticks", getPositionTicks());
        ShuffleUtil.setOutEntry(dash, "Setpoint", pidController.getSetpoint());
        ShuffleUtil.setOutEntry(dash, "At Setpoint", pidController.atSetpoint());
        ShuffleUtil.setOutEntry(dash, "Manual Override", pidEnabled);
    }

    public void resetPidConstants() {
        pidController.setP(kP);
        pidController.setI(kI);
        pidController.setD(kD);
    }

    @Override
    public void periodic() {
        tickPid();
//        displayShuffleboard();
    }
}
