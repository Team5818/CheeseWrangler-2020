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
import org.rivierarobotics.util.Reporting;

public abstract class BasePID {
    private final double pidRange, anglesOrInchesToTicks;
    private final ShuffleboardTab display;
    private boolean manualOverride = true;
    private PIDController pidController;

    public BasePID(double kP, double kI, double kD, double pidRange, double tolerance) {
        this(kP, kI, kD, pidRange, tolerance, 4096.0 / 360);
    }

    public BasePID(double kP, double kI, double kD, double pidRange, double tolerance, double anglesOrInchesToTicks) {
        this.pidController = new PIDController(kP, kI, kD);
        this.pidRange = pidRange;
        this.display = Shuffleboard.getTab(this.getClass().getSimpleName());
        this.anglesOrInchesToTicks = anglesOrInchesToTicks;
        //TODO figure out what tolerance values work, or if zero is good with just some PID tuning
        // remove the tolerance parameter if zero is fine (set to 0 by default)
        pidController.setTolerance(tolerance);

        //TODO test if both types of statistic reporting work for all subsystems
        display.addNumber("Position Ticks", this::getPositionTicks);
        display.addNumber("Position Degrees/Inches", this::getPosition);
        display.addNumber("Setpoint", pidController::getSetpoint);
        display.addBoolean("At Setpoint", pidController::atSetpoint);
    }

    public void tickPid() {
        double pidPower = Math.min(pidRange, Math.max(-pidRange, pidController.calculate(getPositionTicks())));
        //TODO test if manual override works (any manual js overrides/resets setpoint, no jittering)
        if (manualOverride) {
            setPosition(getPosition());
        } else {
            Reporting.setOutEntry(display, "PID Power", pidPower);
            setPower(pidPower);
        }
    }

    public final PIDController getPidController() {
        return pidController;
    }

    public double getPosition() {
        return getPositionTicks() / anglesOrInchesToTicks;
    }

    public void setPosition(double position) {
        pidController.setSetpoint(position * anglesOrInchesToTicks);
    }

    public void setManualPower(double pwr) {
        manualOverride = (pwr != 0);
        Reporting.setOutEntry(display, "Manual Power", pwr);
        Reporting.setOutEntry(display, "Manual Override", manualOverride);
        setPower(pwr);
    }

    public abstract double getPositionTicks();

    protected abstract void setPower(double pwr);
}
