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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class BasePIDSubsystem extends SubsystemBase {
    private final double pidRange, anglesOrInchesToTicks;
    private PIDController pidController;

    public BasePIDSubsystem(double kP, double kI, double kD, double pidRange, double anglesOrInchesToTicks) {
        this.pidController = new PIDController(kP, kI, kD);
        this.pidRange = pidRange;
        this.anglesOrInchesToTicks = anglesOrInchesToTicks;
    }

    public void tickPid() {
        double pwr = pidController.calculate(getPositionTicks());
        double realPower = Math.min(pidRange, Math.max(-pidRange, pwr));
        SmartDashboard.putNumber("Real Powah: " + getName(), realPower);
        setPower(realPower);
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

    public abstract double getPositionTicks();

    public abstract void setPower(double pwr);
}
