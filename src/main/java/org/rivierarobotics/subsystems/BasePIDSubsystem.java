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
    protected final SubsystemShuffleTab shuffleTab;
    protected final PIDController pidController;
    protected final PIDConfig pidConfig;
    protected final double anglesOrInchesToTicks;
    protected boolean pidEnabled = false;

    public BasePIDSubsystem(PIDConfig pidConfig) {
        this(pidConfig, 4096.0 / 360);
    }

    public BasePIDSubsystem(PIDConfig pidConfig, double anglesOrInchesToTicks) {
        this.pidConfig = pidConfig;
        this.anglesOrInchesToTicks = anglesOrInchesToTicks;

        this.shuffleTab = new SubsystemShuffleTab(getName());
        this.pidController = new PIDController(pidConfig.getP(), pidConfig.getI(), pidConfig.getD(), 0.005);
        this.pidController.setTolerance(pidConfig.getTolerance());
    }

    private void tickPid() {
        double pidPower = Math.min(pidConfig.getRange(), Math.max(-pidConfig.getRange(), pidController.calculate(getPositionTicks())));
        if (pidEnabled) {
            if (Math.abs(pidController.getSetpoint() - getPositionTicks()) < pidConfig.getTolerance()) {
                return;
            }
            SmartDashboard.putNumber("InitPID", pidPower);
            if (Math.abs(pidPower) < pidConfig.getF() && pidPower != 0 && pidConfig.getF() != 0) {
                if (pidPower < 0) {
                    setPower(-pidConfig.getF());
                } else if (pidPower > 0) {
                    setPower(pidConfig.getF());
                }
            } else {
                setPower(pidPower);
            }
        }
    }

    public final PIDController getPidController() {
        return pidController;
    }

    public final PIDConfig getPidConfig() {
        return pidConfig;
    }

    public final double getAnglesOrInchesToTicks() {
        return anglesOrInchesToTicks;
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
        shuffleTab.setEntry("Position", getPosition());
        shuffleTab.setEntry("Position Ticks", getPositionTicks());
        shuffleTab.setEntry("Setpoint", pidController.getSetpoint());
        shuffleTab.setEntry("At Setpoint", pidController.atSetpoint());
        shuffleTab.setEntry("PID Enabled", pidEnabled);
        shuffleTab.setEntry("Error", pidController.getSetpoint() - Math.abs(getPosition()));
    }

    public void resetPidConstants() {
        pidController.setPID(pidConfig.getP(), pidConfig.getI(), pidConfig.getD());
    }

    @Override
    public void periodic() {
        tickPid();
        // displayShuffleboard();
    }
}
