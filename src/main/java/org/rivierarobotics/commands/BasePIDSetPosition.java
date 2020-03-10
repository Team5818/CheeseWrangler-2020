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

package org.rivierarobotics.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import org.rivierarobotics.subsystems.BasePIDSubsystem;

public class BasePIDSetPosition<T extends BasePIDSubsystem> extends CommandBase {
    protected final T subsystem;
    protected double positionTicks;
    protected final double maxErrorTicks;
    private final double posTemp;
    private final double timeout;
    private double start;

    // Measurements do not need to be in ticks if get/set methods are overridden, but are in ticks by default
    public BasePIDSetPosition(T subsystem, double maxErrorTicks, double positionTicks, double timeout) {
        this.subsystem = subsystem;
        this.maxErrorTicks = maxErrorTicks;
        posTemp = positionTicks;
        this.timeout = timeout;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        setSetPosition(posTemp);
        setPositionTicks(positionTicks);
        start = Timer.getFPGATimestamp();
    }

    @Override
    public boolean isFinished() {
        double err = Math.abs(getPositionTicks() - positionTicks);
        boolean isInError = err < maxErrorTicks;
        SmartDashboard.putBoolean(subsystem.getName() + " isWithinError", isInError);
        return (isInError && Timer.getFPGATimestamp() - start > 0.2) || (Timer.getFPGATimestamp() - start) > timeout;
    }

    protected double getPositionTicks() {
        return subsystem.getPositionTicks();
    }

    protected void setPositionTicks(double position) {
        subsystem.setPositionTicks(position);
    }

    protected void setSetPosition(double position) {
        this.positionTicks = position;
    }
}
