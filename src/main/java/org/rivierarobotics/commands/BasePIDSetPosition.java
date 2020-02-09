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

package org.rivierarobotics.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import org.rivierarobotics.subsystems.BasePIDSubsystem;

import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;

public class BasePIDSetPosition extends CommandBase {
    private final BasePIDSubsystem subsystem;
    private final DoubleSupplier getPosition;
    private final DoubleConsumer setPosition;
    private final double position, maxError;

    public BasePIDSetPosition(BasePIDSubsystem subsystem, double maxError, double position) {
        this(subsystem, subsystem::getPosition, subsystem::setPosition, maxError, position);
    }

    public BasePIDSetPosition(BasePIDSubsystem subsystem, DoubleSupplier getPosition, DoubleConsumer setPosition, double maxError, double position) {
        this.subsystem = subsystem;
        this.getPosition = getPosition;
        this.setPosition = setPosition;
        this.maxError = maxError;
        this.position = position;
        addRequirements(subsystem);
    }

    @Override
    public void execute() {
        setPosition.accept(position);
    }

    @Override
    public boolean isFinished() {
        double err = Math.abs(getPosition.getAsDouble() - position);
        boolean isInError = err < maxError;
        SmartDashboard.putBoolean(subsystem.getName() + " isWithinError", isInError);
        return isInError;
    }
}
