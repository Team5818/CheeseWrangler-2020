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
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.rivierarobotics.subsystems.RRSubsystem;
import org.rivierarobotics.util.MathUtil;

import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;

public class MotionMagicSetPosition<T extends SubsystemBase & RRSubsystem> extends CommandBase {
    protected final T subsystem;
    protected final DoubleSupplier getPosition;
    protected final DoubleConsumer setPosition;
    protected final double maxErrorTicks;
    protected final double setpoint;
    protected final double timeout;
    private double start;

    // Measurements do not need to be in ticks if get/set methods are overridden, but are in ticks by default
    public MotionMagicSetPosition(T subsystem, DoubleSupplier getPosition, DoubleConsumer setPosition,
                                  double setpoint, double maxErrorTicks, double timeout) {
        this.subsystem = subsystem;
        this.getPosition = getPosition;
        this.setPosition = setPosition;
        this.maxErrorTicks = maxErrorTicks;
        this.setpoint = setpoint;
        this.timeout = timeout;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        setPosition.accept(setpoint);
        start = Timer.getFPGATimestamp();
    }

    @Override
    public boolean isFinished() {
        return MathUtil.isWithinTolerance(getPosition.getAsDouble(), setpoint, maxErrorTicks)
            || (Timer.getFPGATimestamp() - start) > timeout;
    }
}
