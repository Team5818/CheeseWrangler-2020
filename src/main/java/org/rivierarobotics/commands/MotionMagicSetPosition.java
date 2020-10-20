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
import org.rivierarobotics.util.RobotShuffleboard;
import org.rivierarobotics.util.RobotShuffleboardTab;

import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class MotionMagicSetPosition<T extends SubsystemBase & RRSubsystem> extends CommandBase {
    protected final T subsystem;
    protected final DoubleSupplier getPosition;
    protected final DoubleConsumer setPosition;
    protected final Supplier<double[]> getSoftLimits;
    protected final double maxError;
    protected final double setPoint;
    protected final double timeout;
    protected final RobotShuffleboardTab tab;
    private double start;

    public MotionMagicSetPosition(T subsystem, DoubleSupplier getPosition, DoubleConsumer setPosition,
                                  double setPoint, double maxError, double timeout, RobotShuffleboard shuffleboard) {
        this.subsystem = subsystem;
        this.getPosition = getPosition;
        this.setPosition = setPosition;
        this.maxError = maxError;
        this.setPoint = setPoint;
        this.timeout = timeout;
        this.getSoftLimits = null;
        tab = shuffleboard.getTab("MM Tuning");
        tab.setEntry(subsystem.getName() + " setPoint:", setPoint);
        tab.setEntry(subsystem.getName() + " maxErrorTicks:", maxError);
        addRequirements(subsystem);
    }

    public MotionMagicSetPosition(T subsystem, DoubleSupplier getPosition, DoubleConsumer setPosition, Supplier<double[]> getSoftLimits,
                                  double setPoint, double maxError, double timeout, RobotShuffleboard shuffleboard) {
        this.subsystem = subsystem;
        this.getPosition = getPosition;
        this.setPosition = setPosition;
        this.maxError = maxError;
        this.setPoint = setPoint;
        this.getSoftLimits = getSoftLimits;
        this.timeout = timeout;
        tab = shuffleboard.getTab("MM Tuning");
        tab.setEntry(subsystem.getName() + " setPoint:", setPoint);
        tab.setEntry(subsystem.getName() + " maxErrorTicks:", maxError);
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        setPosition.accept(setPoint);
        start = Timer.getFPGATimestamp();
    }

    @Override
    public boolean isFinished() {
        tab.setEntry(subsystem.getName() + " currentPosition", getPosition.getAsDouble());
        boolean finished = MathUtil.isWithinTolerance(getPosition.getAsDouble(), setPoint, maxError) || (Timer.getFPGATimestamp() - start) > timeout;
        if (getSoftLimits != null) {
            finished = finished || subsystem.getPositionTicks() >= getSoftLimits.get()[0] || subsystem.getPositionTicks() <= getSoftLimits.get()[1];
        }
        tab.setEntry(subsystem.getName() + " finished", finished);
        return finished;
    }
}
