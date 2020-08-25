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

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import org.rivierarobotics.commands.cheesewheel.CheeseWheelControl;
import org.rivierarobotics.util.CheeseSlot;
import org.rivierarobotics.util.MathUtil;
import org.rivierarobotics.util.RobotShuffleboard;
import org.rivierarobotics.util.RobotShuffleboardTab;

import javax.inject.Provider;

public class CheeseWheel extends BasePIDSubsystem implements RRSubsystem {
    private final WPI_TalonSRX wheelTalon;
    private final Provider<CheeseWheelControl> command;
    private static final int TICKS_AT_ZERO_DEGREES = 3725;
    private static final double INDEX_SPACING = 4096.0 / 5;
    private final RobotShuffleboardTab tab;

    public CheeseWheel(int motor, Provider<CheeseWheelControl> command, RobotShuffleboard shuffleboard) {
        super(new PIDConfig(0.002, 0.0, 0.0001, 0.0, 30, 1.0));
        this.wheelTalon = new WPI_TalonSRX(motor);
        this.command = command;
        this.tab = shuffleboard.getTab("Cheese Wheel");
        wheelTalon.configFactoryDefault();
        wheelTalon.setNeutralMode(NeutralMode.Brake);
    }

    public CheeseSlot getClosestSlot(AngleOffset offset, Direction direction, boolean requireOpen) {
        CheeseSlot min = CheeseSlot.values()[0];
        for (CheeseSlot slot : CheeseSlot.values()) {
            double posDiff = offset.cwTicks() - getTickPosOfSlot(slot);
            boolean passDir;
            switch (direction) {
                case FORWARDS:
                    passDir = posDiff > 0;
                    break;
                case BACKWARDS:
                    passDir = posDiff < 0;
                    break;
                case ANY:
                    passDir = true;
                    break;
                default:
                    throw new IllegalArgumentException("Cannot handle this direction");
            }
            if ((!requireOpen || slot.hasBall()) && passDir && Math.abs(posDiff) < Math.abs(offset.cwTicks() - getTickPosOfSlot(min))) {
                min = slot;
            }
        }
        return min;
    }

    public boolean onSlot(AngleOffset offset) {
        return MathUtil.isWithinTolerance(getPositionTicks() - TICKS_AT_ZERO_DEGREES, getTickPosOfSlot(getClosestSlot(offset, Direction.ANY, false)), 15);
    }

    public double getTickPosOfSlot(CheeseSlot slot) {
        double slotPos = getPositionTicks() + (slot.ordinal() * INDEX_SPACING) - TICKS_AT_ZERO_DEGREES;
        tab.setEntry("slotPos", slotPos + " FOR " + slot.ordinal());
        return slotPos;
    }

    @Override
    public void setPower(double pwr) {
        logger.powerChange(pwr);
        wheelTalon.set(pwr);
    }

    @Override
    public double getPositionTicks() {
        return wheelTalon.getSensorCollection().getPulseWidthPosition();
    }

    public enum AngleOffset {
        SHOOTING(36), COLLECT_FRONT(106), COLLECT_BACK(253);

        public final int angle;

        AngleOffset(int angle) {
            this.angle = angle;
        }

        public double cwTicks() {
            return (this.angle * 4096 / 360.0) - TICKS_AT_ZERO_DEGREES;
        }

        public Direction getAssocDir() {
            switch (this) {
                case COLLECT_FRONT:
                    return Direction.FORWARDS;
                case COLLECT_BACK:
                    return Direction.BACKWARDS;
                default:
                    throw new IllegalArgumentException("Invalid assoc direction");
            }
        }
    }

    public enum Direction {
        FORWARDS, BACKWARDS, ANY
    }

    @Override
    public void periodic() {
        if (getDefaultCommand() == null) {
            setDefaultCommand(command.get());
        }
        super.periodic();
    }
}
