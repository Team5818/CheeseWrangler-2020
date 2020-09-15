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
    private static final int TICKS_AT_ZERO_DEGREES = -369;
    private static final double INDEX_SPACING = 4096.0 / 5;
    private final RobotShuffleboardTab tab;

    public CheeseWheel(int motor, Provider<CheeseWheelControl> command, RobotShuffleboard shuffleboard) {
        super(new PIDConfig(0.00075, 0.0, 0.000, 0.0, 22, 1.0));
        this.wheelTalon = new WPI_TalonSRX(motor);
        this.command = command;
        this.tab = shuffleboard.getTab("Cheese Wheel");
        wheelTalon.configFactoryDefault();
        wheelTalon.setNeutralMode(NeutralMode.Brake);
    }

    public double getPositionTicksWithOffset(AngleOffset offset) {
        return getPositionTicks() - offset.getAssocCWTicks();
    }

    public int getIndex(AngleOffset offset) {
        int min = 360;
        double minAngle = 360;
        for (int i = 0; i <= 5; i++) {
            double ang = Math.abs(getPositionTicksWithOffset(offset)  - (i * INDEX_SPACING));
            if (minAngle > ang) {
                minAngle = ang;
                if (i == 5) {
                    return 0;
                }
                min = i;
            }
        }
        return min;
    }


    // This should be the only thing actually taking a direction
    public CheeseSlot getClosestSlot(AngleOffset offset, Direction direction, boolean requireOpen) {
        CheeseSlot min = CheeseSlot.values()[0];
        for (CheeseSlot slot : CheeseSlot.values()) {
            double posDiff = getSlotTickPos(slot) - getPositionTicksWithOffset(offset);
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
            if ((!requireOpen || slot.hasBall()) && getIndex(offset) != slot.ordinal() && passDir
                && Math.abs(posDiff) < Math.abs(getSlotTickPos(min) - getPositionTicks())) {
                min = slot;
            }
        }
        return min;
    }

    public boolean onSlot(AngleOffset offset) {
        return onSlot(offset, 15);
    }

    public boolean onSlot(AngleOffset offset, double tolerance) {
        return MathUtil.isWithinTolerance(getPositionTicks() - TICKS_AT_ZERO_DEGREES,
            getSlotTickPos(getClosestSlot(offset, Direction.ANY, false)), tolerance);
    }

    public double getSlotTickPos(CheeseSlot slot) {
        return getSlotTickPos(slot, AngleOffset.SHOOTER);
    }

    //Direction is based off of getClosestSlot, Offset is necessary to return the correct values.
    public double getSlotTickPos(CheeseSlot slot, AngleOffset offset) {
        int index = slot.ordinal();
        if(getIndex(offset) == 4 && slot.ordinal() == 0) {
            index = 5;
        } else if(getIndex(offset) == 0 && slot.ordinal() == 4) {
            index = -1;
        }
        tab.setEntry("tickpos", index * INDEX_SPACING - ((getPositionTicksWithOffset(offset)) % 4096.0) + getPositionTicks() + " FOR " + slot.ordinal());
        return  index * INDEX_SPACING - ((getPositionTicksWithOffset(offset)) % 4096.0) + getPositionTicks();

//        switch (direction) {
//            case FORWARDS:
//                cwPos += slotPos;
//                break;
//            case BACKWARDS:
//                cwPos -= slotPos;
//                break;
//            case ANY:
//            default:
//                double modPos = cwPos % 4096.0;
//                double add = Math.abs(modPos + slotPos);
//                double subtr = Math.abs(modPos - slotPos);
//                cwPos += (((subtr > add) ? -1 : 1) * slotPos);
//                break;
//        }
//
//        return cwPos;
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
        //TODO change angle offsets to new values (0-ctr)
        COLLECT_FRONT(0, Direction.FORWARDS), COLLECT_BACK(0, Direction.BACKWARDS), SHOOTER(0,Direction.ANY);

        public final int angle;
        public final Direction direction;

        AngleOffset(int angle, Direction direction) {
            this.angle = angle;
            this.direction = direction;
        }

        //Has to add zero ticks because just imagine putting 0 into this. You get -some amount of ticks which then need
        //to be added to getPositionTicks which we don't do anywhere in the code. also then we'd need to make everything
        //negative as well because it assumes zeroticks and the offset are in different directions.
        // now 40 degrees offset is just 0 degrees + the angle offset. fits better with the code.
        public double getAssocCWTicks() {
            return (this.angle * 4096 / 360.0) + TICKS_AT_ZERO_DEGREES;
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
