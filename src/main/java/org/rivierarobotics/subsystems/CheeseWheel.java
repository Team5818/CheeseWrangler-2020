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
    private static final int TICKS_AT_ZERO_DEGREES = 48;
    private static final double INDEX_SPACING = 4096.0 / 5;
    private final RobotShuffleboardTab tab;

    public CheeseWheel(int motor, Provider<CheeseWheelControl> command, RobotShuffleboard shuffleboard) {
        super(new PIDConfig(0.001, 0.000022, 0.0000015, 0.0, 20, 1.0));
        this.wheelTalon = new WPI_TalonSRX(motor);
        this.command = command;
        this.tab = shuffleboard.getTab("Cheese Wheel");
        wheelTalon.configFactoryDefault();
        wheelTalon.setNeutralMode(NeutralMode.Brake);
    }

    public double getOffsetPositionTicks(AngleOffset offset) {
        return getPositionTicks() - offset.getAssocCWTicks();
    }

    public int getIndex(AngleOffset offset) {
        double a = MathUtil.wrapToCircle(getOffsetPositionTicks(offset),4096);
        for(int i = 0; i < 5; i++){
            if(MathUtil.isWithinTolerance(INDEX_SPACING * i,a,INDEX_SPACING/2.0)){
                return i;
            }
        }
        return 0;
    }

    public CheeseSlot getSlotWithDirection(AngleOffset offset, Direction direction, CheeseSlot.State requiredState) {
        int modifier = direction == Direction.BACKWARDS ? 1 : -1;
        for (int i = 1; i < 5; i++) {
            CheeseSlot slot = CheeseSlot.slotOfNum((int) MathUtil.wrapToCircle(getIndex(offset) + i * modifier, 5));
            if (requiredState == CheeseSlot.State.EITHER
                    || (requiredState == CheeseSlot.State.BALL && slot.hasBall())
                    || (requiredState == CheeseSlot.State.NO_BALL && !slot.hasBall())) {
                return slot;
            }
        }
        return CheeseSlot.slotOfNum(getIndex(offset));
    }

    public CheeseSlot getClosestSlot(AngleOffset offset, Direction direction, CheeseSlot.State requiredState) {
        if (direction == Direction.ANY) {
            CheeseSlot forward = getSlotWithDirection(offset, Direction.FORWARDS, requiredState);
            CheeseSlot backward = getSlotWithDirection(offset, Direction.BACKWARDS, requiredState);
            return Math.abs(getIndex(offset) - forward.ordinal()) <= Math.abs(getIndex(offset) - backward.ordinal()) ? forward : backward;
        }
        return getSlotWithDirection(offset, direction, requiredState);
    }

    public boolean onSlot(AngleOffset offset, Direction direction, double tolerance) {
        double a = MathUtil.wrapToCircle(getOffsetPositionTicks(offset),4096);
        return MathUtil.isWithinTolerance(a,getIndex(offset) * INDEX_SPACING,tolerance);
    }

    public double getSlotTickPos(CheeseSlot slot) {
        return getSlotTickPos(slot, AngleOffset.COLLECT_FRONT, Direction.ANY);
    }

    public double getSlotTickPos(CheeseSlot slot, AngleOffset offset, Direction direction) {
        int index = slot.ordinal();
        int offsetIndex = getIndex(offset);

        double position = MathUtil.wrapToCircle((getOffsetPositionTicks(offset) % 4096), 4096);

        if (offsetIndex == 4 && index == 0) {
            index = 5;
        } else if (offsetIndex == 0 && index == 4) {
            index = -1;
        }

        //SPECIAL CASE FOR offsetIndex == 0 because of wrapping not working if it is at like 4050 ticks
        if (offsetIndex == 0) {
            if (Math.abs((position % 4096) - 4096) < Math.abs(position)) {
                position -= 4096;
            }
        }

        double outPos = (index * INDEX_SPACING) - position;

        //DIRECTIONAL MODIFIERS
        if (direction == Direction.FORWARDS && outPos > 0) {
            outPos -= 4096;
        } else if (direction == Direction.BACKWARDS && outPos < 0) {
            outPos += 4096;
        }

        outPos += getPositionTicks();

        tab.setEntry("tickpos", outPos + " FOR " + slot.ordinal());
        return outPos ;
    }

    public RobotShuffleboardTab getTab() {
        return tab;
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
        COLLECT_FRONT(835, Direction.FORWARDS), COLLECT_BACK(2458, Direction.BACKWARDS), SHOOTER(3687, Direction.ANY);

        public final int angle;
        public final Direction direction;

        AngleOffset(int angle, Direction direction) {
            this.angle = angle;
            this.direction = direction;
        }

        // Has to add zero ticks because just imagine putting 0 into this. You get -some amount of ticks which then need
        // to be added to getPositionTicks which we don't do anywhere in the code. also then we'd need to make everything
        // negative as well because it assumes zeroticks and the offset are in different directions.
        // now 40 degrees offset is just 0 degrees + the angle offset. fits better with the code.
        public double getAssocCWTicks() {
            return this.angle + TICKS_AT_ZERO_DEGREES;
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
