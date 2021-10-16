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

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.rivierarobotics.commands.cheesewheel.CheeseWheelControl;
import org.rivierarobotics.util.CheeseSlot;
import org.rivierarobotics.util.MathUtil;
import org.rivierarobotics.util.MotorUtil;
import org.rivierarobotics.util.RSTab;
import org.rivierarobotics.util.RobotShuffleboard;

import javax.inject.Provider;

/**
 * Subsystem for the CheeseWheel. Contains five slots, each holding one ball,
 * that rotate with the wheel and are evenly spaced on the wheel. Three I/O
 * positions: collect (front/back) and shoot. Absolute encoder.
 *
 * @see CheeseSlot
 * @see CheeseSlot.State
 * @see CheeseWheel.AngleOffset
 * @see CheeseWheel.Direction
 */
public class CheeseWheel extends SubsystemBase implements RRSubsystem {
    private static final double INDEX_SPACING = 4096.0 / 5;
    private final WPI_TalonSRX wheelTalon;
    private final Provider<CheeseWheelControl> command;
    private final RSTab tab;

    public CheeseWheel(int motor, Provider<CheeseWheelControl> command, RobotShuffleboard shuffleboard) {
        this.wheelTalon = new WPI_TalonSRX(motor);
        this.command = command;
        this.tab = shuffleboard.getTab("Cheese Wheel");
        // Position PID from absolute encoder
        MotorUtil.setupMotionMagic(FeedbackDevice.PulseWidthEncodedPosition,
                new PIDConfig(1023 / 200.0, 0, 0, 0), -1, wheelTalon);
        wheelTalon.setSensorPhase(false);
        // Deactivate limit switches, no soft limits
        wheelTalon.configForwardLimitSwitchSource(LimitSwitchSource.Deactivated, LimitSwitchNormal.NormallyOpen);
        wheelTalon.configReverseLimitSwitchSource(LimitSwitchSource.Deactivated, LimitSwitchNormal.NormallyOpen);
        wheelTalon.setNeutralMode(NeutralMode.Brake);
    }

    /**
     * Gets the position of the CheeseWheel in reference to an angle offset.
     *
     * @param offset the angle offset position reference frame.
     * @return the adjusted/offset position of the Cheese Wheel in ticks.
     */
    public double getOffsetPositionTicks(AngleOffset offset) {
        return getPositionTicks() - offset.getAssocCWTicks();
    }

    /**
     * Gets the ball index (0-4) of the CheeseWheel closest to a certain
     * angle offset position. Indices are wrapped to a circle.
     *
     * @param offset the angle offset closest to the returned index.
     * @return the index of the CheeseWheel closest to offset.
     */
    public int getIndex(AngleOffset offset) {
        double target = MathUtil.wrapToCircle(getOffsetPositionTicks(offset), 4096);
        for (int i = 0; i <= 5; i++) {
            if (MathUtil.isWithinTolerance(INDEX_SPACING * i, target, INDEX_SPACING / 2.0)) {
                return i == 5 ? 0 : i;
            }
        }
        return 0;
    }

    /**
     * Gets the ball slot of the CheeseWheel closest to a certain
     * angle offset position but only in a certain direction and with a
     * certain required fill state.
     *
     * @param offset the angle offset closest to the returned slot.
     * @param direction the required direction from the offset to slot.
     * @param requiredState the fill state required for matching slots.
     * @return the slot matching all param criteria.
     */
    public CheeseSlot getSlotWithDirection(AngleOffset offset, Direction direction, CheeseSlot.State requiredState) {
        int modifier = direction == Direction.BACKWARDS ? 1 : -1;
        for (int i = 0; i < 5; i++) {
            CheeseSlot slot = CheeseSlot.slotOfNum((int) MathUtil.wrapToCircle(getIndex(offset) + i * modifier, 5));
            if (requiredState == CheeseSlot.State.EITHER
                    || (requiredState == CheeseSlot.State.BALL && slot.hasBall())
                    || (requiredState == CheeseSlot.State.NO_BALL && !slot.hasBall())) {
                return slot;
            }
        }
        return CheeseSlot.slotOfNum(getIndex(offset));
    }

    /**
     * Wrapper for {@link #getSlotWithDirection(AngleOffset, Direction, CheeseSlot.State)}
     * that handles any-sided direction. Use this method instead whenever possible.
     *
     * @param offset the angle offset closest to the returned slot.
     * @param direction the required direction from the offset to slot.
     * @param requiredState the fill state required for matching slots.
     * @return the slot matching all param criteria.
     */
    public CheeseSlot getClosestSlot(AngleOffset offset, Direction direction, CheeseSlot.State requiredState) {
        if (direction == Direction.ANY) {
            CheeseSlot forward = getSlotWithDirection(offset, Direction.FORWARDS, requiredState);
            CheeseSlot backward = getSlotWithDirection(offset, Direction.BACKWARDS, requiredState);
            return Math.abs(getIndex(offset) - forward.ordinal()) <= Math.abs(getIndex(offset) - backward.ordinal()) ? forward : backward;
        }
        return getSlotWithDirection(offset, direction, requiredState);
    }

    /**
     * Determine if a slot is present on the Cheese Wheel at a certain angle
     * offset within a tolerance.
     *
     * @param offset the offset to check for a ball.
     * @param tolerance the tolerance around the offset to allow.
     * @return if a ball is within a slot offset within tolerance.
     */
    public boolean onSlot(AngleOffset offset, double tolerance) {
        double value = MathUtil.wrapToCircle(getOffsetPositionTicks(offset), 4096);
        return MathUtil.isWithinTolerance(value, getIndex(offset) * INDEX_SPACING, tolerance);
    }

    /**
     * Get the position of a slot in ticks at an angle offset and in a
     * direction. Used for determining cycle slot tick positions.
     *
     * @param slot the slot to get the position of.
     * @param offset the offset on the wheel for the slot.
     * @param direction the direction to seek in.
     * @return the position of a matching slot in ticks.
     */
    public double getSlotTickPos(CheeseSlot slot, AngleOffset offset, Direction direction) {
        int index = slot.ordinal();
        double position = MathUtil.wrapToCircle((getOffsetPositionTicks(offset)), 4096);

        // Special case for offsetIndex == 0 because wrapping breaks at about 4050 ticks
        if (getIndex(offset) == 0) {
            if (Math.abs((position % 4096) - 4096) < Math.abs(position)) {
                position -= 4096;
            }
        }

        double outPos = (index * INDEX_SPACING) - position;

        // Directional modifiers
        if (direction == Direction.FORWARDS && outPos > 0) {
            outPos -= 4096;
        } else if (direction == Direction.BACKWARDS && outPos < 0) {
            outPos += 4096;
        }

        outPos += getPositionTicks();

        tab.setEntry("SlotTickPos", outPos + " for " + slot.ordinal());
        return outPos;
    }

    public void setPositionTicks(double positionTicks) {
        wheelTalon.set(ControlMode.MotionMagic, positionTicks);
    }

    @Override
    public void setPower(double pwr) {
        wheelTalon.set(ControlMode.PercentOutput, pwr);
    }

    public boolean hasBall() {
        for (int i = 0; i < 5; i++) {
            if (CheeseSlot.slotOfNum(i).hasBall()) {
                return true;
            }
        }
        return false;
    }

    @Override
    public double getPositionTicks() {
        return wheelTalon.getSelectedSensorPosition();
    }

    /**
     * Defines offsets from zero ticks that account for different I/O
     * positions on the Cheese Wheel. Includes shoot and collect, front and
     * back for each position. Also mandates a turn direction.
     */
    public enum AngleOffset {
        COLLECT_FRONT(875, Direction.FORWARDS),
        COLLECT_BACK(2510, Direction.BACKWARDS),
        SHOOTER_FRONT(-350, Direction.BACKWARDS),
        SHOOTER_BACK(-350, Direction.FORWARDS);

        public final int angle;
        public final Direction direction;

        AngleOffset(int angle, Direction direction) {
            this.angle = angle;
            this.direction = direction;
        }

        public double getAssocCWTicks() {
            return this.angle;
        }
    }

    /**
     * Defines the direction that the Cheese Wheel may take. ANY defines a
     * forwards or backwards movement, whichever is shorter.
     */
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
