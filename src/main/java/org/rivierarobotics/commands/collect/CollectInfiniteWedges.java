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

package org.rivierarobotics.commands.collect;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import net.octyl.aptcreator.GenerateCreator;
import net.octyl.aptcreator.Provided;
import org.rivierarobotics.commands.cheesewheel.CWCycleSlot;
import org.rivierarobotics.commands.cheesewheel.CheeseWheelCommands;
import org.rivierarobotics.subsystems.CheeseWheel;
import org.rivierarobotics.subsystems.Intake;
import org.rivierarobotics.util.CheeseSlot;
import org.rivierarobotics.util.RSTab;
import org.rivierarobotics.util.RobotShuffleboard;

/**
 * Collect balls using intake/tentacles on one side. Stops after CheeseWheel
 * is filled (x5). Moves CheeseWheel to safe intake position before starting
 * collection process. Intended to be held while being used. Uses CWCycleSlot
 * command calls to perform movement actions. Does not support dual-sided
 * collection. Variable intake power and slot tolerance.
 *
 * @see CWCycleSlot
 * @see CheeseWheel.AngleOffset
 */
@GenerateCreator
public class CollectInfiniteWedges extends CommandBase {
    private static final double PWR_CONST = 1.0;
    private static final int SLOT_TOLERANCE = 60;
    private final Intake intake;
    private final CheeseWheel cheeseWheel;
    private final CheeseWheelCommands cheeseWheelCommands;
    private final RSTab tab;
    private final CheeseWheel.AngleOffset mode;
    private double frontPower;
    private double backPower;
    private boolean firstMoveDone = false;
    private CWCycleSlot cycleSlot;

    CollectInfiniteWedges(@Provided Intake intake, @Provided CheeseWheel cheeseWheel,
                          @Provided CheeseWheelCommands cheeseWheelCommands,
                          @Provided RobotShuffleboard shuffleboard,
                          CheeseWheel.AngleOffset mode) {
        this.intake = intake;
        this.cheeseWheel = cheeseWheel;
        this.cheeseWheelCommands = cheeseWheelCommands;
        this.tab = shuffleboard.getTab("Cheese Wheel");
        this.mode = mode;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        if (!cheeseWheel.onSlot(mode, SLOT_TOLERANCE) || CheeseSlot.slotOfNum(cheeseWheel.getIndex(mode)).hasBall()) {
            cycleSlot = !isFull() ? cheeseWheelCommands.cycleSlotWait(mode.direction, mode, CheeseSlot.State.NO_BALL, 30) :
                    cheeseWheelCommands.cycleSlotWait(mode.direction, mode, CheeseSlot.State.EITHER, 30);
            cycleSlot.schedule();
        }
    }

    @Override
    public void execute() {
        if (cycleSlot == null || !CommandScheduler.getInstance().isScheduled(cycleSlot) || cheeseWheel.onSlot(mode, SLOT_TOLERANCE)) {
            firstMoveDone = true;
        }

        if (firstMoveDone) {
            switch (mode) {
                case COLLECT_FRONT:
                    frontPower = PWR_CONST;
                    backPower = 0.0;
                    break;
                case COLLECT_BACK:
                    frontPower = 0.0;
                    backPower = PWR_CONST;
                    break;
                default:
                    throw new IllegalArgumentException("Invalid side");
            }
        }

        if (isFull() || !firstMoveDone) {
            frontPower = 0;
            backPower = 0;
        }

        intake.setPower(frontPower, backPower);

        int index = cheeseWheel.getIndex(mode);
        CheeseSlot closest = CheeseSlot.slotOfNum(index);
        tab.setEntry("ClosestIndex", closest.ordinal());
        tab.setEntry("ClosestHasBall", closest.hasBall());
        if (closest.hasBall() && cheeseWheel.onSlot(mode, SLOT_TOLERANCE) && !isFull()) {
            cheeseWheelCommands.cycleSlot(mode.direction, mode, CheeseSlot.State.NO_BALL).schedule();
        }
    }

    private boolean isFull() {
        for (CheeseSlot slot : CheeseSlot.values()) {
            if (!slot.hasBall()) {
                return false;
            }
        }
        return true;
    }

    @Override
    public void end(boolean interrupted) {
        intake.setPower(0, 0);
    }
}