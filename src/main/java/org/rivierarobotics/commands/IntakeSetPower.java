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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import net.octyl.aptcreator.GenerateCreator;
import net.octyl.aptcreator.Provided;
import org.rivierarobotics.subsystems.CheeseWheel;
import org.rivierarobotics.subsystems.Intake;
import org.rivierarobotics.util.Side;

import java.util.function.BooleanSupplier;

@GenerateCreator
public class IntakeSetPower extends CommandBase {
    private final Intake intake;
    private final CheeseWheel cheeseWheel;
    private final CheeseWheelCommands cheeseWheelCommands;
    private final double frontPower;
    private final double backPower;
    private static final double pwrConstant = 1.0;
    private Command goingToNext;
    private final BooleanSupplier hasBall;
    private final CheeseWheel.Mode mode;
    private final CheeseWheel.Mode fixMode;

    IntakeSetPower(@Provided Intake intake, @Provided CheeseWheel cheeseWheel,
                   @Provided CheeseWheelCommands cheeseWheelCommands, Side side) {
        this.intake = intake;
        this.cheeseWheel = cheeseWheel;
        this.cheeseWheelCommands = cheeseWheelCommands;
        if (side == Side.FRONT) {
            frontPower = pwrConstant;
            backPower = 0.0;
            hasBall = cheeseWheel.getSensors()::isFrontBallPresent;
            mode = CheeseWheel.Mode.COLLECT_FRONT;
            fixMode = CheeseWheel.Mode.FIX_FRONT;
        } else {
            if (side != Side.BACK) {
                throw new IllegalArgumentException("Invalid side: " + side);
            }
            frontPower = 0.0;
            backPower = pwrConstant;
            hasBall = cheeseWheel.getSensors()::isBackBallPresent;
            mode = CheeseWheel.Mode.COLLECT_BACK;
            fixMode = CheeseWheel.Mode.FIX_BACK;
        }
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        cheeseWheel.getClosestSlot(mode, CheeseWheel.Filled.NO);
        goingToNext = cheeseWheelCommands.moveToFreeIndex(mode, CheeseWheel.Filled.NO);
        goingToNext.schedule();
    }

    @Override
    public void execute() {
        intake.setPower(frontPower, backPower);
        if (goingToNext != null && goingToNext.isScheduled()) {
            return;
        }
        if (hasBall.getAsBoolean()) {
            SmartDashboard.putString("schedulin", "iz schedule");
            var slot = cheeseWheel.getClosestSlot(mode, CheeseWheel.Filled.NO);
            slot.isFilled = true;
            goingToNext = cheeseWheelCommands.setIndex(fixMode, slot)
                .andThen(cheeseWheelCommands.moveToFreeIndex(mode, CheeseWheel.Filled.NO));
            goingToNext.schedule();
        } else {
            SmartDashboard.putString("schedulin", "CAN HAZ BALL???");
        }
    }

    @Override
    public void end(boolean interrupted) {
        intake.setPower(0, 0);
    }
}
