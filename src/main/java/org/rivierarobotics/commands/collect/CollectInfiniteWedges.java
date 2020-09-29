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
import net.octyl.aptcreator.GenerateCreator;
import net.octyl.aptcreator.Provided;
import org.rivierarobotics.commands.cheesewheel.CheeseWheelCommands;
import org.rivierarobotics.subsystems.CheeseWheel;
import org.rivierarobotics.subsystems.Intake;
import org.rivierarobotics.util.CheeseSlot;

@GenerateCreator
public class CollectInfiniteWedges extends CommandBase {
    private final Intake intake;
    private final CheeseWheel cheeseWheel;
    private final CheeseWheelCommands cheeseWheelCommands;
    private final double frontPower;
    private final double backPower;
    private static final double pwrConstant = 1.0;
    private final CheeseWheel.AngleOffset mode;

    CollectInfiniteWedges(@Provided Intake intake, @Provided CheeseWheel cheeseWheel,
                          @Provided CheeseWheelCommands cheeseWheelCommands,
                          CheeseWheel.AngleOffset mode) {
        this.intake = intake;
        this.cheeseWheel = cheeseWheel;
        this.cheeseWheelCommands = cheeseWheelCommands;
        this.mode = mode;

        switch (mode) {
            case COLLECT_FRONT:
                frontPower = pwrConstant;
                backPower = 0.0;
                break;
            case COLLECT_BACK:
                frontPower = 0.0;
                backPower = pwrConstant;
                break;
            default:
                throw new IllegalArgumentException("Invalid side");
        }

        addRequirements(intake);
    }

    @Override
    public void initialize() {
        if (!cheeseWheel.onSlot(mode, mode.direction, 50)
            || CheeseSlot.slotOfNum(cheeseWheel.getIndex(mode)).hasBall()) {
            moveToNext();
        }
    }

    @Override
    public void execute() {
        intake.setPower(frontPower, backPower);

        int index = cheeseWheel.getIndex(mode);
        CheeseSlot closest = CheeseSlot.slotOfNum(index);
        cheeseWheel.getTab().setEntry("closestIndex", closest.ordinal());
        cheeseWheel.getTab().setEntry("closestHasBall", closest.hasBall());
        if (closest.hasBall()) {
            moveToNext();
        }
    }

    private void moveToNext() {
        cheeseWheelCommands.cycleSlot(mode.direction, mode, CheeseSlot.State.NO_BALL).schedule();
    }

    @Override
    public void end(boolean interrupted) {
        intake.setPower(0, 0);
    }
}
