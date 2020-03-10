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

package org.rivierarobotics.commands.shooting;

import edu.wpi.first.wpilibj2.command.CommandBase;
import net.octyl.aptcreator.GenerateCreator;
import net.octyl.aptcreator.Provided;
import org.rivierarobotics.commands.cheesewheel.CheeseWheelCommands;
import org.rivierarobotics.commands.ejector.EjectorCommands;
import org.rivierarobotics.subsystems.CheeseWheel;
import org.rivierarobotics.subsystems.Ejector;

@GenerateCreator
public class NiceShootinTex extends CommandBase {
    private final CheeseWheel cheeseWheel;
    private final EjectorCommands ejectorCommands;
    private final int slots;
    private CheeseWheelCommands cheeseWheelCommands;
    private int shotSlots;
    private final Ejector ejector;

    public NiceShootinTex(@Provided CheeseWheel cheeseWheel,
                          @Provided CheeseWheelCommands cheeseWheelCommands,
                          @Provided Ejector ejector,
                          @Provided EjectorCommands ejectorCommands, int slots) {
        this.cheeseWheel = cheeseWheel;
        this.ejector = ejector;
        this.cheeseWheelCommands = cheeseWheelCommands;
        this.ejectorCommands = ejectorCommands;
        this.slots = slots;
    }

    @Override
    public void initialize() {
        moveToNext();
    }

    @Override
    public void execute() {
        if (!cheeseWheel.getPidController().atSetpoint()) {
            return;
        }
        shotSlots++;
        if (!isFinished()) {
            moveToNext();
        }
    }

    private void moveToNext() {
        cheeseWheelCommands.moveToNextIndex(-1, CheeseWheel.AngleOffset.SHOOTING).schedule();
    }

    @Override
    public void end(boolean interrupted) {
        ejectorCommands.setPower(0).schedule();
    }

    @Override
    public boolean isFinished() {
        return shotSlots == slots;
    }
}
