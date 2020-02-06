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

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import org.rivierarobotics.subsystems.CheeseWheel;
import org.rivierarobotics.subsystems.Ejector;
import org.rivierarobotics.subsystems.Flywheel;

import javax.inject.Inject;

public class CWShootAll extends SequentialCommandGroup {
    private final Flywheel flywheel;
    private final CheeseWheelCommands cheeseCommands;
    private final Ejector ejector;

    @Inject
    public CWShootAll(CheeseWheelCommands cheeseCommands, Ejector ejector, Flywheel flywheel) {
        this.flywheel = flywheel;
        this.cheeseCommands = cheeseCommands;
        this.ejector = ejector;
        addRequirements(flywheel, ejector);

        for (int i = 0; i < 5; i++) {
            addCommands(cheeseCommands.advanceIndex(), new WaitCommand(1));
        }
    }

    @Override
    public void initialize() {
        addCommands(cheeseCommands.setClosestHalfIndex());
        ejector.setPower(1.0);
        flywheel.setPower(1.0);
        super.initialize();
    }

    @Override
    public void end(boolean interrupted) {
        ejector.setPower(0.0);
        flywheel.setPower(0.0);
        super.end(interrupted);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
