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
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import net.octyl.aptcreator.GenerateCreator;
import net.octyl.aptcreator.Provided;
import org.rivierarobotics.commands.cheesewheel.CheeseWheelCommands;
import org.rivierarobotics.commands.ejector.EjectorCommands;
import org.rivierarobotics.subsystems.CheeseWheel;
import org.rivierarobotics.subsystems.Flywheel;
import org.rivierarobotics.subsystems.Turret;
import org.rivierarobotics.util.CheeseSlot;
import org.rivierarobotics.util.MathUtil;
import org.rivierarobotics.util.RSTab;
import org.rivierarobotics.util.RobotShuffleboard;

/**
 * Shoot a single ball. Waits for AutoAim to be within tolerance before
 * shooting. Designed to be called continuously (button held).
 */
@GenerateCreator
public class ContinuousShoot extends CommandBase {
    private final CheeseWheelCommands cheeseWheelCommands;
    private final EjectorCommands ejectorCommands;
    private final CheeseWheel cheeseWheel;
    private final Turret turret;
    private final Flywheel flywheel;
    private final RSTab tab;
    private ParallelRaceGroup cmd;

    public ContinuousShoot(@Provided CheeseWheelCommands cheeseWheelCommands,
                           @Provided EjectorCommands ejectorCommands,
                           @Provided Turret turret,
                           @Provided CheeseWheel cheeseWheel,
                           @Provided Flywheel flywheel,
                           @Provided RobotShuffleboard shuffleboard) {
        this.cheeseWheelCommands = cheeseWheelCommands;
        this.ejectorCommands = ejectorCommands;
        this.turret = turret;
        this.cheeseWheel = cheeseWheel;
        this.flywheel = flywheel;
        this.tab = shuffleboard.getTab("Cheese Wheel");
    }

    @Override
    public void initialize() {
        boolean isBack = MathUtil.isWithinTolerance(turret.getAngle(false), 0, 90);
        CheeseWheel.AngleOffset offset = isBack ? CheeseWheel.AngleOffset.SHOOTER_BACK : CheeseWheel.AngleOffset.SHOOTER_FRONT;
        CheeseSlot slot = cheeseWheel.getClosestSlot(offset, offset.direction, CheeseSlot.State.BALL);
        tab.setEntry("ccShootSlot", slot.ordinal());

        if (flywheel.isBelowMinShootingVel()) {
            cmd = new SequentialCommandGroup(
                    cheeseWheelCommands.cycleSlotWait(offset.direction, offset, CheeseSlot.State.BALL, 50).withTimeout(3),
                    new WaitUntilCommand(flywheel::withinTolerance),
                    ejectorCommands.setPower(1),
                    new WaitUntilCommand(() -> !slot.hasBall()).andThen(new WaitCommand(0.1)).withTimeout(3),
                    ejectorCommands.setPower(-0.1),
                    new WaitCommand(0.1),
                    ejectorCommands.setPower(0)
            ).withTimeout(2);
            cmd.schedule();
        }
    }

    @Override
    public boolean isFinished() {
        boolean shootFinished = cmd != null && !CommandScheduler.getInstance().isScheduled(cmd);
        tab.setEntry("shootFinished", shootFinished);
        return shootFinished;
    }
}
