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

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import net.octyl.aptcreator.GenerateCreator;
import net.octyl.aptcreator.Provided;
import org.rivierarobotics.commands.cheesewheel.CheeseWheelCommands;
import org.rivierarobotics.commands.ejector.EjectorCommands;
import org.rivierarobotics.subsystems.CheeseWheel;
import org.rivierarobotics.subsystems.Ejector;
import org.rivierarobotics.util.CheeseSlot;
import org.rivierarobotics.util.MathUtil;
import org.rivierarobotics.util.PhysicsUtil;

/**
 * Shoots all stored balls (up to five) sequentially. Rotates one time with
 * 1000 tick forward tolerance. Use other shooting commands if possible. Does
 * not use a PID - dead reckoning only. May jam due to quick rotation.
 *
 * @see ShootNWedges
 * @see ContinuousShoot
 */
@GenerateCreator
public class All5Shoot extends CommandBase {
    private final CheeseWheelCommands cheeseWheelCommands;
    private final EjectorCommands ejectorCommands;
    private Command shoot5;
    private final CheeseWheel cheeseWheel;
    private boolean isFinished = false;
    private double time;

    public All5Shoot(@Provided CheeseWheelCommands cheeseWheelCommands, @Provided EjectorCommands ejectorCommands, @Provided CheeseWheel cheeseWheel) {
        this.cheeseWheelCommands = cheeseWheelCommands;
        this.ejectorCommands = ejectorCommands;
        this.cheeseWheel = cheeseWheel;
    }

    @Override
    public void initialize() {
        PhysicsUtil.dynamicMode = true;
        shoot5 = new SequentialCommandGroup(
                cheeseWheelCommands.cycleSlotWait(CheeseWheel.Direction.ANY, CheeseWheel.AngleOffset.SHOOTER_BACK, CheeseSlot.State.EITHER, 60),
                new WaitCommand(0.2),
                new ParallelDeadlineGroup(
                        cheeseWheelCommands.setPower(0.8).withTimeout(3),
                        ejectorCommands.setPower(1).withTimeout(3)
                ),
                ejectorCommands.setPower(0)
        );
        CommandScheduler.getInstance().schedule(shoot5);
    }

    @Override
    public void end(boolean interrupted) {
        if (CommandScheduler.getInstance().isScheduled(shoot5)) {
            CommandScheduler.getInstance().cancel(shoot5);
        }
        PhysicsUtil.dynamicMode = false;
        isFinished = false;
    }

    @Override
    public boolean isFinished() {
        if(!cheeseWheel.hasBall() && !isFinished) {
            isFinished = true;
            time = Timer.getFPGATimestamp();
        }

        if(isFinished && Timer.getFPGATimestamp() > time + 0.1) {
            return true;
        }

        return !CommandScheduler.getInstance().isScheduled(shoot5);
    }
}
