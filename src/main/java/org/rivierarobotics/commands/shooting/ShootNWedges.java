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

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import net.octyl.aptcreator.GenerateCreator;
import net.octyl.aptcreator.Provided;
import org.rivierarobotics.commands.cheesewheel.CheeseWheelCommands;
import org.rivierarobotics.commands.ejector.EjectorCommands;
import org.rivierarobotics.subsystems.CheeseWheel;
import org.rivierarobotics.subsystems.Flywheel;
import org.rivierarobotics.subsystems.Turret;
import org.rivierarobotics.util.CheeseSlot;
import org.rivierarobotics.util.MathUtil;
import org.rivierarobotics.util.PhysicsUtil;
import org.rivierarobotics.util.ShooterConstants;

/**
 * Shoot a number (N) of balls sequentially. Does not wait for AutoAim.
 * Better than {@link All5Shoot} for variable count and use of caller
 * CycleSlot command. Set timeout between shooting to prevent jamming.
 */
@GenerateCreator
public class ShootNWedges extends CommandBase {
    private final CheeseWheelCommands cheeseWheelCommands;
    private final EjectorCommands ejectorCommands;
    private final Flywheel flywheel;
    private final boolean isBack;
    private final int wedges;
    private SequentialCommandGroup cmd;

    public ShootNWedges(@Provided CheeseWheelCommands cheeseWheelCommands,
                        @Provided EjectorCommands ejectorCommands, @Provided Turret turret, @Provided Flywheel flywheel,
                        int wedges) {
        this.cheeseWheelCommands = cheeseWheelCommands;
        this.flywheel = flywheel;
        this.ejectorCommands = ejectorCommands;
        this.wedges = wedges;
        isBack = MathUtil.isWithinTolerance(turret.getAngle(false), 0, 90);
    }

    private SequentialCommandGroup singleWedgeGroup(boolean isBack) {
        if(flywheel.getPositionTicks() <= 1000){
            return new SequentialCommandGroup();
        }
        return new SequentialCommandGroup(
                cheeseWheelCommands.cycleSlot(isBack ? CheeseWheel.Direction.BACKWARDS : CheeseWheel.Direction.FORWARDS,
                        isBack ? CheeseWheel.AngleOffset.SHOOTER_BACK : CheeseWheel.AngleOffset.SHOOTER_FRONT, CheeseSlot.State.BALL),
                new WaitUntilCommand(flywheel::withinTolerance),
                new WaitCommand(1),
                ejectorCommands.setPower(1.0),
                new WaitCommand(1),
                ejectorCommands.setPower(0)
        );


    }

    @Override
    public void initialize() {
        SmartDashboard.putNumber("RAN", flywheel.getPositionTicks());
        cmd = new SequentialCommandGroup();
        for(int i = 0; i < wedges; i++) {
            cmd.addCommands(singleWedgeGroup(isBack));
        }
        cmd.schedule();
    }

    @Override
    public boolean isFinished() {
        return cmd != null && !CommandScheduler.getInstance().isScheduled(cmd);
    }
}
