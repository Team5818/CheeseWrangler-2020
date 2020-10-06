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
import edu.wpi.first.wpilibj2.command.CommandBase;
import net.octyl.aptcreator.GenerateCreator;
import net.octyl.aptcreator.Provided;
import org.rivierarobotics.subsystems.CheeseWheel;
import org.rivierarobotics.subsystems.Ejector;
import org.rivierarobotics.subsystems.Turret;
import org.rivierarobotics.util.CheeseSlot;
import org.rivierarobotics.util.MathUtil;
import org.rivierarobotics.util.RobotShuffleboard;
import org.rivierarobotics.util.RobotShuffleboardTab;

@GenerateCreator
public class Shoot extends CommandBase {
    private final CheeseWheel cheeseWheel;
    private final Turret turret;
    private final RobotShuffleboardTab tab;
    private final Ejector ejector;
    private boolean finishedShooting;

    public Shoot(@Provided CheeseWheel cheeseWheel, @Provided Turret turret, @Provided Ejector ejector, @Provided RobotShuffleboard shuffleboard) {
        this.turret = turret;
        this.cheeseWheel = cheeseWheel;
        this.ejector = ejector;
        this.tab = shuffleboard.getTab("TurretHood");
        addRequirements(cheeseWheel);
    }

    @Override
    public void execute() {
        CheeseWheel.AngleOffset offset =
                MathUtil.isWithinTolerance(turret.getAngle(), 0, 90) ? CheeseWheel.AngleOffset.SHOOTER_FRONT : CheeseWheel.AngleOffset.SHOOTER_BACK;

        if (!cheeseWheel.onSlot(offset, 30) || (!CheeseSlot.slotOfNum(cheeseWheel.getIndex(offset)).hasBall() && finishedShooting)) {
            finishedShooting = false;
            cheeseWheel.setPositionTicks(cheeseWheel.getSlotTickPos(
                    cheeseWheel.getClosestSlot(offset, offset.direction, CheeseSlot.State.BALL), offset, offset.direction));
        } else {
            ejector.setPower(1);
            while (CheeseSlot.slotOfNum(cheeseWheel.getIndex(offset)).hasBall()) {
            }
            double startTime = Timer.getFPGATimestamp();
            if (startTime + 0.3 < Timer.getFPGATimestamp()) {
                ejector.setPower(0);
                finishedShooting = true;
            }
        }

    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        for (int i = 0; i < 5; i++) {
            if (CheeseSlot.slotOfNum(i).hasBall()) {
                return false;
            }
        }
        return true;
    }
}
