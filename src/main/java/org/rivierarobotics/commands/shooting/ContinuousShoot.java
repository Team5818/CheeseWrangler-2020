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
public class ContinuousShoot extends CommandBase {
    private final CheeseWheel cheeseWheel;
    private final Turret turret;
    private final RobotShuffleboardTab tab;
    private final Ejector ejector;
    private boolean finishedShooting = false;
    private CheeseSlot shootingSlot;
    private double shootStartTimestamp = -1;

    public ContinuousShoot(@Provided CheeseWheel cheeseWheel, @Provided Turret turret, @Provided Ejector ejector, @Provided RobotShuffleboard shuffleboard) {
        this.turret = turret;
        this.cheeseWheel = cheeseWheel;
        this.ejector = ejector;
        this.tab = shuffleboard.getTab("TurretHood");
        addRequirements(cheeseWheel, ejector);
    }

    @Override
    public void initialize() {
        moveWheel();
    }

    @Override
    public void execute() {
        CheeseWheel.AngleOffset offset = MathUtil.isWithinTolerance(turret.getAngle(), 0, 90)
                ? CheeseWheel.AngleOffset.SHOOTER_FRONT : CheeseWheel.AngleOffset.SHOOTER_BACK;
        CheeseSlot currentSlot = CheeseSlot.slotOfNum(cheeseWheel.getIndex(offset));
        if (finishedShooting) {
            moveWheel();
        } else if ((currentSlot.hasBall() || (shootStartTimestamp != -1 && shootStartTimestamp + 0.3 < Timer.getFPGATimestamp()))
                    && cheeseWheel.onSlot(offset, 30)) {
            ejector.setPower(1);
        } else if (shootStartTimestamp == -1) {
            shootStartTimestamp = Timer.getFPGATimestamp();
        } else {
            ejector.setPower(0);
            finishedShooting = true;
            shootStartTimestamp = -1;
        }
    }

    private void moveWheel() {
        CheeseWheel.AngleOffset offset = MathUtil.isWithinTolerance(turret.getAngle(), 0, 90)
                ? CheeseWheel.AngleOffset.SHOOTER_FRONT : CheeseWheel.AngleOffset.SHOOTER_BACK;
        CheeseSlot currentSlot = CheeseSlot.slotOfNum(cheeseWheel.getIndex(offset));
        if (!cheeseWheel.onSlot(offset, 30) || !currentSlot.hasBall()) {
            finishedShooting = false;
            cheeseWheel.setPositionTicks(cheeseWheel.getSlotTickPos(
                    cheeseWheel.getClosestSlot(offset, offset.direction, CheeseSlot.State.BALL), offset, offset.direction));
        }
    }

    @Override
    public boolean isFinished() {
        for (CheeseSlot slot : CheeseSlot.values()) {
            if (slot.hasBall()) {
                return false;
            }
        }
        return true;
    }
}
