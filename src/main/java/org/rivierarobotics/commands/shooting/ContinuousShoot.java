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

@GenerateCreator
public class ContinuousShoot extends CommandBase {
    private final CheeseWheel cheeseWheel;
    private final Turret turret;
    private final Ejector ejector;
    private boolean finishedShooting = true;
    private double shootStartTimestamp = -1;
    private Double endTime;

    public ContinuousShoot(@Provided CheeseWheel cheeseWheel, @Provided Turret turret, @Provided Ejector ejector) {
        this.turret = turret;
        this.cheeseWheel = cheeseWheel;
        this.ejector = ejector;
        addRequirements(cheeseWheel, ejector);
    }

    @Override
    public void execute() {
        CheeseWheel.AngleOffset offset = MathUtil.isWithinTolerance(turret.getAngle(false), 0, 90)
                ? CheeseWheel.AngleOffset.SHOOTER_FRONT : CheeseWheel.AngleOffset.SHOOTER_BACK;
        CheeseSlot currentSlot = CheeseSlot.slotOfNum(cheeseWheel.getIndex(offset));
        if (finishedShooting) {
            if (!cheeseWheel.onSlot(offset, 30) || !currentSlot.hasBall()) {
                finishedShooting = false;
                cheeseWheel.setPositionTicks(cheeseWheel.getSlotTickPos(
                        cheeseWheel.getClosestSlot(offset, offset.direction, CheeseSlot.State.BALL), offset, offset.direction));
            }
        } else if (currentSlot.hasBall() || (shootStartTimestamp != -1 && shootStartTimestamp + 0.3 < Timer.getFPGATimestamp())) {
            ejector.setPower(1);
        } else if (shootStartTimestamp == -1) {
            shootStartTimestamp = Timer.getFPGATimestamp();
        } else {
            ejector.setPower(0);
            finishedShooting = true;
            shootStartTimestamp = -1;
        }
    }

    @Override
    public boolean isFinished() {
        if (endTime != null) {
            return !(endTime + 1 < Timer.getFPGATimestamp());
        }
        for (CheeseSlot slot : CheeseSlot.values()) {
            if (slot.hasBall()) {
                return false;
            }
        }
        endTime = Timer.getFPGATimestamp();
        return false;
    }
}
