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
import org.rivierarobotics.subsystems.CheeseWheel;
import org.rivierarobotics.util.CheeseSlot;

@GenerateCreator
public class NiceShootinTex extends CommandBase {
    private final CheeseWheel cheeseWheel;
    private final int slots;
    private CheeseSlot currentSlot;
    private int direction;
    private int shotSlots;

    public NiceShootinTex(@Provided CheeseWheel cheeseWheel,
                          int slots) {
        this.cheeseWheel = cheeseWheel;
        this.slots = slots;
    }

    @Override
    public void initialize() {
        shotSlots = 0;
        currentSlot = cheeseWheel.getClosestSlot(CheeseWheel.Mode.COLLECT_FRONT, CheeseWheel.Filled.DONT_CARE, 0);
        if (!currentSlot.isFilled) {
            // we have to use whatever is closest
            currentSlot = cheeseWheel.getClosestSlot(CheeseWheel.Mode.COLLECT_FRONT, CheeseWheel.Filled.DONT_CARE, 0);
        }
        // assume (from ShootNWedges) that we already moved to this slot!
        // go to the next one...
        currentSlot = currentSlot.next(1);
        var diff = currentSlot.getModePosition(CheeseWheel.Mode.COLLECT_FRONT) - cheeseWheel.getPositionTicks();
        diff = cheeseWheel.correctDiffForGap(diff);
        direction = 1; //(int) Math.signum(diff);
        moveToNext();
    }

    @Override
    public void execute() {
        if (!cheeseWheel.getPidController().atSetpoint()) {
            return;
        }
        shotSlots++;
        currentSlot.isFilled = false;
        currentSlot = currentSlot.next(direction);
        if (!isFinished()) {
            moveToNext();
        }
    }

    private void moveToNext() {
        cheeseWheel.setPositionTicks(currentSlot.getModePosition(CheeseWheel.Mode.COLLECT_FRONT));
    }

    @Override
    public boolean isFinished() {
        return shotSlots == slots;
    }
}
