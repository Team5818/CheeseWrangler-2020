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

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.CommandBase;
import org.rivierarobotics.subsystems.Hood;
import org.rivierarobotics.subsystems.Turret;
import org.rivierarobotics.util.VisionUtil;

public class VisionAimTurret extends CommandBase {
    private final ShuffleboardTab vision;
    private Turret turret;
    private Hood hood;

    public VisionAimTurret(Turret turret) {
        vision = Shuffleboard.getTab("vision");
        this.turret = turret;
        this.hood = hood;
        addRequirements(turret);
    }

    @Override
    public void execute() {
        double tv = VisionUtil.getLLValue("tv");
        double tx = VisionUtil.getLLValue("tx");
        double ty = VisionUtil.getLLValue("ty");

        getShuffleboardEntry("Valid Target").setBoolean(tv == 1);
        getShuffleboardEntry("X Offset").setDouble(tx);
        getShuffleboardEntry("Y Offset").setDouble(ty);

        if (tv == 1) {
            hood.setPosition(hood.getPosition() + (90 - ty));
            turret.setPosition(turret.getPosition() + tx);
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    private NetworkTableEntry getShuffleboardEntry(String key) {
        return vision.add(key, 0).getEntry();
    }
}
