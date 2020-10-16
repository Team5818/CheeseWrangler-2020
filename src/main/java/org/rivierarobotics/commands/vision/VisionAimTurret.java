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

package org.rivierarobotics.commands.vision;

import edu.wpi.first.wpilibj2.command.CommandBase;
import net.octyl.aptcreator.GenerateCreator;
import net.octyl.aptcreator.Provided;
import org.rivierarobotics.subsystems.Turret;
import org.rivierarobotics.util.PhysicsUtil;
import org.rivierarobotics.util.VisionUtil;

@GenerateCreator
public class VisionAimTurret extends CommandBase {
    private final Turret turret;
    private final PhysicsUtil physics;
    private final VisionUtil vision;

    public VisionAimTurret(@Provided Turret turret, @Provided VisionUtil vision,
                           @Provided PhysicsUtil physics, double extraDistance) {
        this.turret = turret;
        this.vision = vision;
        this.physics = physics;
        this.physics.setExtraDistance(extraDistance);
        physics.setAimMode(PhysicsUtil.AimMode.VISION);
        addRequirements(turret);
    }

    @Override
    public void execute() {
        physics.setVelocity(19);
        physics.calculateVelocities(false, false);
        if (turret.isAutoAimEnabled()) {
            if (vision.getLLValue("tv") == 1) {
                turret.setAngle(physics.getAngleToTarget(), true);
            }
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
