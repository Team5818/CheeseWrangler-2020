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
import org.rivierarobotics.subsystems.Flywheel;
import org.rivierarobotics.subsystems.Hood;
import org.rivierarobotics.subsystems.Turret;
import org.rivierarobotics.util.PhysicsUtil;
import org.rivierarobotics.util.RobotShuffleboard;
import org.rivierarobotics.util.ShooterConstants;
import org.rivierarobotics.util.VisionTarget;
import org.rivierarobotics.util.VisionUtil;

@GenerateCreator
public class VisionAim extends CommandBase {
    private final PhysicsUtil physics;
    private final AutoAimUtil autoAimUtil;
    private final Flywheel flywheel;
    private final VisionUtil vision;
    private final double extraDistance;

    public VisionAim(@Provided Hood hood, @Provided Flywheel flywheel, @Provided Turret turret,
                     @Provided RobotShuffleboard shuffleboard, @Provided PhysicsUtil physics, @Provided VisionUtil vision,
                     VisionTarget target) {
        this.flywheel = flywheel;
        this.physics = physics;
        this.vision = vision;
        this.extraDistance = target == VisionTarget.INNER ? ShooterConstants.getDistanceFromOuterToInnerTarget() : 0;
        this.autoAimUtil = new AutoAimUtil(hood, flywheel, turret, shuffleboard.getTab("Auto Aim"));
        addRequirements(hood, flywheel, turret);
    }

    @Override
    public void execute() {
        physics.setAimMode(PhysicsUtil.AimMode.VISION);
        physics.setExtraDistance(extraDistance);
        if (vision.getLLValue("tv") == 1) {
            physics.calculateVelocities(false);
            double ballVel = physics.getBallVel();
            double hoodAngle = physics.getCalculatedHoodAngle();
            autoAimUtil.setValues(physics, hoodAngle, ballVel, physics.getAngleToTarget(), false);
        }
    }

    @Override
    public void end(boolean interrupted) {
        flywheel.setPower(0);
        physics.setExtraDistance(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
