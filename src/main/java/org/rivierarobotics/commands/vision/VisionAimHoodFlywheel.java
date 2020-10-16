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
import org.rivierarobotics.util.RobotShuffleboardTab;
import org.rivierarobotics.util.ShooterConstants;
import org.rivierarobotics.util.VisionUtil;

@GenerateCreator
public class VisionAimHoodFlywheel extends CommandBase {
    private final Hood hood;
    private final Flywheel flywheel;
    private final Turret turret;
    private final VisionUtil vision;
    private final RobotShuffleboardTab tab;
    private  final PhysicsUtil physics;

    public VisionAimHoodFlywheel(@Provided Hood hd, @Provided Flywheel fly, @Provided RobotShuffleboard shuffleboard,
                                 @Provided PhysicsUtil physics, @Provided Turret turret, @Provided VisionUtil vision,
                                 double extraDistance) {
        this.hood = hd;
        this.flywheel = fly;
        this.physics = physics;
        physics.setExtraDistance(extraDistance);
        physics.setAimMode(PhysicsUtil.AimMode.VISION);
        this.vision = vision;
        this.turret = turret;
        this.tab = shuffleboard.getTab("Auto Aim");
        addRequirements(hood, flywheel);
    }

    @Override
    public void execute() {
        physics.setVelocity(19);
        physics.calculateVelocities(false, false);
        double hoodAngle = physics.getCalculatedHoodAngle();
        double ballVel = physics.getBallVel();
        double encoderVelocity = ShooterConstants.velocityToTicks(ballVel);
        if (turret.isAutoAimEnabled()) {
            if (vision.getLLValue("tv") ==  1) {
                if (ballVel > ShooterConstants.getMaxBallVelocity()) {
                    //Long Shot
                    hood.setAngle(ShooterConstants.getEstimatedHoodAngle(physics.getDistanceToTarget()));
                    flywheel.setVelocity(ShooterConstants.getShooterMaxVelocity());
                    tab.setEntry("Target: ", "Long Shot");
                } else {
                    hood.setAngle(hoodAngle);
                    flywheel.setVelocity(encoderVelocity);
                }
            }
        } else {
            flywheel.setVelocity(0);
        }
    }

    @Override
    public void end(boolean interrupted) {
        flywheel.setPower(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
