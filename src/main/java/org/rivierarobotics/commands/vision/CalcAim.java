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
import org.rivierarobotics.subsystems.DriveTrain;
import org.rivierarobotics.subsystems.Flywheel;
import org.rivierarobotics.subsystems.Hood;
import org.rivierarobotics.subsystems.Turret;
import org.rivierarobotics.util.MathUtil;
import org.rivierarobotics.util.PhysicsUtil;
import org.rivierarobotics.util.RobotShuffleboard;
import org.rivierarobotics.util.RobotShuffleboardTab;
import org.rivierarobotics.util.ShooterConstants;
import org.rivierarobotics.util.VisionTarget;

@GenerateCreator
public class CalcAim extends CommandBase {
    private final Hood hood;
    private final Flywheel flywheel;
    private final Turret turret;
    private final RobotShuffleboardTab tab;
    private final PhysicsUtil physics;
    private final DriveTrain driveTrain;
    private double extraDistance;

    public CalcAim(@Provided Hood hood, @Provided Flywheel flywheel, @Provided Turret turret,
                   @Provided PhysicsUtil physics, @Provided RobotShuffleboard shuffleboard,
                   VisionTarget target, @Provided DriveTrain driveTrain) {
        this.hood = hood;
        this.flywheel = flywheel;
        this.turret = turret;
        this.driveTrain = driveTrain;
        this.physics = physics;
        this.extraDistance = target == VisionTarget.INNER ? ShooterConstants.getDistanceFromOuterToInnerTarget() : 0;
        this.tab = shuffleboard.getTab("Auto Aim");
        addRequirements(hood, flywheel, turret);
    }

    @Override
    public void execute() {
        physics.setExtraDistance(extraDistance);
        physics.setAimMode(PhysicsUtil.AimMode.CALC);
        physics.getTurretVelocity();
        physics.calculateVelocities(false);
        double ballVel = physics.getBallVel();
        double hoodAngle = physics.getCalculatedHoodAngle();
        if (hoodAngle > hood.getZeroedAngle(hood.getBackLimit())) {
            tab.setEntry("Limit?:", "Hood Angle");
            ballVel = physics.getBallVel(hood.getZeroedAngle(hood.getForwardLimit()));
            hoodAngle = hood.getZeroedAngle(hood.getBackLimit());
        } else if (ballVel < ShooterConstants.getShooterMinVelocity()) {
            tab.setEntry("Limit?:", "Slow Ball Velocity");
            ballVel = ShooterConstants.getShooterMinVelocity();
        } else {
            tab.setEntry("Limit?:", "None");
        }
        if (physics.isAutoAimEnabled()) {
            if (MathUtil.isWithinTolerance(driveTrain.getYVelocity(), 0,  0.1) && MathUtil.isWithinTolerance(driveTrain.getXVelocity(), 0, 0.1)) {
                turret.setAngle(physics.getAngleToTarget(), true);
            } else {
                turret.setVelocity(physics.getTurretVelocity());
            }

            flywheel.setVelocity(ShooterConstants.velocityToTicks(ballVel));
            hood.setAngle(hoodAngle);
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
