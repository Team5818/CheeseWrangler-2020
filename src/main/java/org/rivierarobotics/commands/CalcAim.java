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

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import net.octyl.aptcreator.GenerateCreator;
import net.octyl.aptcreator.Provided;
import org.rivierarobotics.subsystems.DriveTrain;
import org.rivierarobotics.subsystems.Flywheel;
import org.rivierarobotics.subsystems.Hood;
import org.rivierarobotics.subsystems.Turret;
import org.rivierarobotics.util.PositionTracker;
import org.rivierarobotics.util.ShooterUtil;
import org.rivierarobotics.util.VisionUtil;

@GenerateCreator
public class CalcAim extends CommandBase {
    private final Hood hood;
    private final DriveTrain driveTrain;
    private final Flywheel flywheel;
    private final VisionUtil vision;
    private final Turret turret;
    private final PositionTracker tracker;
    private final double extraDistance;

    //UNUSED SHOOTING CLASS. USE ENCODER AIM.

    public CalcAim(@Provided Hood hood, @Provided DriveTrain dt, @Provided Flywheel flywheel,
                   @Provided VisionUtil vision, @Provided Turret turret, @Provided PositionTracker tracker,
                   double extraDistance) {
        this.hood = hood;
        this.driveTrain = dt;
        this.flywheel = flywheel;
        this.vision = vision;
        this.turret = turret;
        this.tracker = tracker;
        this.extraDistance = extraDistance;
        addRequirements(hood, flywheel, turret);
    }

    @Override
    public void execute() {
        double[] pos = tracker.getPosition();
        double xFromGoal = ShooterUtil.getFieldLength() - pos[1];
        double zFromGoal = ShooterUtil.getLeftFieldToFarGoal() - pos[0];
        double dist = Math.sqrt(Math.pow(xFromGoal, 2) + Math.pow(zFromGoal, 2));
        SmartDashboard.putNumber("dist", dist);
        double t = ShooterUtil.getTConstant();
        double vx = (extraDistance + xFromGoal) / t - driveTrain.getYVelocity();
        double vz = zFromGoal / t - driveTrain.getXVelocity();
        double vxz = Math.sqrt(Math.pow(vx, 2) + Math.pow(vz, 2));
        double turretAngle = Math.toDegrees(Math.atan2(vz, vx));
        double captainKalbag = Math.toDegrees(captainKalbag(xFromGoal, zFromGoal));

        SmartDashboard.putNumber("changeInAngle", captainKalbag);

        if (Math.abs(driveTrain.getAvgVelocity()) > 20) {
            SmartDashboard.putNumber("turretvel", captainKalbag * turret.getAnglesOrInchesToTicks() / 10);
            turret.changeAimMode(Turret.AimMode.MOVING);
            turret.setPositionTicks(captainKalbag * turret.getAnglesOrInchesToTicks() / 10 + 5);
        } else {
            turret.changeAimMode(Turret.AimMode.STILL);
            turret.setAbsolutePosition(turretAngle);
        }

        double y = ShooterUtil.getYVelocityConstant();
        double hoodAngle = Math.toDegrees(Math.atan2(y, vxz));
        double ballVel = vxz / Math.cos(Math.toRadians(hoodAngle));
        double encoderVelocity = ShooterUtil.velocityToTicks(ballVel);
        SmartDashboard.putNumber("BallVel", ballVel);
        SmartDashboard.putNumber("FlyVel", encoderVelocity + 10);
        SmartDashboard.putNumber("HoodAngleMath", hoodAngle);
        if (hoodAngle <= ShooterUtil.getMaxHoodAngle() && encoderVelocity <= ShooterUtil.getMaxFlywheelVelocity()) {
            hood.setAbsoluteAngle(hoodAngle + 3.5);
            flywheel.setPositionTicks(encoderVelocity + 10);
        } else {
            if (dist < 1) {
                hood.setAbsoluteAngle(ShooterUtil.getMaxHoodAngle());
                flywheel.setPositionTicks(120);
            } else {
                if (dist > 1) {
                    hood.setAbsoluteAngle(hoodAngle);
                    flywheel.setPositionTicks(ShooterUtil.getMaxFlywheelVelocity());
                }
            }
        }
    }

    private double captainKalbag(double xFromGoal, double zFromGoal) {
        double epicTime = 0.1;
        double xDist = xFromGoal - driveTrain.getYVelocity() * epicTime;
        double zDist = zFromGoal - driveTrain.getXVelocity() * epicTime;
        return (1 / (Math.pow((zDist / xDist), 2) + 1)) * ((-driveTrain.getXVelocity() * xDist)
            - (-driveTrain.getYVelocity() * xDist)) / Math.pow(xDist, 2);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
