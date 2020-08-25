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

package org.rivierarobotics.commands.vision; /*
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

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import net.octyl.aptcreator.GenerateCreator;
import net.octyl.aptcreator.Provided;
import org.rivierarobotics.inject.GlobalComponent;
import org.rivierarobotics.robot.Robot;
import org.rivierarobotics.subsystems.DriveTrain;
import org.rivierarobotics.subsystems.Flywheel;
import org.rivierarobotics.subsystems.Hood;
import org.rivierarobotics.subsystems.Turret;
import org.rivierarobotics.util.PositionTracker;
import org.rivierarobotics.util.RobotShuffleboard;
import org.rivierarobotics.util.RobotShuffleboardTab;
import org.rivierarobotics.util.ShooterConstants;
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
    private final RobotShuffleboardTab tab;

    public CalcAim(@Provided Hood hood, @Provided DriveTrain dt, @Provided Flywheel flywheel,
                   @Provided VisionUtil vision, @Provided Turret turret, @Provided PositionTracker tracker, @Provided RobotShuffleboard shuffleboard,
                   double extraDistance) {
        this.hood = hood;
        this.driveTrain = dt;
        this.flywheel = flywheel;
        this.vision = vision;
        this.turret = turret;
        this.tracker = tracker;
        this.extraDistance = extraDistance;
        this.tab = shuffleboard.getTab("Auto Aim");
        addRequirements(hood, flywheel, turret);
    }

    @Override
    public void execute() {
        tab.clear();
        tab.setEntry("Aim Mode: ", "CalcAim");
        double y = ShooterConstants.getYVelocityConstant();
        double[] pos = tracker.getPosition();
        double xFromGoal = pos[1];
        double zFromGoal = pos[0];
        double dist = Math.sqrt(Math.pow(xFromGoal, 2) + Math.pow(zFromGoal, 2));
        SmartDashboard.putNumber("dist", dist);
        double t = ShooterConstants.getTConstant();
        double vx = (extraDistance + xFromGoal) / t - driveTrain.getYVelocity();
        double vz = zFromGoal / t - driveTrain.getXVelocity();
        double vxz = Math.sqrt(Math.pow(vx, 2) + Math.pow(vz, 2));
        double hoodAngle = Math.toDegrees(Math.atan2(y, vxz));
        double ballVel = vxz / Math.cos(Math.toRadians(hoodAngle));
        double encoderVelocity = ShooterConstants.velocityToTicks(ballVel);
        double captainKalbag = Math.toDegrees(captainKalbag(xFromGoal, zFromGoal));

        tab.setEntry("Change In Angle", captainKalbag);
        tab.setEntry("BallVel", ballVel);
        tab.setEntry("hoodAngle ", hoodAngle);

        if (turret.isAutoAimEnabled()) {
            if (hoodAngle > ShooterConstants.getMaxHoodAngle()) {
                //Close Shot
                hood.setAngle(90 - hoodAngle);
                flywheel.setVelocity(ShooterConstants.velocityToTicks(ShooterConstants.getShooterMinVelocity()));
            } else if (ballVel > ShooterConstants.getMaxBallVelocity()) {
                //Long Shot
                hood.setAngle(90 - (33 + 0.1 * dist));
                flywheel.setVelocity(ShooterConstants.velocityToTicks(ShooterConstants.getShooterMaxVelocity()));
            } else {
                //Calculated Shot
                hood.setAngle(90 - hoodAngle);
                flywheel.setVelocity(encoderVelocity);
            }

            turret.setPositionTicks(captainKalbag * turret.getAnglesOrInchesToTicks() / 10 + 5);
        }
    }

    private double captainKalbag(double xFromGoal, double zFromGoal) {
        double timeAdvance = 0.1;
        double xDist = xFromGoal - driveTrain.getYVelocity() * timeAdvance;
        double zDist = zFromGoal - driveTrain.getXVelocity() * timeAdvance;
        return (1 / (Math.pow((zDist / xDist), 2) + 1)) * ((-driveTrain.getXVelocity() * xDist)
                - (-driveTrain.getYVelocity() * xDist)) / Math.pow(xDist, 2);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
