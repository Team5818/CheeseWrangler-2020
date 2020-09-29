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
import org.rivierarobotics.util.PositionTracker;
import org.rivierarobotics.util.RobotShuffleboard;
import org.rivierarobotics.util.RobotShuffleboardTab;
import org.rivierarobotics.util.ShooterConstants;
import org.rivierarobotics.util.VisionTarget;

@GenerateCreator
public class EncoderAim extends CommandBase {
    private final Hood hood;
    private final DriveTrain driveTrain;
    private final Flywheel flywheel;
    private final Turret turret;
    private final PositionTracker tracker;
    private final VisionTarget target;
    private final RobotShuffleboardTab tab;

    public EncoderAim(@Provided Hood hood, @Provided DriveTrain dt, @Provided Flywheel flywheel,
                      @Provided Turret turret, @Provided PositionTracker tracker, @Provided RobotShuffleboard shuffleboard, VisionTarget target) {
        this.hood = hood;
        this.driveTrain = dt;
        this.flywheel = flywheel;
        this.turret = turret;
        this.tracker = tracker;
        this.target = target;
        this.tab = shuffleboard.getTab("Auto Aim");
        addRequirements(hood, flywheel, turret);
    }

    @Override
    public void execute() {
        tab.setEntry("Aim Mode: ", "Encoder Aim");
        double extraDistance = target == VisionTarget.INNER ? ShooterConstants.getDistanceFromOuterToInnerTarget() : 0;

        double[] pos = tracker.getPosition();
        double xFromGoal = pos[1];
        double zFromGoal = pos[0];
        double t = ShooterConstants.getTConstant();
        double vx = (extraDistance + xFromGoal) / t - driveTrain.getXVelocity();
        double vz = zFromGoal / t;
        double turretAngle = MathUtil.wrapToCircle(Math.toDegrees(Math.atan2(vz, vx)));
        double dist = Math.sqrt(Math.pow(xFromGoal + extraDistance, 2) + Math.pow(zFromGoal, 2));

        double y = ShooterConstants.getYVelocityConstant();
        double vxz = Math.sqrt(Math.pow(vx, 2) + Math.pow(vz, 2));
        double hoodAngle = Math.toDegrees(Math.atan2(y, vxz));
        double ballVel = vxz / Math.cos(Math.toRadians(hoodAngle));
        double encoderVelocity = ShooterConstants.velocityToTicks(ballVel);

        tab.setEntry("Dist", dist);
        tab.setEntry("hoodAngle", hoodAngle);
        tab.setEntry("vx", vx);
        tab.setEntry("vxz", vxz);
        tab.setEntry("ballVel", ballVel);

        Turret.IS_ABSOLUTE_ANGLE = true;

        if (turret.isAutoAimEnabled()) {
            if (hoodAngle > ShooterConstants.getMaxHoodAngle()) {
                //Close Shot
                hood.setAngle(90 - hoodAngle);
                flywheel.setVelocity(ShooterConstants.velocityToTicks(ShooterConstants.getShooterMinVelocity()));
                tab.setEntry("Target: ", "Close Shot");
            } else if (ballVel > ShooterConstants.getMaxBallVelocity()) {
                //Long Shot
                hood.setAngle(90 - (33 + 0.1 * dist));
                flywheel.setVelocity(ShooterConstants.velocityToTicks(ShooterConstants.getShooterMinVelocity()));
                tab.setEntry("Target: ", "Long Shot");
            } else {
                //Calculated Shot
                hood.setAngle(90 - hoodAngle);
                flywheel.setVelocity(encoderVelocity);
                tab.setEntry("Target: ", "Calculated Shot");
            }
            turret.setAngle(turretAngle);
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
