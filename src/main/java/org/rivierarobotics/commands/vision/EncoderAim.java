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

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import net.octyl.aptcreator.GenerateCreator;
import net.octyl.aptcreator.Provided;
import org.rivierarobotics.subsystems.DriveTrain;
import org.rivierarobotics.subsystems.Flywheel;
import org.rivierarobotics.subsystems.Hood;
import org.rivierarobotics.subsystems.LimelightServo;
import org.rivierarobotics.subsystems.Turret;
import org.rivierarobotics.util.MathUtil;
import org.rivierarobotics.util.PositionTracker;
import org.rivierarobotics.util.ShooterUtil;
import org.rivierarobotics.util.VisionUtil;

@GenerateCreator
public class EncoderAim extends CommandBase {
    private final Hood hood;
    private final DriveTrain driveTrain;
    private final Flywheel flywheel;
    private final VisionUtil vision;
    private final Turret turret;
    private final PositionTracker tracker;
    private final double extraDistance;
    private final LimelightServo limelightServo;

    public EncoderAim(@Provided Hood hood, @Provided DriveTrain dt, @Provided Flywheel flywheel,
                      @Provided VisionUtil vision, @Provided Turret turret, @Provided PositionTracker tracker,
                      @Provided LimelightServo limelightServo, double extraDistance) {
        this.hood = hood;
        this.limelightServo = limelightServo;
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
        double xFromGoal = pos[1];
        double zFromGoal = pos[0];
        double dist = Math.sqrt(Math.pow(xFromGoal + extraDistance, 2) + Math.pow(zFromGoal, 2));
        double t = ShooterUtil.getTConstant();
        double vx = (extraDistance + xFromGoal) / t - driveTrain.getXVelocity();
        double vz = zFromGoal / t;
        double turretAngle = MathUtil.wrapToCircle(Math.toDegrees(Math.atan2(vz, vx)));

        turret.setAbsoluteAngle(turretAngle);

        double y = ShooterUtil.getYVelocityConstant();
        double vxz = Math.sqrt(Math.pow(vx, 2) + Math.pow(vz, 2));
        double hoodAngle = Math.toDegrees(Math.atan2(y, vxz));
        double ballVel = vxz / Math.cos(Math.toRadians(hoodAngle));
        double encoderVelocity = ShooterUtil.velocityToTicks(ballVel);

        double adjustedHoodAngle = 90 - hoodAngle;

        if (dist < ShooterUtil.getTopHeight() / Math.tan(Math.toRadians(ShooterUtil.getMaxHoodAngle()))) {
            //Close Shot >:(
            hood.setAbsoluteAngle(ShooterUtil.getMaxHoodAngle());
            flywheel.setPositionTicks(encoderVelocity);
        } else if (vxz > ShooterUtil.getMaxBallVelocity() || hoodAngle < ShooterUtil.getMinHoodAngle()) {
            //Long Shot / Arc shot
            hood.setAbsoluteAngle(adjustedHoodAngle);
            flywheel.setPositionTicks(ShooterUtil.getMaxFlywheelVelocity());
        } else {
            //Calculated Shot :)
            hood.setAbsoluteAngle(adjustedHoodAngle);
            flywheel.setPositionTicks(encoderVelocity);
        }

    }




    @Override
    public boolean isFinished() {
        return false;
    }
}
