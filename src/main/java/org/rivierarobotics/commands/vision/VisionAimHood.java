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
import org.rivierarobotics.robot.Robot;
import org.rivierarobotics.subsystems.DriveTrain;
import org.rivierarobotics.subsystems.Flywheel;
import org.rivierarobotics.subsystems.Hood;
import org.rivierarobotics.subsystems.Turret;
import org.rivierarobotics.util.ShooterUtil;
import org.rivierarobotics.util.VisionUtil;

@GenerateCreator
public class VisionAimHood extends CommandBase {
    private final Hood hood;
    private final DriveTrain driveTrain;
    private final Flywheel flywheel;
    private final VisionUtil vision;
    private final Turret turret;
    private final double extraDistance;
    private final double height;

    public VisionAimHood(@Provided Hood hd, @Provided DriveTrain dt, @Provided Flywheel fly, @Provided VisionUtil vision, @Provided Turret turret, double extraDistance, double height) {
        this.hood = hd;
        this.driveTrain = dt;
        this.flywheel = fly;
        this.vision = vision;
        this.turret = turret;
        this.extraDistance = extraDistance;
        this.height = height;
        addRequirements(hood, flywheel);
    }

    @Override
    public void execute() {
        double vy = ShooterUtil.getYVelocityConstant();  //Vy constant
        double t = ShooterUtil.getTConstant();   //time constant
        double dist = height / Math.tan(Math.toRadians(vision.getActualTY(hood.getAngle())));
        double txTurret = turret.getTxTurret(dist, extraDistance);
        double vx = (dist * Math.cos(txTurret) + extraDistance) / t;
        double vz = (dist * Math.sin(txTurret)) / t;
        double vxz = Math.sqrt(Math.pow(vx, 2) + Math.pow(vz, 2)) + (0.5 * 0.2 * t * t);
        double hoodAngle = Math.toDegrees(Math.atan2(vy, vxz));
        double ballVel = vxz / Math.cos(Math.toRadians(hoodAngle));
        double encoderVelocity = ShooterUtil.velocityToTicks(ballVel);

        Robot.getShuffleboard().getTab("Auto Aim").setEntry("Dist", dist);
        Robot.getShuffleboard().getTab("Auto Aim").setEntry("hoodAngle", hoodAngle);
        Robot.getShuffleboard().getTab("Auto Aim").setEntry("vx", vx);
        Robot.getShuffleboard().getTab("Auto Aim").setEntry("vxz", vxz);
        Robot.getShuffleboard().getTab("Auto Aim").setEntry("ballVel", ballVel);

        if (vision.getLLValue("tv") ==  1) {
            if (hoodAngle > ShooterUtil.getMaxHoodAngle()) {
                //Close Shot
                hood.setAngle(90 - hoodAngle);
                flywheel.setVelocity(ShooterUtil.getShooterMinVelocity());
                Robot.getShuffleboard().getTab("Auto Aim").setEntry("Target: ", "Close Shot");
            } else if (ballVel > ShooterUtil.getMaxBallVelocity()) {
                //Long Shot
                hood.setAngle(90 - (33 + 0.1 * dist));
                flywheel.setVelocity(ShooterUtil.getShooterMaxVelocity());
                Robot.getShuffleboard().getTab("Auto Aim").setEntry("Target: ", "Long Shot");
            } else {
                //Calculated Shot
                hood.setAngle(90 - hoodAngle);
                flywheel.setVelocity(encoderVelocity);
                Robot.getShuffleboard().getTab("Auto Aim").setEntry("Target: ", "Calculated Shot");
            }
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
