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

import edu.wpi.first.wpilibj.Timer;
import org.rivierarobotics.subsystems.DriveTrain;
import org.rivierarobotics.subsystems.Flywheel;
import org.rivierarobotics.subsystems.Hood;
import org.rivierarobotics.subsystems.Turret;
import org.rivierarobotics.util.MathUtil;
import org.rivierarobotics.util.PhysicsUtil;
import org.rivierarobotics.util.RobotShuffleboardTab;
import org.rivierarobotics.util.ShooterConstants;

public class AutoAimUtil {
    private final Hood hood;
    private final Flywheel flywheel;
    private final Turret turret;
    private final RobotShuffleboardTab tab;
    private final DriveTrain driveTrain;
    private double start = 0;

    public AutoAimUtil(Hood hood, Flywheel flywheel, Turret turret, RobotShuffleboardTab tab, DriveTrain driveTrain) {
        this.hood = hood;
        this.flywheel = flywheel;
        this.turret = turret;
        this.tab = tab;
        this.driveTrain = driveTrain;
    }

    public AutoAimUtil(Hood hood, Flywheel flywheel, Turret turret, RobotShuffleboardTab tab) {
        this.hood = hood;
        this.flywheel = flywheel;
        this.turret = turret;
        this.tab = tab;
        this.driveTrain = null;
    }

    public void setValues(PhysicsUtil physics, double hoodAngle, double ballVel, double turretAngle, boolean useVelocity) {
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
        tab.setEntry("bbvv", ballVel);
        if (physics.isAutoAimEnabled()) {
            flywheel.setVelocity(ShooterConstants.velocityToTicks(ballVel));
            hood.setAngle(hoodAngle);
            if (!useVelocity) {
                turret.setAngle(turretAngle, true);
            } else {
                if (driveTrain != null && MathUtil.isWithinTolerance(driveTrain.getLeft().getVelocity(), 0,  0.1)
                        && MathUtil.isWithinTolerance(driveTrain.getRight().getVelocity(), 0, 0.1)) {
                    if (Timer.getFPGATimestamp() - start < 0.001) {
                        turret.setVelocity(physics.getTurretVelocity());
                    } else {
                        turret.setAngle(turretAngle, true);
                    }
                } else {
                    start = Timer.getFPGATimestamp();
                    turret.setVelocity(physics.getTurretVelocity());
                }
            }
        } else {
            flywheel.setVelocity(0);
        }
    }
}
