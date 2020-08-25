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
import org.rivierarobotics.inject.GlobalComponent;
import org.rivierarobotics.robot.Robot;
import org.rivierarobotics.subsystems.Hood;
import org.rivierarobotics.subsystems.Turret;
import org.rivierarobotics.util.RobotShuffleboard;
import org.rivierarobotics.util.RobotShuffleboardTab;
import org.rivierarobotics.util.ShooterConstants;
import org.rivierarobotics.util.VisionUtil;

@GenerateCreator
public class VisionAimTurret extends CommandBase {
    private final Turret turret;
    private final Hood hood;
    private final VisionUtil vision;
    private final double extraDistance;
    private final double height;
    private final RobotShuffleboardTab tab;

    public VisionAimTurret(@Provided Turret turret, @Provided Hood hood, @Provided VisionUtil vision, @Provided RobotShuffleboard shuffleboard, double extraDistance, double height) {
        this.turret = turret;
        this.hood = hood;
        this.vision = vision;
        this.height = height;
        this.extraDistance = extraDistance;
        this.tab = shuffleboard.getTab("Auto Aim");
        addRequirements(turret);
    }

    @Override
    public void execute() {
        double t = ShooterConstants.getTConstant();
        double dist = height / Math.tan(Math.toRadians(vision.getActualTY(hood.getAngle())));
        double txTurret = turret.getTxTurret(dist, extraDistance);
        double vx = (dist * Math.cos(txTurret) + extraDistance) / t;
        double vz = dist * Math.sin(txTurret) / t;
        double turretAngle = Math.toDegrees(Math.atan2(vz, vx));
        double absolute = turret.getAbsoluteAngle();
        var turretAngleAdj = turretAngle + absolute;

        tab.setEntry("TurretAngleAdj", turretAngleAdj);
        tab.setEntry("turretAngle", turretAngle);
        tab.setEntry("txTurret", txTurret);
        tab.setEntry("turretVZ", vz);

        if (turret.isAutoAimEnabled()) {
            if (vision.getLLValue("tv") == 1) {
                turret.setAngle(turretAngleAdj);
            }
        }


    }

    @Override
    public boolean isFinished(){
        return false;
    }
}
