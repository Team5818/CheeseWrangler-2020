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

package org.rivierarobotics.commands.ejector;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import net.octyl.aptcreator.GenerateCreator;
import net.octyl.aptcreator.Provided;
import org.rivierarobotics.commands.cheesewheel.CheeseWheelCommands;
import org.rivierarobotics.subsystems.CheeseWheel;
import org.rivierarobotics.subsystems.Ejector;
import org.rivierarobotics.util.BallTracker;

@GenerateCreator
public class EjectorSetPower extends InstantCommand {
    private final Ejector ejector;
    private final double power;
    private final BallTracker ballTracker;
    private final CheeseWheelCommands cheeseWheelCommands;

    public EjectorSetPower(@Provided Ejector ejector, @Provided BallTracker ballTracker, double power, @Provided CheeseWheelCommands cheeseCommands) {
        this.ejector = ejector;
        this.power = power;
        this.ballTracker = ballTracker;
        this.cheeseWheelCommands = cheeseCommands;
        addRequirements(ejector);
    }

    @Override
    public void execute() {
        if(!ballTracker.shooterOnIndex && power > 0) {
             SmartDashboard.putNumber("IRANP", Timer.getFPGATimestamp());
           cheeseWheelCommands.moveToNextIndexCancel(-1, CheeseWheel.AngleOffset.SHOOTING);
            ejector.setPower(0);
        } else {
            ejector.setPower(power);
        }
        ejector.setPower(power);
    }
}
