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

package org.rivierarobotics.commands.drive;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import net.octyl.aptcreator.GenerateCreator;
import net.octyl.aptcreator.Provided;
import org.rivierarobotics.inject.Input;
import org.rivierarobotics.subsystems.DriveTrain;
import org.rivierarobotics.util.MathUtil;

@GenerateCreator
public class DriveControl extends CommandBase {
    private final DriveTrain driveTrain;
    private final Joystick leftJs;
    private final Joystick rightJs;

    public DriveControl(@Provided @Input(Input.Selector.DRIVER_LEFT) Joystick left,
                        @Provided @Input(Input.Selector.DRIVER_RIGHT) Joystick right,
                        DriveTrain dt) {
        this.driveTrain = dt;
        this.leftJs = left;
        this.rightJs = right;
        addRequirements(driveTrain);
    }

    @Override
    public void execute() {
        double x = MathUtil.fitDeadband(rightJs.getX());
        double y = MathUtil.fitDeadband(-leftJs.getY());
        double[] pwr = MathUtil.arcadeDrive(x, y);
        driveTrain.setPower(pwr[0], pwr[1]);
    }
}
