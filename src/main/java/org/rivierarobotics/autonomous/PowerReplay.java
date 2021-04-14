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

package org.rivierarobotics.autonomous;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import net.octyl.aptcreator.GenerateCreator;
import net.octyl.aptcreator.Provided;
import org.rivierarobotics.inject.Input;
import org.rivierarobotics.subsystems.DriveTrain;
import org.rivierarobotics.util.MathUtil;
import org.rivierarobotics.util.RobotShuffleboard;
import org.rivierarobotics.util.RobotShuffleboardTab;

import java.util.LinkedList;

@GenerateCreator
public class PowerReplay extends CommandBase {
    private final DriveTrain driveTrain;
    private final Joystick driverLeft;
    private final Joystick driverRight;
    private final Joystick driverButtons;
    private final RobotShuffleboardTab tab;
    private LinkedList<double[]> pwrs;
    private int phase;

    public PowerReplay(@Provided DriveTrain driveTrain,
                       @Provided @Input(Input.Selector.DRIVER_LEFT) Joystick driverLeft,
                       @Provided @Input(Input.Selector.DRIVER_RIGHT) Joystick driverRight,
                       @Provided @Input(Input.Selector.DRIVER_BUTTONS) Joystick driverButtons,
                       @Provided RobotShuffleboard shuffleboard) {
        this.driveTrain = driveTrain;
        this.driverLeft = driverLeft;
        this.driverRight = driverRight;
        this.driverButtons = driverButtons;
        this.tab = shuffleboard.getTab("PathTracer");
        addRequirements(driveTrain);
    }

    @Override
    public void initialize() {
        pwrs = new LinkedList<>();
        phase = 0;
    }

    @Override
    public void execute() {
        if (driverButtons.getRawButtonPressed(5)) {
            phase++;
        }
        double jsx = MathUtil.fitDeadband(driverRight.getX());
        double jsy = MathUtil.fitDeadband(-driverLeft.getY());
        double[] pTemp = MathUtil.arcadeDrive(jsx, jsy);
        if (phase == 0) {
            pwrs.add(pTemp);
        } else if (phase >= 2) {
            pTemp = pwrs.remove(0);
        }
        tab.setEntry("phase", phase);
        driveTrain.setPower(pTemp[0], pTemp[1]);
    }

    @Override
    public void end(boolean interrupted) {
        driveTrain.setPower(0, 0);
    }

    @Override
    public boolean isFinished() {
        return phase >= 2 && pwrs.size() == 0;
    }
}
