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

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import org.rivierarobotics.inject.Input;
import org.rivierarobotics.subsystems.Turret;
import org.rivierarobotics.util.MathUtil;

import javax.inject.Inject;

public class TurretControl extends CommandBase {
    private final Turret turret;
    private final Joystick coDriverRightJs;

    @Inject
    public TurretControl(@Input(Input.Selector.CODRIVER_RIGHT) Joystick coDriverRightJs,
                         Turret turret) {
        this.turret = turret;
        this.coDriverRightJs = coDriverRightJs;
        addRequirements(turret);
    }

    @Override
    public void execute() {
        //TODO uncomment when robot is stable
        turret.setManualPower(0.7 * MathUtil.fitDeadband(coDriverRightJs.getX()));
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
