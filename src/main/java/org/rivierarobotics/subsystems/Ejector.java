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

package org.rivierarobotics.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.rivierarobotics.commands.ejector.EjectorControl;
import org.rivierarobotics.inject.Sided;
import org.rivierarobotics.util.MathUtil;
import org.rivierarobotics.util.Side;

import javax.inject.Inject;
import javax.inject.Provider;
import javax.inject.Singleton;

@Singleton
public class Ejector extends SubsystemBase {
    private final EjectorSide left;
    private final EjectorSide right;
    private final Provider<EjectorControl> command;

    @Inject
    public Ejector(@Sided(Side.LEFT) EjectorSide left,
                  @Sided(Side.RIGHT) EjectorSide right,
                   Provider<EjectorControl> command) {
        this.left = left;
        this.right = right;
        this.command = command;
    }

    public void setPower(double pwr) {
        setPower(pwr, pwr);
    }

    public void setPower(double leftPwr, double rightPwr) {
        //TODO change this to reflect shooting offset
        left.setPower(leftPwr);
        right.setPower(MathUtil.fitDeadband(rightPwr - 0.1, 0.1));
    }

    @Override
    public void periodic() {
        if (getDefaultCommand() == null) {
            setDefaultCommand(command.get());
        }
        super.periodic();
    }
}
