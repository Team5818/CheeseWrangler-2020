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

package org.rivierarobotics.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import dagger.Provides;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.rivierarobotics.commands.EjectorControl;
import org.rivierarobotics.inject.Sided;

import javax.inject.Inject;
import javax.inject.Provider;

public class Ejector extends SubsystemBase {
    private final EjectorSide left;
    private final EjectorSide right;
    private final Provider<EjectorControl> command;

    @Inject
    public Ejector(@Sided(Sided.Side.LEFT) EjectorSide left,
                   @Sided(Sided.Side.RIGHT) EjectorSide right,
                   Provider<EjectorControl> command) {
        this.command = command;
        this.left = left;
        this.right = right;
    }


    public void setPower(double pwr) {
        left.setPower(pwr);
        right.setPower(pwr);
    }

    public void setPower(double leftPwr, double rightPwr) {
        left.setPower(leftPwr);
        right.setPower(rightPwr);
    }

    @Override
    public void periodic() {
        if (getDefaultCommand() == null) {
            setDefaultCommand(command.get());
        }
        super.periodic();
    }
}
