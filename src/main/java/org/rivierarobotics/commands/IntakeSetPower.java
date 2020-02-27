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

import edu.wpi.first.wpilibj2.command.InstantCommand;
import net.octyl.aptcreator.GenerateCreator;
import net.octyl.aptcreator.Provided;
import org.rivierarobotics.inject.Sided;
import org.rivierarobotics.subsystems.Intake;

@GenerateCreator
public class IntakeSetPower extends InstantCommand {
    private final Intake intake;
    private final double frontPower;
    private final double backPower;
    private static final double pwrConstant = 0.5;

    IntakeSetPower(@Provided Intake intake, Sided.Side side) {
        this.intake = intake;
        if (side == Sided.Side.FRONT) {
            frontPower = pwrConstant;
            backPower = 0.0;
        } else {
            if (side != Sided.Side.BACK) {
                throw new IllegalArgumentException("Invalid side: " + side);
            }
            frontPower = 0.0;
            backPower = pwrConstant;
        }
    }

    @Override
    public void execute() {
        intake.setPower(frontPower, backPower);
    }
}
