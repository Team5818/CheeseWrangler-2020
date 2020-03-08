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

package org.rivierarobotics.commands.hood;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import net.octyl.aptcreator.GenerateCreator;
import net.octyl.aptcreator.Provided;
import org.rivierarobotics.subsystems.Hood;
import org.rivierarobotics.subsystems.HoodPosition;
import org.rivierarobotics.util.MathUtil;

@GenerateCreator
public class HoodSetAngle extends CommandBase {
    private final Hood hood;
    private final double angle;
    private double start;

    public HoodSetAngle(@Provided Hood hood, double angle) {
        this.hood = hood;
        this.angle = MathUtil.limit(angle, HoodPosition.BACK_DEFAULT.angle, HoodPosition.FORWARD.angle);
        addRequirements(hood);
    }

    @Override
    public void initialize() {
        hood.setAbsoluteAngle(angle);
        start = Timer.getFPGATimestamp();
    }

    @Override
    public boolean isFinished() {
        return MathUtil.isWithinTolerance(hood.getAbsoluteAngle(), angle, 0.5)
            || (Timer.getFPGATimestamp() - start) > 2;
    }
}
