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

import net.octyl.aptcreator.GenerateCreator;
import net.octyl.aptcreator.Provided;
import org.rivierarobotics.commands.MotionMagicSetPosition;
import org.rivierarobotics.subsystems.Hood;
import org.rivierarobotics.subsystems.HoodPosition;
import org.rivierarobotics.util.MathUtil;
import org.rivierarobotics.util.RobotShuffleboard;

@GenerateCreator
public class HoodSetAngle extends MotionMagicSetPosition<Hood> {
    public HoodSetAngle(@Provided Hood hood, @Provided RobotShuffleboard shuffleboard, double angle) {
        super(hood, hood::getAngle, hood::setAngle,
                MathUtil.limit(angle, HoodPosition.BACK_DEFAULT.angle, HoodPosition.FORWARD.angle),
                0.5, 2, shuffleboard);
    }
}
