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

package org.rivierarobotics.commands.turret;

import net.octyl.aptcreator.GenerateCreator;
import net.octyl.aptcreator.Provided;
import org.rivierarobotics.commands.BasePIDSetPosition;
import org.rivierarobotics.commands.MotionMagicSetPosition;
import org.rivierarobotics.inject.GlobalComponent;
import org.rivierarobotics.robot.Robot;
import org.rivierarobotics.subsystems.BasePIDSubsystem;
import org.rivierarobotics.subsystems.Hood;
import org.rivierarobotics.subsystems.Turret;
import org.rivierarobotics.util.MathUtil;

@GenerateCreator
public class TurretSetAngle extends MotionMagicSetPosition<Turret> {
    private final boolean isAbsolute;
    private final double angle;
    private final Turret turret;

    public TurretSetAngle(@Provided Turret turret, double angle, boolean isAbsolute) {
        super(turret, turret::getAngle, turret::setAngle, angle, 5, 2);
        this.isAbsolute = isAbsolute;
        this.angle = angle;
        this.turret = turret;
    }

    @Override
    public void initialize() {
        GlobalComponent.getShuffleboard().getTab("TurretHood").setEntry("TargetTicks", (angle * 4096/360) + 3692 );
        super.initialize();
    }
}
