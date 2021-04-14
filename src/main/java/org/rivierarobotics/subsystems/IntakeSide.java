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

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import org.rivierarobotics.appjack.Logging;
import org.rivierarobotics.appjack.MechLogger;

public class IntakeSide {
    private final WPI_VictorSPX intakeVictor;
    private final MechLogger logger;

    public IntakeSide(int id, boolean invert) {
        this.intakeVictor = new WPI_VictorSPX(id);
        intakeVictor.configFactoryDefault();
        intakeVictor.setInverted(invert);
        intakeVictor.setNeutralMode(NeutralMode.Brake);
        this.logger = Logging.getLogger(getClass(), invert ? "left" : "right");
    }

    public void setPower(double pwr) {
        logger.powerChange(pwr);
        intakeVictor.set(pwr);
    }
}
