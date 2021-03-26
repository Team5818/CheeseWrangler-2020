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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.rivierarobotics.appjack.Logging;
import org.rivierarobotics.appjack.MechLogger;
import org.rivierarobotics.commands.ejector.EjectorControl;
import org.rivierarobotics.inject.Sided;
import org.rivierarobotics.util.Side;

import javax.inject.Inject;
import javax.inject.Provider;
import javax.inject.Singleton;

@Singleton
public class Ejector extends SubsystemBase {
    private final Provider<EjectorControl> command;
    private final WPI_VictorSPX ejectorVictor;
    private final MechLogger logger;

    public Ejector(int id, Provider<EjectorControl> command) {
        this.ejectorVictor = new WPI_VictorSPX(id);
        this.command = command;
        ejectorVictor.configFactoryDefault();
        ejectorVictor.setNeutralMode(NeutralMode.Brake);
        logger = Logging.getLogger(getClass());
    }

    public void setPower(double pwr) {
        logger.powerChange(pwr);
        ejectorVictor.set(pwr);
    }

    public double getPower() {
        return ejectorVictor.get();
    }

    @Override
    public void periodic() {
        if (getDefaultCommand() == null) {
            setDefaultCommand(command.get());
        }
        super.periodic();
    }
}
