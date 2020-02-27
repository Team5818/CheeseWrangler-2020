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

package org.rivierarobotics.util;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.BaseMotorController;

public enum NeutralIdleMode {
    BRAKE(NeutralMode.Brake),
    COAST(NeutralMode.Coast);

    public final NeutralMode ctre;

    NeutralIdleMode(NeutralMode ctre) {
        this.ctre = ctre;
    }

    public void applyTo(BaseMotorController... controllers) {
        for (BaseMotorController controller : controllers) {
            controller.setNeutralMode(this.ctre);
        }
    }
}
