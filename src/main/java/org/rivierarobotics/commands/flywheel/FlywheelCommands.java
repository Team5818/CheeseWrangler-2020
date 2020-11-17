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

package org.rivierarobotics.commands.flywheel;

import org.rivierarobotics.commands.shooting.ChangeSpeed;
import org.rivierarobotics.commands.shooting.ChangeSpeedCreator;

import javax.inject.Inject;

public class FlywheelCommands {
    private final FlywheelSetPowerCreator flywheelSetPowerCreator;
    private final FlywheelSetVelocityCreator flywheelSetVelocityCreator;
    private final FlywheelStepToleranceCreator flywheelStepToleranceCreator;
    private final ChangeSpeedCreator changeSpeedCreator;

    @Inject
    public FlywheelCommands(FlywheelSetPowerCreator flywheelSetPowerCreator,
                            FlywheelSetVelocityCreator flywheelSetVelocityCreator,
                            FlywheelStepToleranceCreator flywheelStepToleranceCreator,
                            ChangeSpeedCreator changeSpeedCreator) {
        this.flywheelSetPowerCreator = flywheelSetPowerCreator;
        this.flywheelSetVelocityCreator = flywheelSetVelocityCreator;
        this.flywheelStepToleranceCreator = flywheelStepToleranceCreator;
        this.changeSpeedCreator = changeSpeedCreator;
    }

    public FlywheelSetPower setPower(double pwr) {
        return flywheelSetPowerCreator.create(pwr);
    }

    public FlywheelSetVelocity setVelocity() {
        return flywheelSetVelocityCreator.create();
    }

    public FlywheelSetVelocity setVelocity(double vel) {
        return flywheelSetVelocityCreator.create(vel);
    }

    public FlywheelStepTolerance stepTolerance(int amount) {
        return flywheelStepToleranceCreator.create(amount);
    }

    public ChangeSpeed changeAutoAimSpeed(double amount) {
        return changeSpeedCreator.create(amount);
    }
}
