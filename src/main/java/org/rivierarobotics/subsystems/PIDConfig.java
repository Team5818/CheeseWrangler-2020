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

import com.ctre.phoenix.motorcontrol.can.BaseTalon;

/**
 * Stores a PIDF configuration through loop gain constants.
 *
 * <p>Terms may be accessed or changed through standard getters/setters.
 * Contains helper method to apply terms to a given CTRE motor controller.
 * Should be used in all places where PIDF loops are needed.</p>
 */
public class PIDConfig {
    private double kP;
    private double kI;
    private double kD;
    private double kF;
    private double pidRange;

    public PIDConfig(double kP, double kI, double kD, double kF, double pidRange) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kF = kF;
        this.pidRange = pidRange;
    }

    public PIDConfig(double kP, double kI, double kD, double kF) {
        this(kP, kI, kD, kF, 1.0);
    }

    public PIDConfig(double kP, double kI, double kD) {
        this(kP, kI, kD, 0.0, 1.0);
    }

    public double getP() {
        return kP;
    }

    public double getI() {
        return kI;
    }

    public double getD() {
        return kD;
    }

    public double getF() {
        return kF;
    }

    public double getRange() {
        return pidRange;
    }

    public PIDConfig setP(double kP) {
        this.kP = kP;
        return this;
    }

    public PIDConfig setI(double kI) {
        this.kI = kI;
        return this;
    }

    public PIDConfig setD(double kD) {
        this.kD = kD;
        return this;
    }

    public PIDConfig setF(double kF) {
        this.kF = kF;
        return this;
    }

    public PIDConfig setRange(double pidRange) {
        this.pidRange = pidRange;
        return this;
    }

    /**
     * Applies the current PIDF configuration to a given CTRE motor controller.
     *
     * <p>The slot number is [0, 3] as dictated by the 4 slots per controller.
     * Configurations only need to be applied once, then switched between with
     * <code>motor.selectProfileSlot(idx, 0)</code>. Note that the 0 represents
     * the primary controller. It is suggested to remain on the primary for quick
     * switching (i.e. position to velocity) and resort to auxiliary if more
     * than four configurations are needed (unlikely) or two controllers need
     * to be running simultaneously (not recommended).</p>
     *
     * @param motor the CTRE motor to apply the current configuration to.
     * @param slotIdx the index of the profile slot to apply the configuration to.
     */
    public void applyTo(BaseTalon motor, int slotIdx) {
        motor.config_kP(slotIdx, kP);
        motor.config_kI(slotIdx, kI);
        motor.config_kD(slotIdx, kD);
        motor.config_kF(slotIdx, kF);
    }
}
