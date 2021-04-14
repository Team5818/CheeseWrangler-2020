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

package org.rivierarobotics.util;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.BaseTalon;
import edu.wpi.first.wpilibj.Timer;
import org.rivierarobotics.subsystems.PIDConfig;

import java.util.LinkedList;
import java.util.List;

public class AutoTuner {
    private static final double RUN_TIME = 5;
    private static final double MAX_POS_ERR = 10;
    private static final double KP_MOD_FACTOR = 0.01;
    private static final Pair<Double> TIME_ERR_BOUNDS = new Pair<>(0.0, 0.0);
    private final BaseTalon motor;
    private final double set;
    private final List<Double> zeroTimes;
    private final PIDConfig config;
    private RobotShuffleboardTab tab;

    public AutoTuner(BaseTalon motor, double set) {
        this.motor = motor;
        this.set = set;
        this.zeroTimes = new LinkedList<>();
        this.config = new PIDConfig(0.5, 0, 0);
    }

    public void withRST(RobotShuffleboardTab tab) {
        this.tab = tab;
        setRSTEntry("isFinalPID", false);
        setRSTEntry("Set", set);
        setRSTEntry("kP", config.getP());
        setRSTEntry("kI", config.getI());
        setRSTEntry("kD", config.getD());
    }

    private <T> void setRSTEntry(String title, T value) {
        if (tab != null) {
            setRSTEntry(title, value);
        }
    }

    private void getZeros() {
        config.applyTo(motor, 0);
        int initPos = (int) motor.getSelectedSensorPosition();
        int pos;
        double startTime = Timer.getFPGATimestamp();
        double currTime;
        double lastTime = startTime;
        motor.set(ControlMode.MotionMagic, set);
        while ((currTime = Timer.getFPGATimestamp()) < startTime + RUN_TIME) {
            pos = (int) motor.getSelectedSensorPosition();
            setRSTEntry("pos", pos);
            if (MathUtil.isWithinTolerance(pos, set, MAX_POS_ERR)) {
                zeroTimes.add(currTime - lastTime);
                lastTime = currTime;
            }
        }
        motor.set(ControlMode.MotionMagic, initPos);
    }

    private void calculateGains() {
        double oscPeriod = 0;

        for (int i = 1; i < zeroTimes.size(); i += 2) {
            oscPeriod += zeroTimes.get(i - 1) + zeroTimes.get(i);
        }
        oscPeriod /= zeroTimes.size();
        // Magic constants from Wikipedia page "no overshoot"
        double maxKP = config.getP();
        config.setP(0.2 * maxKP).setI(0.4 * maxKP / oscPeriod).setD(0.066 * maxKP * oscPeriod);
        setRSTEntry("oscPeriod", oscPeriod);
        setRSTEntry("isFinalPID", true);
        setRSTEntry("kP", config.getP());
        setRSTEntry("kI", config.getI());
        setRSTEntry("kD", config.getD());
    }

    private boolean ensureOscillation() {
        double tDiff;
        double lastTDiff = 0;
        for (int i = 1; i < zeroTimes.size(); i++) {
            tDiff = zeroTimes.get(i) - zeroTimes.get(i - 1);
            if (tDiff - lastTDiff > TIME_ERR_BOUNDS.getB()) {
                config.setP(config.getP() * (1 - KP_MOD_FACTOR));
                setRSTEntry("oscState", "high");
                return false;
            } else if (tDiff - lastTDiff < TIME_ERR_BOUNDS.getA()) {
                config.setP(config.getP() * (1 + KP_MOD_FACTOR));
                setRSTEntry("oscState", "low");
                return false;
            }
            lastTDiff = tDiff;
        }
        setRSTEntry("oscState", "stable");
        return true;
    }

    public PIDConfig tune() {
        while (zeroTimes.size() == 0 || !ensureOscillation()) {
            getZeros();
        }
        calculateGains();
        config.applyTo(motor, 0);
        return config;
    }
}
