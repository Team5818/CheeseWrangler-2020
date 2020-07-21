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

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.rivierarobotics.commands.turret.TurretControl;
import org.rivierarobotics.robot.Robot;
import org.rivierarobotics.util.*;

import javax.inject.Provider;

public class Turret extends SubsystemBase implements RRSubsystem {
    private static final double ZERO_TICKS = 3793;
    private static final double MAX_ANGLE = 25;
    private static final double MIN_ANGLE = -243.7;
    private final double TICKS_PER_DEGREE = 4096.0 / 360;
    private final double DEGREES_PER_TICK = 360 / 4096;
    private final WPI_TalonSRX turretTalon;
    private final Provider<TurretControl> command;
    private final NavXGyro gyro;
    private final VisionUtil vision;

    public Turret(int id, Provider<TurretControl> command, NavXGyro gyro, VisionUtil vision) {
        this.command = command;
        this.gyro = gyro;
        this.vision = vision;
        turretTalon = new WPI_TalonSRX(id);
        MotorUtil.setupMotionMagic(FeedbackDevice.PulseWidthEncodedPosition,
                new PIDConfig((1.5 * 1023 / 400), 0, 0, 0), 800, turretTalon);
        turretTalon.configFactoryDefault();
        turretTalon.setSensorPhase(false);
        turretTalon.setNeutralMode(NeutralMode.Brake);
        turretTalon.configSelectedFeedbackSensor(FeedbackDevice.PulseWidthEncodedPosition);
    }

    @Override
    public double getPositionTicks() {
        return turretTalon.getSensorCollection().getPulseWidthPosition();
    }

    public double getAbsoluteAngle() {
        return MathUtil.wrapToCircle((getPositionTicks() - ZERO_TICKS) * DEGREES_PER_TICK + (gyro.getYaw()));
    }

    public double getAngle() {
        return (getPositionTicks() - ZERO_TICKS) * DEGREES_PER_TICK;
    }

    public double getTxTurret(double distance, double extraDistance) {
        double tx = Math.toRadians(vision.getLLValue("tx"));
        double txTurret = Math.atan2(distance * Math.sin(tx) + ShooterUtil.getLLtoTurretZ(), distance * Math.cos(tx) + extraDistance);
        Robot.getShuffleboard().getTab("TurretHood").setEntry("txTurret", txTurret);
        return txTurret;
    }

    public void setPositionTicks(int positionTicks) {

        /*
        //!!Experimental Version of Code!!
        //Please do not attempt to move the turret with this code until tested thoroughly. Entry 'PosTicks' should
        //display a value within the limits of the turret in ticks. If not, please don't use the code ;)

        double ticks = MathUtil.limit(
                ZERO_TICKS + positionTicks, ZERO_TICKS + getMinAngleInTicks(), ZERO_TICKS + getMaxAngleInTicks());
        Robot.getShuffleboard().getTab("Turret").setEntry("PosTicks", ticks);
        //turretTalon.set(ControlMode.MotionMagic, ticks);
        */
    }

    public void setAngle(double angle) {
        /*
        double position = getPositionTicks() + ((angle - getAbsoluteAngle()) * TICKS_PER_DEGREE);
        if (position < ZERO_TICKS + getMaxAngleInTicks() && position > ZERO_TICKS + getMinAngleInTicks()) {
            setPositionTicks(position);
        } else if (position - 4096 < ZERO_TICKS + getMaxAngleInTicks() && position > ZERO_TICKS + getMinAngleInTicks()) {
            setPositionTicks(position - 4096);
        }
        */

        /*
        //!!Experimental Version of Code!!
        //Please do not attempt to move the turret with this code until tested thoroughly. Also, the 'ticksAng' entry
        //should display ticks within the limits of the turret. If not, please don't use the code ;)

        Robot.getShuffleboard().getTab("Turret").setEntry("SetTurretAngle", angle);
        double ticks = ZERO_TICKS + (angle * TICKS_PER_DEGREE);
        Robot.getShuffleboard().getTab("Turret").setEntry("SetTurAngInTicks", ticks);

        ticks = MathUtil.limit(ticks, ZERO_TICKS + getMinAngleInTicks(), ZERO_TICKS + getMaxAngleInTicks());
        Robot.getShuffleboard().getTab("Turret").setEntry("SetTicks", ticks);
        //turretTalon.set(ControlMode.MotionMagic, ticks);
         */
    }

    public double getMaxAngleInTicks() {
        return MAX_ANGLE * TICKS_PER_DEGREE;
    }

    public double getMinAngleInTicks() {
        return MIN_ANGLE * TICKS_PER_DEGREE;
    }

    @Override
    public void setPower(double pwr) {
        if (pwr <= 0 && getPositionTicks() - ZERO_TICKS < getMinAngleInTicks()) {
            pwr = 0;
        } else if (pwr > 0 && getPositionTicks() - ZERO_TICKS > getMaxAngleInTicks()) {
            pwr = 0;
        }
        turretTalon.set(ControlMode.PercentOutput, pwr);
    }

    /*
    @Override
    public void setManualPower(double pwr) {
        if (pwr <= 0 && getPositionTicks() - zeroTicks < getMinAngleInTicks()) {
            pwr = 0;
        } else if (pwr > 0 && getPositionTicks() - zeroTicks > getMaxAngleInTicks()) {
            pwr = 0;
        }
        super.setManualPower(pwr);
    }
    */

    @Override
    public void periodic() {
        if (getDefaultCommand() == null) {
            setDefaultCommand(command.get());
        }
        super.periodic();
    }
}