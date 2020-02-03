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

package org.rivierarobotics.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.sensors.PigeonIMU;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.rivierarobotics.commands.TurretControl;
import org.rivierarobotics.inject.GlobalComponent;
import org.rivierarobotics.robot.Robot;

import javax.inject.Provider;

public class Turret extends BasePIDSubsystem {
    private static final double zeroticks = 1383;
    private final WPI_TalonSRX turretTalon;
    private final Provider<TurretControl> command;
    private final PigeonIMU pigeon;
    private double [] ypr = new double [3];

<<<<<<< HEAD
    public Turret(int id, Provider<TurretControl> command) {
        super(0.001, 0.0000751, 0.0, 1.0);
=======
    public Turret(int id, Provider<TurretControl> command, PigeonIMU pigeon1) {
        super(0.0005, 0.0045, 0.0, 1.0);
>>>>>>> 1bec35d8f221e30cf0b41b65386bc5e2e2394576
        this.command = command;
        this.pigeon = pigeon1;
        turretTalon = new WPI_TalonSRX(id);
        turretTalon.configFactoryDefault();
        turretTalon.setSensorPhase(false);
        turretTalon.setNeutralMode(NeutralMode.Brake);
        turretTalon.configSelectedFeedbackSensor(FeedbackDevice.PulseWidthEncodedPosition);
//        getPidController().enableContinuousInput(0, 4096);
    }

    @Override
    public double getPositionTicks() {
        double pos = turretTalon.getSensorCollection().getPulseWidthPosition();
        SmartDashboard.putNumber("Position", pos);
        SmartDashboard.putBoolean("atSetpoint", getPidController().atSetpoint());
        SmartDashboard.putNumber("setpoint", getPidController().getSetpoint());
        return pos;
    }

    public double getYaw(){
        pigeon.getYawPitchRoll(ypr);
        return(ypr[0]);
    }

    public double getAbsoluteAngle(){ return ((getPositionTicks() - zeroticks)*360/4096)-getYaw();}

    public void setAbsolutePosition(double angle){
        setPositionTicks((angle - getAbsoluteAngle())*getAnglesOrInchesToTicks()+zeroticks);
    }

    @Override
    public void setPower(double pwr) {
        turretTalon.set(pwr);
    }

    @Override
    public void periodic() {
        if (getDefaultCommand() == null) {
            setDefaultCommand(command.get());
        }
        super.periodic();
    }
}
