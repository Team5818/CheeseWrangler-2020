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
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.controller.LinearQuadraticRegulator;
import edu.wpi.first.wpilibj.estimator.KalmanFilter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.system.LinearSystem;
import edu.wpi.first.wpilibj.system.LinearSystemLoop;
import edu.wpi.first.wpilibj.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpiutil.math.Nat;
import edu.wpi.first.wpiutil.math.VecBuilder;
import edu.wpi.first.wpiutil.math.numbers.N1;
import org.rivierarobotics.appjack.Logging;
import org.rivierarobotics.appjack.MechLogger;
import org.rivierarobotics.util.MathUtil;
import org.rivierarobotics.util.MotorUtil;
import org.rivierarobotics.util.RSTab;
import org.rivierarobotics.util.RobotShuffleboard;
import org.rivierarobotics.util.ShooterConstants;

/**
 * Subsystem for flywheel. Spins at a velocity and shoots balls previously
 * pushed by ejectors from Cheese Wheel.
 */
public class Flywheel extends SubsystemBase implements RRSubsystem {
    private static final double MIN_SHOOTING_VEL = 1000.0;
    private static double targetVel = 0;
    private static double tolerance = 70;
    private final WPI_TalonFX flywheelFalconLeft;
    private final WPI_TalonFX flywheelFalconRight;
    private final MechLogger logger;
    private final RSTab tab;
    private final LinearSystemLoop<N1, N1, N1> flywheelStateSpaceDriver;

    public Flywheel(int leftID, int rightID, RobotShuffleboard shuffleboard) {
        this.logger = Logging.getLogger(getClass());
        this.tab = shuffleboard.getTab("Vision");

        this.flywheelFalconLeft = new WPI_TalonFX(leftID);
        this.flywheelFalconRight = new WPI_TalonFX(rightID);

        // States: [velocity], in radians per second.
        // Inputs (what we can "put in"): [voltage], in volts.
        // Outputs (what we can measure): [velocity], in radians per second.
        // kV = volts/radian/s kA = volts/radian/s*s
        LinearSystem<N1, N1, N1> flywheelPlant = LinearSystemId.identifyVelocitySystem(.3800, .1016);

        KalmanFilter<N1, N1, N1> observer = new KalmanFilter<>(
                Nat.N1(), Nat.N1(),
                flywheelPlant,
                VecBuilder.fill(3.0), // How accurate we think our model is
                VecBuilder.fill(0.01), // How accurate we think our encoder data is
                0.020);

        LinearQuadraticRegulator<N1, N1, N1> controller
                = new LinearQuadraticRegulator<>(flywheelPlant,
                VecBuilder.fill(8.0), //Velocity error tolerance, in radians per second
                VecBuilder.fill(12.0), // Control effort (voltage) tolerance.
                0.020);


        //plant State-space plant.
        //controller State-space controller.
        //observer State-space observer.
        //maxVoltageVolts The maximum voltage that can be applied. Commonly 12.
        //dtSeconds The nominal timestep.
        this.flywheelStateSpaceDriver = new LinearSystemLoop<>(
                flywheelPlant,
                controller,
                observer,
                12.0,
                0.020);

        flywheelFalconLeft.setInverted(true);
        flywheelFalconLeft.setNeutralMode(NeutralMode.Coast);
        flywheelFalconRight.follow(flywheelFalconLeft);
        flywheelFalconRight.setInverted(InvertType.OpposeMaster);
        flywheelFalconRight.setNeutralMode(NeutralMode.Coast);
    }

    @Override
    public double getPositionTicks() {
        return flywheelFalconLeft.getSelectedSensorVelocity();
    }

    /**
     * Get the velocity that a ball will have at the current velocity.
     *
     * @return the ball velocity in meters per second.
     */
    public double getBallVelocity() {
        return ShooterConstants.ticksToVelocity(flywheelFalconLeft.getSelectedSensorVelocity());
    }

    @Override
    public void setPower(double pwr) {
        logger.powerChange(pwr);
        flywheelFalconLeft.set(ControlMode.PercentOutput, pwr);
    }

    /**
     * Check if the velocity is within tolerance of the target.
     *
     * @return if the velocity is within tolerance of target.
     */
    public boolean withinTolerance() {
        return MathUtil.isWithinTolerance(getPositionTicks(), targetVel, tolerance);
    }

    /**
     * Change the flywheel pre-shoot tolerance. A lower value will cause more
     * accurate shots with a longer spool-up time. Minimum 0.
     *
     * @param amount the offset amount to change the tolerance by.
     */
    public void stepTolerance(int amount) {
        tolerance = Math.max(0, tolerance + amount);
        tab.setEntry("SetTolerance", tolerance);
    }

    public static double getTolerance() {
        return tolerance;
    }

    /**
     * Set the velocity of the flywheel. Controlled by PIDF loop. Will cut
     * off current and set velocity to zero if stopped. Passed velocity is
     * automatically set as target.
     *
     * @param vel the velocity in ticks per 100ms to set.
     */
    public void setVelocity(double vel) {
        tab.setEntry("Flywheel Set Vel", vel);
        SmartDashboard.putNumber("flywheel target", Math.toRadians(MathUtil.ticksToDegrees(vel)));
        flywheelStateSpaceDriver.setNextR(VecBuilder.fill(Math.toRadians(MathUtil.ticksToDegrees(vel))));
    }

    public double getTargetVel() {
        return targetVel;
    }

    public boolean isBelowMinShootingVel() {
        return getPositionTicks() < MIN_SHOOTING_VEL;
    }

    public MotorTemp getTemp() {
        return new MotorTemp(flywheelFalconLeft.getDeviceID(), flywheelFalconLeft.getTemperature(), "FlywheelFalcon");
    }

    public void setVoltage(double v) {
        flywheelFalconLeft.setVoltage(v);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("at velocity", Math.toRadians(MathUtil.ticksToDegrees(flywheelFalconLeft.getSelectedSensorVelocity())));
        flywheelStateSpaceDriver.correct(VecBuilder.fill(Math.toRadians(MathUtil.ticksToDegrees(flywheelFalconLeft.getSelectedSensorVelocity()))));
        flywheelStateSpaceDriver.predict(0.020);
        double nextVoltage = flywheelStateSpaceDriver.getU(0);
        SmartDashboard.putNumber("Target V", nextVoltage);
        flywheelFalconLeft.setVoltage(Math.max(0, nextVoltage));
    }
}
