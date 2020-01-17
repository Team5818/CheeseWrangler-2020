package org.rivierarobotics.subsystems;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class BasePIDSubsystem extends SubsystemBase {
    private PIDController pidController;
    private final double pidRange, ticksToAngleOrInches;

    public BasePIDSubsystem(double kP, double kI, double kD, double pidRange, double ticksToAngleOrInches) {
        this(kP, kI, kD, pidRange, ticksToAngleOrInches, false, 0, 0);
    }

    public BasePIDSubsystem(double kP, double kI, double kD, double pidRange, double ticksToAngleOrInches,
                            boolean enableContinuous, int minContinuous, int maxContinuous) {
        this.pidController = new PIDController(kP, kI, kD);
        this.pidRange = pidRange;
        this.ticksToAngleOrInches = ticksToAngleOrInches;
        if(enableContinuous) {
            pidController.enableContinuousInput(minContinuous, maxContinuous);
        }
    }

    public void setPosition(double position) {
        pidController.setSetpoint(position * ticksToAngleOrInches);
    }

    public void tickPid() {
        double pwr = pidController.calculate(getPosition());
        setPower(Math.min(pidRange, Math.max(-pidRange, pwr)));
    }

    public final PIDController getPidController() {
        return pidController;
    }

    public double getPositionInches() {
        return getPosition() / ticksToAngleOrInches;
    }

    public abstract double getPosition();

    public abstract void setPower(double pwr);
}
