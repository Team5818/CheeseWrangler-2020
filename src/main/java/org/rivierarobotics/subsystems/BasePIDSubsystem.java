package org.rivierarobotics.subsystems;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class BasePIDSubsystem extends SubsystemBase {
    private PIDController pidController;
    private final double pidRange;

    public BasePIDSubsystem(double kP, double kI, double kD, double pidRange) {
        this(kP, kI, kD, pidRange, false, 0, 0);
    }

    public BasePIDSubsystem(double kP, double kI, double kD, double pidRange,
                            boolean enableContinuous, int minContinuous, int maxContinuous) {
        this.pidController = new PIDController(kP, kI, kD);
        this.pidRange = pidRange;
        if(enableContinuous) {
            pidController.enableContinuousInput(minContinuous, maxContinuous);
        }
    }

    public void setPosition(double position) {
        pidController.setSetpoint(position);
    }

    public void tickPid() {
        double pwr = pidController.calculate(getPosition());
        setPower(Math.min(pidRange, Math.max(-pidRange, pwr)));
    }

    public PIDController getPidController() {
        return pidController;
    }

    public abstract void setPower(double pwr);

    public abstract double getPosition();
}
