package org.rivierarobotics.subsystems;

public class PidConfig {
    private final double kP;
    private final double kI;
    private final double kD;
    private final double kF;
    private final double tolerance;
    private final double pidRange;

    public PidConfig(double kP, double kI, double kD, double kF, double tolerance, double pidRange) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kF = kF;
        this.tolerance = tolerance;
        this.pidRange = pidRange;
    }

    public PidConfig(double kP, double kI, double kD, double pidRange) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kF = 0;
        this.tolerance = 0;
        this.pidRange = pidRange;
    }

    public double getKP() {
        return kP;
    }

    public double getKI() {
        return kI;
    }

    public double getKD() {
        return kD;
    }

    public double getKF() {
        return kF;
    }

    public double getTolerance() {
        return tolerance;
    }

    public double getPidRange() {
        return pidRange;
    }



}
