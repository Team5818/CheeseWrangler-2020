package org.rivierarobotics.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.controller.PIDController;

public class DriveTrainSide {
    private final WPI_TalonSRX masterTalon;
    private final CANSparkMax sparkSlaveOne, sparkSlaveTwo;
    private final PIDController pidLoop;
    private final double kP = 0.0005, kI = 0.0, kD = 0.0, ticksToInches = 1 / 1;

    public DriveTrainSide(int master, int slaveOne, int slaveTwo, boolean invert) {
        this.masterTalon = new WPI_TalonSRX(master);
        this.sparkSlaveOne = new CANSparkMax(slaveOne, CANSparkMaxLowLevel.MotorType.kBrushless);
        this.sparkSlaveTwo = new CANSparkMax(slaveTwo, CANSparkMaxLowLevel.MotorType.kBrushless);
        this.pidLoop = new PIDController(kP, kI, kD);

        masterTalon.setInverted(invert);
        sparkSlaveOne.setInverted(!invert);
        sparkSlaveTwo.setInverted(!invert);

        masterTalon.setNeutralMode(NeutralMode.Brake);
        sparkSlaveOne.setIdleMode(CANSparkMax.IdleMode.kBrake);
        sparkSlaveTwo.setIdleMode(CANSparkMax.IdleMode.kBrake);
    }

    public void setPower(double pwr) {
        if(!pidLoop.atSetpoint()) {
            pwr = pidLoop.calculate(getPositionTicks());
        }
        masterTalon.set(pwr);
        sparkSlaveOne.set(pwr);
        sparkSlaveTwo.set(pwr);
    }

    public double getPositionTicks() {
        return masterTalon.getSensorCollection().getPulseWidthPosition();
    }

    public double getPositionInches() {
        return getPositionTicks() / ticksToInches;
    }

    public void setPosition(double position) {
        pidLoop.setSetpoint(position * ticksToInches);
    }
}
