package org.rivierarobotics.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import org.rivierarobotics.util.RobotMap;

public class Flywheel extends BasePIDSubsystem{
    private final WPI_TalonSRX flywheelTalon;

    public Flywheel(){
        super(0,0,0,1,4096/360.0);
        flywheelTalon = new WPI_TalonSRX(RobotMap.FLYWHEEL_TALON);
    }

    public void setFlywheelPower(double power) {
        flywheelTalon.set(power);
    }

    @Override
    public double getPosition() {
        return flywheelTalon.getSensorCollection().getPulseWidthVelocity();
    }

    @Override
    public void setPower(double pwr) {
        flywheelTalon.set(pwr);
    }
}
