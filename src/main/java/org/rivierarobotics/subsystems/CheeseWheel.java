package org.rivierarobotics.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import org.rivierarobotics.util.RobotMap;

public class CheeseWheel extends BasePIDSubsystem {
    private final WPI_TalonSRX wheelTalon;
    private final double baseTicks = 0;

    public CheeseWheel() {
        super(0.0, 0.0, 0.0, 1.0);
        this.wheelTalon = new WPI_TalonSRX(RobotMap.Controllers.CHEESE_TALON);
        wheelTalon.configFactoryDefault();
        wheelTalon.setNeutralMode(NeutralMode.Brake);
    }

    public void setIndex(int index, boolean side) {
        super.setPositionTicks(((360.0 / 5) * index) + (baseTicks * ((side) ? -1 : 1)));
    }

    @Override
    public double getPositionTicks() {
        return wheelTalon.getSensorCollection().getPulseWidthPosition();
    }

    @Override
    protected void setPower(double pwr) {
        wheelTalon.set(pwr);
    }
}
