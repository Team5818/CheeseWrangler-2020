package org.rivierarobotics.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import org.rivierarobotics.util.ColorWheelColor;

public class ColorWheel extends BasePIDSubsystem {
    private final WPI_TalonSRX ColorWheelTalon;
    private ColorWheelColor colorInit;
    //initial color
    public final double colorWheelRadius;
    //TODO Input some kind of Color sensor

    public ColorWheel(int id, double colorWheelRadius) {
        super( 0.0, 0.0, 0.0, 1);
        ColorWheelTalon = new WPI_TalonSRX(id);
        ColorWheelTalon.configFactoryDefault();
        ColorWheelTalon.setNeutralMode(NeutralMode.Brake);
        this.colorWheelRadius = colorWheelRadius;
    }

    @Override
    public double getPositionTicks() {
        return ColorWheelTalon.getSensorCollection().getQuadratureVelocity();
    }

    @Override
    protected void setPower(double pwr) {
        ColorWheelTalon.set(pwr);
    }
}