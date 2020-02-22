package org.rivierarobotics.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

public class EjectorSide {
    private final WPI_TalonSRX ejectorTalon;

    public EjectorSide(int id, boolean invert) {
        ejectorTalon = new WPI_TalonSRX(id);
        ejectorTalon.configFactoryDefault();
        ejectorTalon.setNeutralMode(NeutralMode.Brake);
        ejectorTalon.setInverted(invert);
    }

    public void setPower(double pwr) {
        ejectorTalon.set(pwr);
    }
}
