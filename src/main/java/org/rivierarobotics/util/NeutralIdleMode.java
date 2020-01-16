package org.rivierarobotics.util;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.SpeedController;

public enum NeutralIdleMode {
    BRAKE(CANSparkMax.IdleMode.kBrake, NeutralMode.Brake),
    COAST(CANSparkMax.IdleMode.kCoast, NeutralMode.Coast);

    public final CANSparkMax.IdleMode spark;
    public final NeutralMode talon;

    NeutralIdleMode(CANSparkMax.IdleMode spark, NeutralMode talon) {
        this.spark = spark;
        this.talon = talon;
    }

    public void applyTo(SpeedController... controllers) {
        for (SpeedController controller : controllers) {
            if (controller instanceof CANSparkMax) {
                ((CANSparkMax) controller).setIdleMode(this.spark);
            } else if (controller instanceof WPI_TalonSRX) {
                ((WPI_TalonSRX) controller).setNeutralMode(this.talon);
            }
        }
    }
}
