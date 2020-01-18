package org.rivierarobotics.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.SuppliedValueWidget;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.rivierarobotics.commands.TurretControlPrototype;
import org.rivierarobotics.util.RobotMap;

public class Turret extends BasePIDSubsystem {
    private final SuppliedValueWidget<Double> entry;
    private final NetworkTableEntry power = Shuffleboard.getTab("Turret")
            .add("Power", 0.0).getEntry();
    private final WPI_TalonSRX turretTalon;

    public Turret() {
        super(0.0004, 0, 0.0001, 1, 4096/360);
        turretTalon = new WPI_TalonSRX(RobotMap.TURRET_TALON);
        turretTalon.configFactoryDefault();
        turretTalon.setSensorPhase(false);
        turretTalon.configSelectedFeedbackSensor(FeedbackDevice.PulseWidthEncodedPosition);
        turretTalon.configFeedbackNotContinuous(false, 10);
		//pidController.enableContinuousInput(0, 4095);
        entry = Shuffleboard.getTab("Turret")
                .addNumber("Position", this::getPosition);
    }

    public void rotateTurret(double power) {
        this.power.setNumber(power);
        setPower(power);
    }


    public double getPosition() {
        var pos = turretTalon.getSensorCollection().getPulseWidthPosition();
//		while (pos < 0) {
//			pos += 4096;
//		}
//		pos %= 4096;
        return pos;
    }

    @Override
    public void setPower(double pwr) {
        turretTalon.set(pwr);
    }

    public void setPosition(int position) {
        super.setPosition(position);
    }
}
