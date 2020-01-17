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

public class Turret extends SubsystemBase {
    private final SuppliedValueWidget<Double> entry;
    private final NetworkTableEntry power = Shuffleboard.getTab("Turret")
            .add("Power", 0.0).getEntry();
    private final WPI_TalonSRX turretTalon;
    private final WPI_TalonSRX hoodTalon;
    private final WPI_TalonSRX flywheelTalon;
    private final PIDController pidController;

    public Turret() {
        turretTalon = new WPI_TalonSRX(RobotMap.TURRET_TALON);
        hoodTalon = new WPI_TalonSRX(RobotMap.HOOD_TALON);
        flywheelTalon = new WPI_TalonSRX(RobotMap.FLYWHEEL_TALON);
        turretTalon.configFactoryDefault();
        turretTalon.setSensorPhase(false);
        turretTalon.configSelectedFeedbackSensor(FeedbackDevice.PulseWidthEncodedPosition);
        turretTalon.configFeedbackNotContinuous(false, 10);
        pidController = new PIDController(0.0004, 0, 0.0001);
//		pidController.enableContinuousInput(0, 4095);
        setDefaultCommand(new TurretControlPrototype(this));
        entry = Shuffleboard.getTab("Turret")
                .addNumber("Position", this::getPosition);
        //TODO split turret and hood/flywheel into separate subsystems with their own PID loops
        //TODO base both subsystems on BasePIDSubsystem & cleanup
        //TODO setHoodPosition should be setPosition if on separate subsystems
    }

    public void rotateTurret(double power) {
        this.power.setNumber(power);
        turretTalon.set(power);
    }

    public void setHoodPosition(double power) {
        hoodTalon.set(power / 2);
    }

    public void setFlywheelPower(double power) {
        flywheelTalon.set(power);
    }

    public int getPosition() {
        var pos = turretTalon.getSensorCollection().getPulseWidthPosition();
//		while (pos < 0) {
//			pos += 4096;
//		}
//		pos %= 4096;
        return pos;
    }

    public void tickPID() {
        var output = pidController.calculate(getPosition());
        double p = Math.min(0.4, Math.max(-0.4, output));
        rotateTurret(p);
    }


    public void setPosition(int position) {
        pidController.setSetpoint(position);
    }
}
