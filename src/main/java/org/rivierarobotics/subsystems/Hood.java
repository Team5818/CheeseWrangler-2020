package org.rivierarobotics.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.controller.PIDController;
import org.rivierarobotics.commands.TurretControlPrototype;
import org.rivierarobotics.util.RobotMap;

public class Hood extends BasePIDSubsystem {

    private final WPI_TalonSRX hoodTalon;
    private final WPI_TalonSRX flywheelTalon;
    private final PIDController pidController;


    public Hood() {
        super(0.0004, 0, 0.0001, 1, 4096.0/360.0);
        hoodTalon = new WPI_TalonSRX(RobotMap.HOOD_TALON);
        hoodTalon.configFactoryDefault();
        hoodTalon.setSensorPhase(false);
        hoodTalon.configSelectedFeedbackSensor(FeedbackDevice.PulseWidthEncodedPosition);
        hoodTalon.configFeedbackNotContinuous(false, 10);
        flywheelTalon = new WPI_TalonSRX(RobotMap.FLYWHEEL_TALON);
        pidController = new PIDController(0.0004, 0, 0.0001);

    }

    public double getPosition() {
        var pos = hoodTalon.getSensorCollection().getPulseWidthPosition();
        return pos;
    }

    @Override
    public void setPower(double pwr) {

    }


    public void setHoodPosition(double power) {
        hoodTalon.set(power / 2);
    }

    public void setFlywheelPower(double power) {
        flywheelTalon.set(power);
    }

    public void setPosition(int position) {
        pidController.setSetpoint(position);
    }


}
