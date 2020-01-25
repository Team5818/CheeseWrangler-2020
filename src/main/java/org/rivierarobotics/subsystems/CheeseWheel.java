package org.rivierarobotics.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj2.command.Subsystem;
import org.rivierarobotics.util.RobotMap;

public class CheeseWheel extends BasePID implements Subsystem {
    private final WPI_TalonSRX cheeseTalon;

    public CheeseWheel(){
        super(0.0,0.0,0.0,0.0,0.0);
        cheeseTalon = new WPI_TalonSRX(RobotMap.Controllers.CHEESE_TALON);
        cheeseTalon.configFactoryDefault();
        cheeseTalon.setNeutralMode(NeutralMode.Brake);
        getPidController().enableContinuousInput(0,4096);
    }

    @Override
    public double getPositionTicks() { return cheeseTalon.getSensorCollection().getPulseWidthVelocity();}

    @Override
    protected void setPower(double pwr) { cheeseTalon.set(pwr); }
}
