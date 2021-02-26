package org.rivierarobotics.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import net.octyl.aptcreator.Provided;
import org.rivierarobotics.commands.climb.HookControl;
import org.rivierarobotics.util.MotorUtil;

import javax.inject.Provider;

public class Hook extends SubsystemBase implements RRSubsystem {
    private final WPI_TalonFX hookTalon;
    private final Provider<HookControl> command;

    public Hook (int motorID, Provider<HookControl> command){
        hookTalon = new WPI_TalonFX(motorID);
        this.command = command;
        MotorUtil.setupMotionMagic(FeedbackDevice.IntegratedSensor,
                new PIDConfig((1023 * 0.1) / 500, 0, 0, (1023.0 * 0.75) / 15900), 0, hookTalon);
        hookTalon.setSensorPhase(true);
        hookTalon.setNeutralMode(NeutralMode.Brake);
    }


    public void setPositionTicks(double positionTicks){
        hookTalon.set(ControlMode.MotionMagic, positionTicks);
    }

    @Override
    public double getPositionTicks() {
        return hookTalon.getSelectedSensorPosition();
    }

    @Override
    public void setPower(double pwr) {
        hookTalon.set(pwr);
    }

//    @Override
//    public void periodic() {
//        if (getDefaultCommand() == null) {
//            setDefaultCommand(command.get());
//        }
//        super.periodic();
//    }
}
