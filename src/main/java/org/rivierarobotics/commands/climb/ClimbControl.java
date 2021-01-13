package org.rivierarobotics.commands.climb;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import org.rivierarobotics.inject.Input;
import org.rivierarobotics.subsystems.Climb;
import org.rivierarobotics.util.MathUtil;

import javax.inject.Inject;

public class ClimbControl extends CommandBase {
    private final Climb climb;
    private final Joystick coDriverRightJoystick;
    private final Joystick driverButtons;


    @Inject
    public ClimbControl(@Input(Input.Selector.CODRIVER_RIGHT) Joystick coDriverRightJoystick, @Input(Input.Selector.DRIVER_BUTTONS) Joystick driverButtons, Climb climb){
        this.climb = climb;
        this.coDriverRightJoystick = coDriverRightJoystick;
        this.driverButtons = driverButtons;
    }

    @Override
    public void execute(){
        if (driverButtons.getRawButton(3)) {
            climb.setPower(MathUtil.fitDeadband(coDriverRightJoystick.getX()));
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
