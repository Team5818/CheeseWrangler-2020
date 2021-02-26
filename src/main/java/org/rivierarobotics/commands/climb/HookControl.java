package org.rivierarobotics.commands.climb;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.rivierarobotics.inject.Input;
import org.rivierarobotics.subsystems.Hook;
import org.rivierarobotics.util.MathUtil;

import javax.inject.Inject;

public class HookControl extends CommandBase {
    private final Joystick coDriverLeftJs;
    private final Hook hook;
    private final Joystick driverButtons;


    @Inject
    public HookControl(@Input(Input.Selector.CODRIVER_LEFT) Joystick coDriverLeftJs,@Input(Input.Selector.DRIVER_BUTTONS) Joystick driverButtons, Hook hook){
        this.coDriverLeftJs = coDriverLeftJs;
        this.hook = hook;
        this.driverButtons = driverButtons;
    }

    @Override
    public void execute() {
        if (driverButtons.getRawButton(4)) {
            //Button Number TBD
            hook.setPower(MathUtil.fitDeadband(coDriverLeftJs.getX()));
        }
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}
