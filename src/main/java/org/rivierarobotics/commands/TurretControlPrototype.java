package org.rivierarobotics.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import org.rivierarobotics.robot.Robot;
import org.rivierarobotics.subsystems.Turret;
import org.rivierarobotics.util.MathUtil;

public class TurretControlPrototype extends CommandBase {
    private final Turret turr;
    private final Joystick leftCoDriverJs;

    public TurretControlPrototype(Turret turr) {

        addRequirements(turr);
        this.turr = turr;
        leftCoDriverJs = Robot.runningRobot.coDriverLeftJs;
    }

    @Override
    public void execute() {
        turr.rotateTurret(MathUtil.fitDeadband(leftCoDriverJs.getX()));
        turr.setHoodPosition(MathUtil.fitDeadband(leftCoDriverJs.getY()));
        turr.setFlywheelPower(MathUtil.fitDeadband(leftCoDriverJs.getTwist()));
    }
}
