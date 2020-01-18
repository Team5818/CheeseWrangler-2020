package org.rivierarobotics.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import org.rivierarobotics.robot.Robot;
import org.rivierarobotics.subsystems.Flywheel;
import org.rivierarobotics.subsystems.Hood;
import org.rivierarobotics.subsystems.Turret;
import org.rivierarobotics.util.MathUtil;

public class TurretControlPrototype extends CommandBase {
    private final Turret turr;
    private final Hood ho;
    private final Flywheel fl;
    private final Joystick leftCoDriverJs;

    public TurretControlPrototype(Turret turr, Hood ho, Flywheel fl) {

        addRequirements(turr);
        this.ho = ho;
        this.fl = fl;
        this.turr = turr;
        leftCoDriverJs = Robot.runningRobot.coDriverLeftJs;
    }

    @Override
    public void execute() {
        turr.rotateTurret(MathUtil.fitDeadband(leftCoDriverJs.getX()));
        ho.setPosition(MathUtil.fitDeadband(leftCoDriverJs.getY()));
        fl.setFlywheelPower(MathUtil.fitDeadband(leftCoDriverJs.getTwist()));
    }
}
