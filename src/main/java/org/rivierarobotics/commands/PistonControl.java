package org.rivierarobotics.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import org.rivierarobotics.robot.Robot;
import org.rivierarobotics.subsystems.Piston;
import org.rivierarobotics.subsystems.PistonController;

public class PistonControl extends InstantCommand {
    private final PistonController pistc;
    private final Piston p;
    private final boolean extendPiston;

    public PistonControl(boolean extendPiston, Piston p) {
        this.p = p;
        this.extendPiston = extendPiston;
        this.pistc = Robot.runningRobot.pistonController;
    }

    @Override
    public void execute() {
        pistc.operatePiston(p,extendPiston);
    }
}
