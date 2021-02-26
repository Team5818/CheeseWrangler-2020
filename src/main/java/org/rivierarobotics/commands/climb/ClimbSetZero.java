package org.rivierarobotics.commands.climb;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import net.octyl.aptcreator.GenerateCreator;
import net.octyl.aptcreator.Provided;
import org.rivierarobotics.subsystems.Climb;

@GenerateCreator
public class ClimbSetZero extends CommandBase {
    private final Climb climb;

    public ClimbSetZero (@Provided Climb climb){
        this.climb = climb;
    }

    @Override
    public void execute() {
        climb.setPower(0.1);
    }

    @Override
    public boolean isFinished() {
        return climb.isAtBottom();
    }

    @Override
    public void end(boolean interrupted) {
        climb.resetEncoder();
        climb.setPower(0);
    }
}
