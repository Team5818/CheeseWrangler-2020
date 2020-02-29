package org.rivierarobotics.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import net.octyl.aptcreator.GenerateCreator;
import net.octyl.aptcreator.Provided;
import org.rivierarobotics.subsystems.CheeseWheel;
import org.rivierarobotics.subsystems.Flywheel;
import org.rivierarobotics.util.MathUtil;
import org.rivierarobotics.util.VisionTarget;

@GenerateCreator
public class ShootNWedges extends ParallelRaceGroup {

    public ShootNWedges(@Provided EjectorCommands ejectorCommands,
                        @Provided VisionCommands visionAimCommands,
                        @Provided CheeseWheelCommands cheeseWheelCommands,
                        @Provided Flywheel flywheel,
                        VisionTarget visionTarget,
                        int wedges) {
        super(
            new CommandBase() {
                @Override
                public void execute() {
                    flywheel.setVelocity(10_000);
                }

                @Override
                public void end(boolean interrupted) {
                    flywheel.setVelocity(0);
                }
            },
            new SequentialCommandGroup(
                cheeseWheelCommands.moveToFreeIndex(CheeseWheel.Mode.COLLECT_FRONT, CheeseWheel.Filled.DONT_CARE),
                ejectorCommands.setPower(1.0),
                new CommandBase() {
                    @Override
                    public boolean isFinished() {
                        return MathUtil.isWithinTolerance(flywheel.getPositionTicks(), 10_000, 500);
                    }
                },
                cheeseWheelCommands.niceShootinTex(wedges)
            )
        );
    }
}
