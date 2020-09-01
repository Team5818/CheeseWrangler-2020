package org.rivierarobotics.commands.cheesewheel;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import net.octyl.aptcreator.GenerateCreator;
import net.octyl.aptcreator.Provided;
import org.rivierarobotics.subsystems.CheeseWheel;

@GenerateCreator
public class CWCycleSlotInterrupt extends InstantCommand {
    private final CheeseWheel cheeseWheel;
    private final CheeseWheel.Direction direction;
    private final CheeseWheel.AngleOffset mode;
    private final boolean requireOpen;

    public CWCycleSlotInterrupt(@Provided CheeseWheel cheeseWheel, CheeseWheel.Direction direction, CheeseWheel.AngleOffset mode, boolean requireOpen) {
        this.cheeseWheel = cheeseWheel;
        this.direction = direction;
        this.mode = mode;
        this.requireOpen = requireOpen;
    }

    @Override
    public void execute() {
        new CWCycleSlot(cheeseWheel, direction, mode, requireOpen).schedule();
    }
}