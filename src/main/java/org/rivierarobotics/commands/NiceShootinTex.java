package org.rivierarobotics.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import net.octyl.aptcreator.GenerateCreator;
import net.octyl.aptcreator.Provided;
import org.rivierarobotics.subsystems.CheeseWheel;
import org.rivierarobotics.util.CheeseSlot;

@GenerateCreator
public class NiceShootinTex extends CommandBase {
    private final CheeseWheel cheeseWheel;
    private final int slots;
    private CheeseSlot currentSlot;
    private int direction;
    private int shotSlots;

    public NiceShootinTex(@Provided CheeseWheel cheeseWheel,
                          int slots) {
        this.cheeseWheel = cheeseWheel;
        this.slots = slots;
    }

    @Override
    public void initialize() {
        shotSlots = 0;
        currentSlot = cheeseWheel.getClosestSlot(CheeseWheel.Mode.COLLECT_FRONT, CheeseWheel.Filled.DONT_CARE, 0);
        if (!currentSlot.isFilled) {
            // we have to use whatever is closest
            currentSlot = cheeseWheel.getClosestSlot(CheeseWheel.Mode.COLLECT_FRONT, CheeseWheel.Filled.DONT_CARE, 0);
        }
        // assume (from ShootNWedges) that we already moved to this slot!
        // go to the next one...
        currentSlot = currentSlot.next(1);
        var diff = currentSlot.getModePosition(CheeseWheel.Mode.COLLECT_FRONT) - cheeseWheel.getPositionTicks();
        diff = cheeseWheel.correctDiffForGap(diff);
        direction = 1; //(int) Math.signum(diff);

        moveToNext();
    }

    @Override
    public void execute() {
        if (!cheeseWheel.getPidController().atSetpoint()) {
            return;
        }
        shotSlots++;
        currentSlot.isFilled = false;
        currentSlot = currentSlot.next(direction);
        if (!isFinished()) {
            moveToNext();
        }
    }

    private void moveToNext() {
        cheeseWheel.setPositionTicks(currentSlot.getModePosition(CheeseWheel.Mode.COLLECT_FRONT));
    }

    @Override
    public boolean isFinished() {
        return shotSlots == slots;
    }
}
