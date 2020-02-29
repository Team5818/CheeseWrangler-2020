package org.rivierarobotics.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import net.octyl.aptcreator.GenerateCreator;
import net.octyl.aptcreator.Provided;
import org.rivierarobotics.subsystems.CheeseWheel;
@GenerateCreator
public class All5Shoot extends CommandBase {

    private CheeseWheel cheeseWheel;
    private double halfway;
    private boolean doneHalf;
    private double start;

    public All5Shoot(@Provided CheeseWheel cheeseWheel){
        this.cheeseWheel = cheeseWheel;
    }

    @Override
    public void initialize() {
        doneHalf = false;
        start = cheeseWheel.getPositionTicks();
        halfway = (start + 2048) % 4096;

    }

    @Override
    public void execute() {
        if (!doneHalf && cheeseWheel.getPositionTicks() > halfway) {
            doneHalf = true;
        }
        cheeseWheel.setManualPower(0.5);
    }

    @Override
    public boolean isFinished() {
        var pos = cheeseWheel.getPositionTicks();
        return doneHalf && pos >= start && pos <= halfway;
    }
}
