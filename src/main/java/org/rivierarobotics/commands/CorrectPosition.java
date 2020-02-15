package org.rivierarobotics.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import net.octyl.aptcreator.GenerateCreator;
import org.rivierarobotics.util.PositionTracker;

import javax.inject.Inject;

public class CorrectPosition extends InstantCommand {

    PositionTracker tracker;
    @Inject
    public CorrectPosition(PositionTracker tracker) {
        this.tracker = tracker;
    }

    @Override
    public void execute() {
        tracker.correctPosition();
    }

}
