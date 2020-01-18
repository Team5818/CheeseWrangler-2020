package org.rivierarobotics.commands;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.CommandBase;
import org.rivierarobotics.subsystems.Hood;
import org.rivierarobotics.subsystems.Turret;
import org.rivierarobotics.util.VisionUtils;

public class VisionAimTurret extends CommandBase {
    private final ShuffleboardTab vision = Shuffleboard.getTab("vision");
    private Turret turret;
    private Hood hood;

    public VisionAimTurret(Turret turret) {
        this.turret = turret;
        this.hood = hood;
        addRequirements(turret);
    }

    @Override
    public void execute() {
        double tv = VisionUtils.getLLValue("tv");
        double tx = VisionUtils.getLLValue("tx");
        double ty = VisionUtils.getLLValue("ty");

        getShuffleboardEntry("Valid Target").setBoolean(tv == 1);
        getShuffleboardEntry("X Offset").setDouble(tx);
        getShuffleboardEntry("Y Offset").setDouble(ty);

        if(tv == 1) {

            double TICKS_TO_DEGREES = 4096.0/360;
            hood.setPosition(hood.getPosition() + TICKS_TO_DEGREES * ty);
            turret.setPosition((int) (turret.getPosition() + TICKS_TO_DEGREES * tx));
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    private NetworkTableEntry getShuffleboardEntry(String key) {
        return vision.add(key, 0).getEntry();
    }
}
