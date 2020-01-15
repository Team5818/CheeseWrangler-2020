package org.rivierarobotics.commands;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.CommandBase;
import org.rivierarobotics.subsystems.Turret;
import org.rivierarobotics.util.VisionUtils;

public class VisionAimTurret extends CommandBase {
    private final ShuffleboardTab vision = Shuffleboard.getTab("vision");
    private Turret turret;

    public VisionAimTurret(Turret turret) {
        this.turret = turret;
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
            //TODO change setHoodPosition() to a method that sets a PID setpoint for the hood, copy of the setPosition() one
            //TODO make another PID controller for ^^^^ in Turret
            //TODO change the wording of the ...Position to ...turretPosition
            //TODO change the call to getPosition in setHoodPosition to a new method getHoodPosition based on ^^^
            //TODO move TICKS_TO_DEGREES into Turret and make it public static, move the call to it into the setPosition() method
            //TODO figure out how to calculate ty angle to hood position/angle

            double TICKS_TO_DEGREES = 4096.0 / 360;
            turret.setHoodPosition(turret.getPosition() + TICKS_TO_DEGREES * ty);
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
