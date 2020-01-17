package org.rivierarobotics.subsystems;

import edu.wpi.first.wpilibj.Solenoid;
import org.rivierarobotics.util.RobotMap;

public class Piston {
    private Solenoid sol;

    public Piston() {
        //TODO make this look something like
        // https://github.com/Team5818/ShermanCrab-2019/blob/master/src/main/java/org/rivierarobotics/subsystems/PistonController.java
        // and call this PistonController instead of just Piston, Piston implies one piston but controller can hold mutliple and be a subsystem
        sol = new Solenoid(RobotMap.CODRIVER_LEFT_JS);
    }

    public void extend(Boolean extend) {
        sol.set(extend);
    }
}
