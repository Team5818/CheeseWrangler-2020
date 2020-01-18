package org.rivierarobotics.subsystems;

import edu.wpi.first.wpilibj.Solenoid;
import org.rivierarobotics.util.RobotMap;

public class PistonController{
    private Solenoid test;

    public PistonController() {
        test = new Solenoid(0);
        //TODO more pistons if necessary
    }
    private Solenoid pistonFor(Piston piston){
        if(piston == Piston.TEST) {
            return test;
        }
        return null;
    }

    public void extendPiston(Piston piston) {
        pistonFor(piston).set(true);
    }
    public void retractPiston(Piston piston) {
        pistonFor(piston).set(false);
    }
}
