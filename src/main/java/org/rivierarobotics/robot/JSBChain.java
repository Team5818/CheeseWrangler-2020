package org.rivierarobotics.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class JSBChain extends JoystickButton {
    public JSBChain(GenericHID joystick, int buttonNumber) {
        super(joystick, buttonNumber);
    }

    public JSBChain onPress(Command cmd) {
        this.whenPressed(cmd);
        return this;
    }

    public JSBChain onRelease(Command cmd) {
        this.whenReleased(cmd);
        return this;
    }

    public JSBChain onHold(Command cmd) {
        this.whileHeld(cmd);
        return this;
    }
}
