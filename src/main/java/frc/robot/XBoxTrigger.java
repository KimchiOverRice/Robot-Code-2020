package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.Button;

public class XBoxTrigger extends Button {
    Joystick controller;
    int port;
    public XBoxTrigger(Joystick controller, int port){
        this.controller = controller;
        this.port = port;
    }
    public boolean get(){
        return controller.getRawAxis(port) > 0.2;
    }

}