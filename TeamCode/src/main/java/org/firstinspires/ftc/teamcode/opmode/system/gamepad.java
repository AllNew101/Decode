package org.firstinspires.ftc.teamcode.opmode.system;
import com.qualcomm.robotcore.hardware.Gamepad;

public class gamepad {
    private Gamepad current;
    public boolean cross(Gamepad current){return current.a || current.cross;}
    public boolean circle(Gamepad current){return current.b || current.circle;}
    public boolean square(Gamepad current){return current.x || current.square;}
    public boolean triangle(Gamepad current){return current.y || current.triangle;}
    public boolean cross_WasPressed(Gamepad current){return current.aWasPressed() || current.crossWasPressed();}
    public boolean circle_WasPressed(Gamepad current){return current.bWasPressed() || current.circleWasPressed();}
    public boolean square_WasPressed(Gamepad current){return current.xWasPressed() || current.squareWasPressed();}
    public boolean triangle_WasPressed(Gamepad current){return current.yWasPressed() || current.triangleWasPressed();}

    public boolean dpad_up(Gamepad current){return current.dpad_up;}
    public boolean dpad_down(Gamepad current){return current.dpad_down;}
    public boolean dpad_left(Gamepad current){return current.dpad_left;}
    public boolean dpad_right(Gamepad current){return current.dpad_right;}
    public boolean dpadUpWasPressed(Gamepad current){return current.dpadUpWasPressed();}
    public boolean dpadDownWasPressed(Gamepad current){return current.dpadDownWasPressed();}
    public boolean dpadLeftWasPressed(Gamepad current){return current.dpadLeftWasPressed();}
    public boolean dpadRightWasPressed(Gamepad current){return current.dpadRightWasPressed();}

    public void RGB_SETUP(Gamepad current, double RGB){}







}
