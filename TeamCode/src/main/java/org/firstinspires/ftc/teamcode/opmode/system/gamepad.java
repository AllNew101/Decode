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

    public void RGB_SETUP(Gamepad current, int r, int g, int b, int duration){
        current.setLedColor(r, g, b, duration);
    }







}
