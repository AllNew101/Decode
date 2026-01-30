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

    public void RGB_SETUP(Gamepad current, int r, int g, int b, int duration){current.setLedColor(r, g, b, duration);}
    public void RGB_SETUP(Gamepad current, String color, int duration){
        switch (color.toLowerCase()) { 
            case "red":
                current.setLedColor(255, 0, 0, duration);
                break;
            case "green":
                current.setLedColor(0, 255, 0, duration);
                break;
            case "blue":
                current.setLedColor(0, 0, 255, duration);
                break;
            case "yellow":
                current.setLedColor(255, 255, 0, duration);
                break;
            case "purple":
                current.setLedColor(255, 0, 255, duration);
                break;
            case "cyan":
                current.setLedColor(0, 255, 255, duration);
                break;
            case "white":
                current.setLedColor(255, 255, 255, duration);
                break;
            default:
                current.setLedColor(0, 0, 0, duration);
                break;
        }}






}
