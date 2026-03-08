package org.firstinspires.ftc.teamcode.opmode.Indev;
import com.acmerobotics.dashboard.config.Config;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;



public class Intake_sup_servo {
    public Servo sup_servo;

    public void int_sup_servo (HardwareMap hardwareMap) {
        sup_servo = hardwareMap.get(Servo.class, "Front-keeper");
    }

    public double get_position (){return sup_servo.getPosition();}
    public void set_servo (double position) {sup_servo.setPosition(position);}

}
