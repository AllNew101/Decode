package org.firstinspires.ftc.teamcode.opmode.system;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class Closer {
    private Servo closer;
    public static double close = 0.88;
    public static double open = 0.42;

    public void init_angular(HardwareMap hardwareMap) {
        closer = hardwareMap.get(Servo.class, "tuakan");
    }
    public void open(){
        closer.setPosition(open);
    }
    public void close(){closer.setPosition(close);}
    public void setPosition(double position){closer.setPosition(position);}
}
