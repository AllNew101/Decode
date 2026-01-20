package org.firstinspires.ftc.teamcode.opmode.system;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public class Intake {
    DcMotor Front;
    public double power = 1.0;


    private TelemetryManager telemetryM;

    public void init_intake(HardwareMap hardwareMap) {
        Front = hardwareMap.get(DcMotor.class, "Front");
        Front.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Front.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void intake(double power){
        Front.setPower(power);
        Front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    public void stop_intake(){
        Front.setPower(0);
        Front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

}
