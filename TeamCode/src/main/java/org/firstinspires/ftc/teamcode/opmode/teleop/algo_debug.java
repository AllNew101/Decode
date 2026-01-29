package org.firstinspires.ftc.teamcode.opmode.teleop;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.opmode.system.PIDF_Shooter;
import org.firstinspires.ftc.teamcode.opmode.system.telemetryX;
import org.firstinspires.ftc.teamcode.opmode.Calculate.BinarySearch;

@TeleOp
public class algo_debug extends OpMode {
    private PIDF_Shooter Ying;
    private telemetryX telemetryX;
    private BinarySearch binarySearch;
    private double[] x = {1,3,5,7,9};

    @Override
    public void init() {
        Ying = new PIDF_Shooter();
        telemetryX = new telemetryX();
        binarySearch = new BinarySearch(x);
    }

    @Override
    public void loop() {
        telemetryX.addData();
    }
}
