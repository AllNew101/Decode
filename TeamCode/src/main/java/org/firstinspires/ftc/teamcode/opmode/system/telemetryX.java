package org.firstinspires.ftc.teamcode.opmode.system;



import org.firstinspires.ftc.robotcore.external.Telemetry;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.acmerobotics.dashboard.FtcDashboard;


@Disabled
public class telemetryX extends LinearOpMode {
    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();

    public void addData(String head, String child){
        dashboardTelemetry.addData(head,child);
        //telemetry.addData(head,child);
    }

    public void update(){
        dashboardTelemetry.update();
        //telemetry.update();
    }
    @Override
    public void runOpMode() {}
}

