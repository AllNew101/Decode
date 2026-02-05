package org.firstinspires.ftc.teamcode.opmode.system;



import org.firstinspires.ftc.robotcore.external.Telemetry;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.acmerobotics.dashboard.FtcDashboard;


public class telemetryX {
    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();
    Telemetry telemetry;

    public void init(Telemetry t){
        telemetry = t;
    }
    public void addData(String head, Object child, int key){
        switch (key) {
            case 0:
                dashboardTelemetry.addData(head, child);
                break;
            case 1:
                telemetry.addData(head, child);
                break;
            case 2:
                dashboardTelemetry.addData(head, child);
                telemetry.addData(head, child);
                break;
        }
    }

    public void update(){
        dashboardTelemetry.update();
        telemetry.update();
    }
    public void clear(){
        dashboardTelemetry.clear();
        telemetry.clear();
    }
}

