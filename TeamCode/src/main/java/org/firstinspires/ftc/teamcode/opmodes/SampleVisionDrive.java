package org.firstinspires.ftc.teamcode.opmodes;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.OpenCvDetector;

@Autonomous
public class SampleVisionDrive extends OpenCvDetector {

    @Override
    public void runOpMode() throws InterruptedException {

        super.runOpMode(); //runs OpenCV program and sets rings to the number of rings it sees
        if (isStopRequested()) return;

        //todo: place your auto code below
        FtcDashboard.getInstance().startCameraStream(webcam, 0);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        //This while loop is just a sample
        while (opModeIsActive())
        {

            telemetry.addData("Rings", rings);
            //telemetry.addData("Threshold", thresholdValue);
            telemetry.update();

            sleep(50);
        }

        FtcDashboard.getInstance().stopCameraStream();


    }
}
