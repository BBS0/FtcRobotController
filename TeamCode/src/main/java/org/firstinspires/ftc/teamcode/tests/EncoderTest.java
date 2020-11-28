package org.firstinspires.ftc.teamcode.tests;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.PracticeMecanumDrive;


import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;




@TeleOp(name="Encoder", group="Test")
//@Disabled
public class EncoderTest extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();



    @Override
    public void runOpMode() {


        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        DcMotor fleft = hardwareMap.get(DcMotor.class,"front_left_motor");

        DcMotor fright = hardwareMap.get(DcMotor.class,"front_right_motor");

        DcMotor bright = hardwareMap.get(DcMotor.class,"back_right_motor");

        DcMotor bleft = hardwareMap.get(DcMotor.class,"back_left_motor");

        DcMotor shooter = hardwareMap.get(DcMotor.class,"shooter");

        DcMotor right_enc = hardwareMap.get(DcMotor.class,"intake");
        DcMotor left_enc = hardwareMap.get(DcMotor.class,"left_enc");
        DcMotor center_enc = hardwareMap.get(DcMotor.class,"right_enc");

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {


            telemetry.addData("F Left", fleft.getCurrentPosition());
            telemetry.addData("F Right", fright.getCurrentPosition());
            telemetry.addData("B Right", bright.getCurrentPosition());
            telemetry.addData("B Left", bleft.getCurrentPosition());
            telemetry.addData("Shooter", shooter.getCurrentPosition());

            telemetry.addData("L Enc", left_enc.getCurrentPosition());
            telemetry.addData("R Enc", right_enc.getCurrentPosition());
            telemetry.addData("C Enc", center_enc.getCurrentPosition());


            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();
        }
    }
}

