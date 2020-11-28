package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.hardware.motors.SimpleMotor;
import com.arcrobotics.ftclib.kinematics.DifferentialOdometry;
import com.arcrobotics.ftclib.util.Direction;
import com.arcrobotics.ftclib.Robot;
import com.arcrobotics.ftclib.util.Safety;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.drivebase.RobotDrive;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class PracticeMecanumDrive extends Robot {

    private MecanumDrive driveTrain;

    private SimpleMotor frontLeftDrive = null;
    private SimpleMotor frontRightDrive = null;
    private SimpleMotor backLeftDrive = null;
    private SimpleMotor backRightDrive = null;

    public PracticeMecanumDrive(HardwareMap hardwareMap) {

        frontLeftDrive = new SimpleMotor("frontLeftMotor", hardwareMap);
        frontRightDrive = new SimpleMotor("frontRightMotor", hardwareMap);
        backLeftDrive = new SimpleMotor("backLeftMotor", hardwareMap);
        backRightDrive = new SimpleMotor("backRightMotor", hardwareMap);

        backLeftDrive.setInverted(false);
        backRightDrive.setInverted(false);


        this.driveTrain = new MecanumDrive(frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive);

    }

    public void driveRobot(double strafe, double forward, double turn) {
        this.driveTrain.driveRobotCentric(strafe, forward, turn);
    }
}
