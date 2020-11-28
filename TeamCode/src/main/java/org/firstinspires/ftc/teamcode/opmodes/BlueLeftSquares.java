package org.firstinspires.ftc.teamcode.opmodes;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robot.BotBuildersMecBot;
import org.firstinspires.ftc.teamcode.robot.OpenCvDetector;

import static android.icu.lang.UCharacter.DecompositionType.SQUARE;

@Autonomous(name = "BlueLeftSquares", group = "drive")
public class BlueLeftSquares extends OpenCvDetector {

    @Override
    public void runOpMode() throws InterruptedException {
        BotBuildersMecBot drive = new BotBuildersMecBot(hardwareMap);
        drive.opMode = this;

        drive.AutoLockGoal();
        super.runOpMode(); //runs OpenCV program and sets rings to the number of rings it sees

        if (isStopRequested()) return;

        drive.AutoArmPickUp();

        Trajectory moveForward = new TrajectoryBuilder(new Pose2d(), drive.constraints)
                .forward(50)
                .build();

        Trajectory moveForwardAgain = new TrajectoryBuilder(new Pose2d(), drive.constraints)
                .forward(5)
                .build();

        Trajectory strafeRight = new TrajectoryBuilder(new Pose2d(), drive.constraints)
                .strafeRight(-15)
                .build();

        Trajectory locateA = new TrajectoryBuilder(new Pose2d(), drive.constraints)
                .splineTo(new Vector2d(20, 0), 0)
                .build();

        Trajectory locateB = new TrajectoryBuilder(new Pose2d(), drive.constraints)
                .splineTo(new Vector2d(40, -10), 0)
                .build();

        Trajectory locateC = new TrajectoryBuilder(new Pose2d(), drive.constraints)
                .splineTo(new Vector2d(60, 0), 0)
                .build();


        Trajectory pickedTraj = null;


        telemetry.addData("Rings", rings);
        telemetry.update();

        drive.followTrajectory(moveForwardAgain);
        drive.followTrajectory(strafeRight);
        drive.followTrajectory(moveForward);

        drive.turn(Math.toRadians(-12));
        //todo: pauses and other actions are done via markers.
        sleep(1000);
        drive.TurnJHopShooterOn(210);
        drive.FullAutoShoot();

        drive.turn(Math.toRadians(-14));
        sleep(500);
        drive.FullAutoShoot();

        drive.TurnJHopShooterOff();

        drive.turn(Math.toRadians(30));

        if (rings == GoalDeterminationPipeline.RingPosition.NONE) {

            drive.followTrajectory(locateA);
            pickedTraj = locateA;

            Trajectory left = new TrajectoryBuilder(pickedTraj.end(), drive.constraints)
                    .strafeLeft(-10)
                    .build();

            drive.turn(Math.toRadians(-210));
            sleep(100);
            drive.AutoArmDropOff();
            sleep(100);
            drive.followTrajectory(strafeRight);
            sleep(50);
            drive.turn(Math.toRadians(210));

            //wait for the servo to disengage
            sleep(1000);

            //move the arm to the top

            drive.followTrajectory(left);
            drive.AutoArmTop();
        }
        else if (rings == GoalDeterminationPipeline.RingPosition.ONE) {
            drive.followTrajectory(locateB);
            pickedTraj = locateB;

        }
        else if (rings == GoalDeterminationPipeline.RingPosition.FOUR) {
            drive.followTrajectory(locateC);
            pickedTraj = locateC;
        }

        sleep(500);

        telemetry.addData("Path", "Ready to park");
        telemetry.update();

        if(pickedTraj != null) {
            if( rings == GoalDeterminationPipeline.RingPosition.FOUR) {

                drive.turn(Math.toRadians(-210));
                sleep(100);
                drive.AutoArmDropOff();
                sleep(100);
                drive.followTrajectory(strafeRight);
                sleep(50);
                drive.turn(Math.toRadians(210));

                //wait for the servo to disengage
                sleep(1000);

                //move the arm to the top


                /*Trajectory right = new TrajectoryBuilder(pickedTraj.end(), drive.constraints)
                        .strafeLeft(-40)
                        .build();

                drive.followTrajectory(right);*/

                drive.AutoArmTop();

                Trajectory rev = new TrajectoryBuilder(new Pose2d(), drive.constraints)
                        .back(38)
                        .build();

                drive.followTrajectory(rev);

            }else if( rings == GoalDeterminationPipeline.RingPosition.ONE){

                drive.turn(Math.toRadians(-210));
                sleep(100);
                drive.AutoArmDropOff();
                sleep(100);
                drive.followTrajectory(strafeRight);
                sleep(50);
                drive.turn(Math.toRadians(210));

                //wait for the servo to disengage
                sleep(1000);

                //move the arm to the top


                /*Trajectory right = new TrajectoryBuilder(pickedTraj.end(), drive.constraints)
                        .strafeLeft(-10)
                        .build();

                drive.followTrajectory(right);*/
                drive.AutoArmTop();

                Trajectory rev = new TrajectoryBuilder(new Pose2d(), drive.constraints)
                        .back(25)
                        .build();

                drive.followTrajectory(rev);
            }
        }
    }
}