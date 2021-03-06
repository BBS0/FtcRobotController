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

@Autonomous(name = "RedRightSquares", group = "drive")
public class RedRightSquares extends OpenCvDetector {

    @Override
    public void runOpMode() throws InterruptedException {
        BotBuildersMecBot drive = new BotBuildersMecBot(hardwareMap);
        drive.opMode = this;

        drive.AutoLockGoal();
        super.runOpMode(); //runs OpenCV program and sets rings to the number of rings it sees

        if (isStopRequested()) return;

        drive.AutoArmPickUp();

        Trajectory moveForward = new TrajectoryBuilder(new Pose2d(), drive.constraints)
                .forward(49)
                .build();

        Trajectory moveForwardAgain = new TrajectoryBuilder(new Pose2d(), drive.constraints)
                .forward(5)
                .build();

        Trajectory strafeRight = new TrajectoryBuilder(new Pose2d(), drive.constraints)
                .strafeRight(10)
                .build();

        Trajectory locateA = new TrajectoryBuilder(new Pose2d(), drive.constraints)
                .splineTo(new Vector2d(20, 10), 0)
                .build();

        Trajectory locateB = new TrajectoryBuilder(new Pose2d(), drive.constraints)
                .splineTo(new Vector2d(40, 25), 0)
                .build();

        Trajectory locateC = new TrajectoryBuilder(new Pose2d(), drive.constraints)
                .splineTo(new Vector2d(50, 10), 0)
                .build();



        Trajectory pickedTraj = null;


        telemetry.addData("Rings", rings);
        telemetry.update();

        drive.followTrajectory(moveForwardAgain);
        drive.followTrajectory(strafeRight);
        drive.followTrajectory(moveForward);

        drive.turn(Math.toRadians(39));

        sleep(500);
        drive.TurnJHopShooterOn(215);
        drive.FullAutoShoot();

        drive.turn(Math.toRadians(10));
        sleep(500);
        drive.FullAutoShoot();

        drive.turn(Math.toRadians(10));
        sleep(500);
        drive.FullAutoShoot();
        drive.TurnJHopShooterOff();
        sleep(500);

        drive.turn(Math.toRadians(-45));

        if (rings == GoalDeterminationPipeline.RingPosition.NONE) {
            drive.followTrajectory(locateA);
            pickedTraj = locateA;

            drive.AutoArmDropOff();

            //wait for the servo to disengage
            sleep(1200);

            Trajectory strafeAwayLeft = new TrajectoryBuilder(new Pose2d(), drive.constraints)
                    .strafeLeft(15)
                    .build();

            drive.followTrajectory(strafeAwayLeft);

            drive.AutoArmTop();

            sleep(1000);
            drive.LockArm();
            sleep(1000);
        }
        else if (rings == GoalDeterminationPipeline.RingPosition.ONE) {
            drive.followTrajectory(locateB);
            pickedTraj = locateB;

        }
        else if (rings == GoalDeterminationPipeline.RingPosition.FOUR) {
            drive.followTrajectory(locateC);
            pickedTraj = locateC;
        }



        if(pickedTraj != null) {
            if( rings == GoalDeterminationPipeline.RingPosition.FOUR) {

                drive.turn(Math.toRadians(25));
                drive.AutoArmDropOff();

                //wait for the servo to disengage
                sleep(1200);



                Trajectory strafeAwayLeft = new TrajectoryBuilder(new Pose2d(), drive.constraints)
                        .strafeLeft(10)
                        .build();


                drive.followTrajectory(strafeAwayLeft);


                drive.AutoArmTop();

                sleep(1000);

                drive.turn(Math.toRadians(-25));

                Trajectory rev = new TrajectoryBuilder(new Pose2d(), drive.constraints)
                        .back(35)
                        .build();
                drive.LockArm();
                drive.followTrajectory(rev);



            }else if( rings == GoalDeterminationPipeline.RingPosition.ONE){

                drive.AutoArmDropOff();

                //wait for the servo to disengage
                sleep(1200);

                Trajectory strafeAwayLeft = new TrajectoryBuilder(new Pose2d(), drive.constraints)
                        .strafeLeft(10)
                        .build();

                drive.followTrajectory(strafeAwayLeft);

                drive.AutoArmTop();

                sleep(1000);
                drive.LockArm();
                Trajectory rev = new TrajectoryBuilder(new Pose2d(), drive.constraints)
                        .back(25)
                        .build();

                drive.followTrajectory(rev);


            }
        }
    }
}