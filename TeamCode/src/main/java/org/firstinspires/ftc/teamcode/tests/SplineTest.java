/*
This is a modified version of SplineTest.java from the ACME Robotics Road-Runner-Quickstart project.
Included with permission from ACME Robotics.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
 */

package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robot.BotBuildersMecBot;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Autonomous(name = "SplineTest", group = "drive")
@Disabled
public class SplineTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
//        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        BotBuildersMecBot drive = new BotBuildersMecBot(hardwareMap);

        waitForStart();

        if (isStopRequested()) return;

       Trajectory traj = drive.trajectoryBuilder(new Pose2d())
               .splineTo(new Vector2d(30, 30), 0)
                //.splineTo(new Pose2d(30, 30, 0))
                .build();

        /*Trajectory traj = new TrajectoryBuilder(new Pose2d(), drive.constraints)
                .splineTo(new Vector2d(30, 30), 0)
                .build();*/

        drive.followTrajectory(traj);

        sleep(2000);

//        drive.followTrajectory(
//                drive.trajectoryBuilder(new Pose2d(30, 30, Math.toRadians(0)))
//                        .splineTo(new Pose2d(0, 0, Math.toRadians(180)))
//                        .build()
//        );



    }
}
