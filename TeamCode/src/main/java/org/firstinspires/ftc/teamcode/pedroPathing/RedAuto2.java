package org.firstinspires.ftc.teamcode.pedroPathing; // make sure this aligns with class location

import static java.lang.Thread.sleep;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "Red Auto Near", group = "Examples")
public class RedAuto2 extends OpMode {

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private HuskyLens huskyLens;
    private Servo fanRotate, cam;
    private DcMotorEx outtake1, outtake2, outtake3, intake, backSpinRoller;
    private double currPosFan = .05, camPos = 1, currRelease=-.01;
    private double fanPos1 = .1, fanPos2 =  .145, fanPos3 = .195, fanPos4 = .24;
    private double upPos1 = .075, upPos2 = .125, upPos3 =.17;
    private boolean x = true;
    private boolean x2 = true;
    private boolean launchStarted = false;
    private int id = -1;
    private int count = 1, targetVel = 1150, rollerVel = 1860;
    private int count2 = 1;
    private int pathState;
    private final Pose startPose = new Pose(107, 131, Math.toRadians(-90)); // Start Pose of our robot.
    private final Pose detectPose = new Pose(77, 70, Math.toRadians(-90));
    private final Pose launchPose = new Pose(85, 93, Math.toRadians(220));
    private final Pose launchOrder = new Pose(80,36, Math.toRadians(0));
    private final Pose order3 = new Pose(92, 56.5, Math.toRadians(0)); // Middle (Second Set) of Artifacts from the Spike Mark.
    private final Pose order3s = new Pose(103,56.5,Math.toRadians(0));
    private final Pose order31 = new Pose(106, 56.5, Math.toRadians(0)); // Middle (Second Set) of Artifacts from the Spike Mark.
    private final Pose order32 = new Pose(112, 56.5, Math.toRadians(0)); // Middle (Second Set) of Artifacts from the Spike Mark.
    private final Pose order2 = new Pose(89, 80.5, Math.toRadians(0)); // Middle (Second Set) of Artifacts from the Spike Mark.
    private final Pose order2s = new Pose(103,80.5,Math.toRadians(0));
    private final Pose order21 = new Pose(106, 80.5, Math.toRadians(0)); // Middle (Second Set) of Artifacts from the Spike Mark.
    private final Pose order22 = new Pose(112, 80.5, Math.toRadians(0)); // Middle (Second Set) of Artifacts from the Spike Mark.
    private final Pose park = new Pose(89, 32.5, Math.toRadians(0));
    private final Pose order1s = new Pose(103,32.5,Math.toRadians(0));
    private final Pose order11 = new Pose(106, 32.5, Math.toRadians(0)); // Middle (Second Set) of Artifacts from the Spike Mark.
    private final Pose order12 = new Pose(111, 32.5, Math.toRadians(0)); // Middle (Second Set) of Artifacts from the Spike Mark.
    // Middle (Second Set) of Artifacts from the Spike Mark.
    private Path detect;
    private PathChain launch, launch3,launch2, moveToOrder3,moveToOrder31,moveToOrder32,moveToOrder3s, moveToOrder2,moveToOrder21,moveToOrder22,moveToOrder2s, parkP, moveToOrder11,moveToOrder12,moveToOrder1s;

    /** This is the main loop of the OpMode, it will run repeatedly after clicking "Play". **/
    public void buildPaths(){
        detect = new Path(new BezierLine(startPose, detectPose));
        detect.setConstantHeadingInterpolation(startPose.getHeading());
        launch = follower.pathBuilder()
                .addPath(new BezierLine(startPose,launchPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), launchPose.getHeading())
                .build();
//        /* This is our scorePickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        moveToOrder3 = follower.pathBuilder()
                .addPath(new BezierCurve(launchPose,launchOrder, order3))
                .setLinearHeadingInterpolation(launchPose.getHeading(), order3s.getHeading())
                .build();
        moveToOrder3s = follower.pathBuilder()
                .addPath(new BezierLine(order3, order3s))
                .setLinearHeadingInterpolation(order3.getHeading(), order3s.getHeading())
                .build();
//        /* This is our grabPickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        moveToOrder31 = follower.pathBuilder()
                .addPath(new BezierCurve(order3s, order31))
                .setLinearHeadingInterpolation(order3s.getHeading(), order31.getHeading())
                .build();
        moveToOrder32 = follower.pathBuilder()
                .addPath(new BezierCurve(order31, order32))
                .setLinearHeadingInterpolation(order31.getHeading(), order32.getHeading())
                .build();
        moveToOrder2 = follower.pathBuilder()
                .addPath(new BezierCurve(launchPose, order2))
                .setLinearHeadingInterpolation(launchPose.getHeading(), order2s.getHeading())
                .build();
        moveToOrder2s = follower.pathBuilder()
                .addPath(new BezierLine(order2, order2s))
                .setLinearHeadingInterpolation(order2.getHeading(), order2s.getHeading())
                .build();
//        /* This is our grabPickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        moveToOrder21 = follower.pathBuilder()
                .addPath(new BezierCurve(order2s, order21))
                .setLinearHeadingInterpolation(order2s.getHeading(), order21.getHeading())
                .build();
        moveToOrder22 = follower.pathBuilder()
                .addPath(new BezierCurve(order21, order22))
                .setLinearHeadingInterpolation(order21.getHeading(), order22.getHeading())
                .build();
        launch2 = follower.pathBuilder()
                .addPath(new BezierLine(order32,launchPose))
                .setLinearHeadingInterpolation(order32.getHeading(), launchPose.getHeading())
                .build();
        launch3 = follower.pathBuilder()
                .addPath(new BezierLine(order22,launchPose))
                .setLinearHeadingInterpolation(order22.getHeading(), launchPose.getHeading())
                .build();
        parkP = follower.pathBuilder()
                .addPath(new BezierLine(launchPose,park))
                .setLinearHeadingInterpolation(launchPose.getHeading(), park.getHeading())
                .build();
        moveToOrder1s = follower.pathBuilder()
                .addPath(new BezierLine(park, order1s))
                .setLinearHeadingInterpolation(park.getHeading(), order1s.getHeading())
                .build();
//        /* This is our grabPickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        moveToOrder11 = follower.pathBuilder()
                .addPath(new BezierCurve(order1s, order11))
                .setLinearHeadingInterpolation(order1s.getHeading(), order11.getHeading())
                .build();
        moveToOrder12 = follower.pathBuilder()
                .addPath(new BezierCurve(order11, order12))
                .setLinearHeadingInterpolation(order11.getHeading(), order12.getHeading())
                .build();
    }
    public void autonomousPathUpdate() throws InterruptedException {


        switch (pathState) {
            case (0):
                follower.followPath(launch);
                setPathState(2);
                break;
            case 1:
                if (!follower.isBusy()) {
                    HuskyLens.Block[] blocks = huskyLens.blocks();
                    sleep(200);
                    if (blocks.length > 0) {
                        telemetry.addData("Block count", blocks.length);
                        telemetry.addData("ID: ", blocks[0].id);
                        id = blocks[0].id;
                        if (blocks[0].id == 1) {
                            setPathState(2);
                            telemetry.update();
                        } else if (blocks[0].id == 2) {
                            setPathState(2);
                            telemetry.update();
                        } else if (blocks[0].id == 3) {
                            setPathState(2);
                            telemetry.update();
                        } else {
                            setPathState(-1);
                            telemetry.update();
                        }
                    }
                }
                break;
            case 2:
            /* You could check for
            - Follower State: "if(!follower.isBusy()) {}"
            - Time: "if(pathTimer.getElapsedTimeSeconds() > 1) {}"
            - Robot Position: "if(follower.getPose().getX() > 36) {}"
            */
                if (!follower.isBusy()) {
                    sleep(500);
                    launchArtifact();
                    runIntake();
                    setPathState(3);
                    x = true; // reset for next state
                }
                break;
            /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
            /* Score Preload */
            /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
//                sleep(1000);
//
//                setPathState(-1);

            case 3:
                if (x) { // trigger path once
                    follower.followPath(moveToOrder3); // start moving
                    x = false;
                }
                if (!follower.isBusy()) {
                    fan1();           // trigger fan after movement is done
                    setPathState(4);  // advance state
                    x = true;         // reset for next state
                }
                break;

            case 4:
                if (x) {
                    follower.followPath(moveToOrder3s);
                    x = false;
                }
                if (!follower.isBusy()) {
                    fan2();
                    setPathState(5);
                    x = true;
                }
                break;

            case 5:
                if (x) {
                    follower.followPath(moveToOrder31);
                    sleep(250);
                    x = false;
                }
                if (!follower.isBusy()) {
                    fan3();
                    setPathState(6);
                    x = true;
                }
                break;

            case 6:
                if (x) {
                    follower.followPath(moveToOrder32);
                    sleep(250);
                    x = false;
                }
                if (!follower.isBusy()) {
                    stopIntake();
                    setPathState(7); // end auto
                    x = true;
                }
                break;
            case 7:
                if (x) { // trigger path once
                    follower.followPath(launch2);
                    fanF();
                    x = false;
                }
                if (!follower.isBusy()) {
                    sleep(500);
                    launchArtifact();

                    runIntake();
                    setPathState(8);
                    x = true; // reset for next state
                }
                break;

            case 8:
                if (x) {
                    follower.followPath(moveToOrder2);
                    sleep(250);
                    x = false;
                }
                if (!follower.isBusy()) {
                    fan1();
                    setPathState(9);
                    x = true;
                }
                break;

            case 9:
                if (x) {
                    follower.followPath(moveToOrder2s);
                    sleep(250);
                    x = false;
                }
                if (!follower.isBusy()) {
                    fan2();
                    setPathState(10);
                    x = true;
                }
                break;

            case 10:
                if (x) {
                    follower.followPath(moveToOrder21);
                    sleep(250);
                    x = false;
                }
                if (!follower.isBusy()) {
                    fan3();
                    setPathState(11);
                    x = true;
                }
                break;

            case 11:
                if (x) {
                    follower.followPath(moveToOrder22);
                    x = false;
                }
                if (!follower.isBusy()) {
                    stopIntake();
                    setPathState(12); // end auto
                    x = true;
                }
                break;

            case 12:
                if (x) {
                    follower.followPath(launch3);
                    fanF();
                    x = false;
                }
                if (!follower.isBusy()) {
                    sleep(500);
                    launchArtifact();
                    runIntake();
                    setPathState(13); // finish auto
                    x = true;
                }
                break;

            case 13:
                if (x) {
                    follower.followPath(parkP);
                    x = false;
                }
                if(!follower.isBusy()){
                    fan1();
                    setPathState(14);
                    x = true;
                }
            case 14:
                if (x) { // trigger path once
                    follower.followPath(moveToOrder1s); // start moving
                    x = false;
                }
                if (!follower.isBusy()) {
                    fan2();           // trigger fan after movement is done
                    setPathState(15);  // advance state
                    x = true;         // reset for next state
                }
                break;

            case 15:
                if (x) {
                    follower.followPath(moveToOrder11);
                    sleep(250);
                    x = false;
                }
                if (!follower.isBusy()) {
                    fan3();
                    setPathState(16);
                    x = true;
                }
                break;

            case 16:
                if (x) {
                    follower.followPath(moveToOrder12);
                    x = false;
                }
                if (!follower.isBusy()) {
                    fan4();
                    setPathState(-1);
                    x = true;
                }
                break;
        }
    }

    /** These change the states of the paths and actions. It will also reset the timers of the individual switches **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }
    @Override
    public void loop() {
        targetVel = 775;
        rollerVel= 2000;
        backSpinRoller.setDirection(DcMotorSimple.Direction.REVERSE);
        outtake1.setVelocity(targetVel);
        outtake2.setVelocity(targetVel);
        backSpinRoller.setVelocity(rollerVel);
        // These loop the movements of the robot, these must be called continuously in order to work
        follower.update();
        try {
            autonomousPathUpdate();
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        // Feedback to Driver Hub for debugging

        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("ID: ", id);
        telemetry.update();
    }

    /** This method is called once at the init of the OpMode. **/
    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();
        huskyLens = hardwareMap.get(HuskyLens.class, "huskylens");
        fanRotate = hardwareMap.get(Servo.class, "fanRotate");
        cam = hardwareMap.get(Servo.class, "cam");
        outtake1 = hardwareMap.get(DcMotorEx.class, "outtake1");
        backSpinRoller = hardwareMap.get(DcMotorEx.class, "outtake2");
        outtake2 = hardwareMap.get(DcMotorEx.class, "outtake3");
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);
        cam.setPosition(camPos);
        try {
            sleep(1000);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        fanRotate.setPosition(upPos1);
        outtake1.setVelocityPIDFCoefficients(20,0,0,20);
        outtake2.setVelocityPIDFCoefficients(20,0,0,20);
    }

    /** This method is called continuously after Init while waiting for "play". **/
    @Override
    public void init_loop() {}

    /** This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system **/
    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }
    public void launchArtifact() throws InterruptedException {
        camUp();
        sleep(500);
        fanRotate.setPosition(upPos2);
        sleep(500);
        camUp();
        sleep(500);
        fanRotate.setPosition(upPos3);
        sleep(500);
        camUp();
    }
    public void fan1(){
        fanRotate.setPosition(fanPos1);
    }
    public void fan2(){
        fanRotate.setPosition(fanPos2);
    }
    public void fan3(){
        fanRotate.setPosition(fanPos3);
    }
    public void fan4(){
        fanRotate.setPosition(fanPos4);
    }
    public void fanF(){
        fanRotate.setPosition(upPos1);
    }
    public void camUp() throws InterruptedException {
        if (camPos == 1) {
            camPos = 0;
        } else if (camPos == 0) {
            camPos = 1;
        }
        cam.setPosition(camPos);
    }
    public void runIntake() throws InterruptedException {
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        intake.setPower(1);
    }
    public void stopIntake(){
        intake.setPower(1);
    }

    /** We do not use this because everything should automatically disable **/
    @Override
    public void stop() {}

    public Pose getFinalPose(){
        return order12;
    }
}