
package org.firstinspires.ftc.teamcode.pedroPathing;

import static java.lang.Thread.sleep;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.function.Supplier;

@Configurable
@TeleOp
public class sprTeleopRedCorner extends OpMode {
    private Follower follower;
    private HuskyLens huskyLens;
    private Servo fanRotate, cam, park1, park2;
    private DcMotorEx outtake1, backspinRoller, outtake2;
    private DcMotorSimple intake, rightFront, leftFront, rightRear, leftRear;
    public static Pose startingPose; //See ExampleAuto to understand how to use this
    private boolean automatedDrive, isCam;
    private Supplier<PathChain> pathChain1, pathChain2;
    private TelemetryManager telemetryM;
    ElapsedTime artifactTimer = new ElapsedTime();
    boolean artifactRunning = false;
    int artifactState = 0;

    private boolean slowMode = false;

    private double currPosFan = .05, camPos = 1, currRelease=-.01;
    private double fanPos1 = .1, fanPos2 =  .145, fanPos3 = .195, fanPos4 = .24;
    private double upPos1 = .075, upPos2 = .125, upPos3 =.17;
    private boolean x = true;

    private boolean x2 = true;
    private int count = 1, count3 = 1, targetVel = 1150, rollerVel = 1250;
    private int count2 = 1;
    private double motorPower1 = .63;
    private double fastModeMultiplier = .3;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        MecanumConstants drive = new MecanumConstants();
        follower.setStartingPose(new Pose(10,10, Math.toRadians(-90)));
        follower.update();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        huskyLens = hardwareMap.get(HuskyLens.class, "huskylens");
        fanRotate = hardwareMap.get(Servo.class, "fanRotate");
        cam = hardwareMap.get(Servo.class, "cam");
        park1 = hardwareMap.get(Servo.class, "park1");
        park2 = hardwareMap.get(Servo.class, "park2");
        outtake1 = hardwareMap.get(DcMotorEx.class, "outtake1");
        backspinRoller = hardwareMap.get(DcMotorEx.class, "outtake2");
        outtake2 = hardwareMap.get(DcMotorEx.class, "outtake3");
        leftFront = hardwareMap.get(DcMotorSimple.class, "leftFront");
        leftRear = hardwareMap.get(DcMotorSimple.class, "leftRear");
        rightRear = hardwareMap.get(DcMotorSimple.class, "rightRear");
        rightFront = hardwareMap.get(DcMotorSimple.class, "rightFront");
        intake = hardwareMap.get(DcMotorSimple.class, "intake");
        pathChain1 = () -> follower.pathBuilder() //Lazy Curve Generation
                .addPath(new Path(new BezierLine(follower::getPose, new Pose(85, 19))))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(248), 0.8))
                .build();
        pathChain2 = () -> follower.pathBuilder() //Lazy Curve Generation
                .addPath(new Path(new BezierLine(follower::getPose, new Pose(85, 93))))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(222), 0.8))
                .build();
        outtake1.setVelocityPIDFCoefficients(20,0,0,20);
        outtake2.setVelocityPIDFCoefficients(20,0,0,20);

    }
    @Override
    public void start() {
        //The parameter controls whether the Follower should use break mode on the motors (using it is recommended).
        //In order to use float mode, add .useBrakeModeInTeleOp(true); to your Drivetrain Constants in Constant.java (for Mecanum)
        //If you don't pass anything in, it uses the default (false)
        follower.startTeleopDrive();
        fanRotate.setPosition(currPosFan);
        park1.setPosition(0.1);
        park2.setPosition(0.1);
    }
    @Override
    public void loop() {
        //Call this once per loop
        follower.update();
        telemetryM.update();
        launchArtifact();
        if (!automatedDrive) {
            //Make the last parameter false for field-centric
            //In case the drivers want to use a "slowMode" you can scale the vectors
            //This is the normal version to use in the TeleOp
            follower.setTeleOpDrive(
                    -gamepad1.left_stick_y * fastModeMultiplier,
                    -gamepad1.left_stick_x * fastModeMultiplier,
                    -gamepad1.right_stick_x * fastModeMultiplier,
                    true // Robot Centric
            );
            if (gamepad1.rightStickButtonWasPressed()) {
                fastModeMultiplier = 0.75;
            }
            if(gamepad1.rightStickButtonWasReleased()){
                fastModeMultiplier = .3;
            }
            if(gamepad1.leftStickButtonWasPressed()){
                outtake1.setVelocity(0);
                backspinRoller.setVelocity(0);
                outtake2.setVelocity(0);
            }
            if(gamepad1.dpadUpWasPressed()){
                cam.setPosition(camPos);
                if(camPos == 1){
                    camPos = 0;
                }
                else if(camPos == 0){
                    camPos = 1;
                }
            }
            if(gamepad1.rightBumperWasPressed()){
                if(count == 3 && count2 == 1){
                    fanRotate.setPosition(currPosFan);
                    count = 1;
                }
                else if(count == 1){
                    fanRotate.setPosition(fanPos1);
                    count++;
                }
                else if(count == 2){
                    fanRotate.setPosition(fanPos2);
                    count++;
                }
                else if(count==3){
                    fanRotate.setPosition(fanPos3);
                    count++;
                }
                else if(count == 4){
                    fanRotate.setPosition(fanPos4);
                    count = 1;
                }
            }
            if(gamepad1.leftBumperWasPressed()){
                fanRotate.setPosition(fanRotate.getPosition() - .11);
            }
            if(gamepad1.dpadRightWasPressed()){
                if(count2 ==1){
                    fanRotate.setPosition(upPos3);
                    count2++;
                }
                else if(count2 == 2) {
                    fanRotate.setPosition(upPos2);
                    count2++;
                }
                else if(count2==3){
                    fanRotate.setPosition(upPos1);
                    count2 = 1;

                }
            }
            if(gamepad2.dpadDownWasPressed()){
                park1.setPosition(.75);
                park2.setPosition(.75);
                try {
                    sleep(5000);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }
                while(true){
                    leftRear.setPower(.5);
                    rightRear.setPower(.5);
                }

            }
            if(gamepad2.aWasPressed()){
                park1.setPosition(.25);
                park2.setPosition(.25);
            }
            if(gamepad2.yWasPressed()){
                fanRotate.setPosition(currPosFan);
                count=1;
            }
            if(gamepad1.dpadLeftWasPressed()){
                fanRotate.setPosition(fanRotate.getPosition() - .11);
            }

//            if(gamepad1.rightBumperWasPressed()){
//                currPosFan+=.11;
//                fanRotate.setPosition(currPosFan);
//            } else if (gamepad1.leftBumperWasPressed()) {
//                currPosFan-=.11;
//                fanRotate.setPosition(currPosFan);
//            }
//            if(gamepad1.leftBumperWasPressed()){
//                currPosFan -= .11;
//                fanRotate.setPosition(currPosFan);
//            }

            if(gamepad1.aWasPressed()){
                targetVel=790;
                rollerVel = 1860;
                backspinRoller.setDirection(DcMotorSimple.Direction.REVERSE);
                outtake1.setVelocity(targetVel);
                backspinRoller.setVelocity(rollerVel);
                outtake2.setVelocity(targetVel);
            }
            if(gamepad1.bWasPressed()){
                targetVel = 900;
                rollerVel = 1860;
                backspinRoller.setDirection(DcMotorSimple.Direction.REVERSE);
                outtake1.setVelocity(targetVel);
                backspinRoller.setVelocity(rollerVel);
                outtake2.setVelocity(targetVel);
            }
            if(gamepad1.dpadDownWasPressed() && !artifactRunning){
                startArtifact();
            }
        }
        if(automatedDrive){
            if(gamepad1.aWasPressed()){
                targetVel = 790;
                rollerVel = 1860;
                backspinRoller.setDirection(DcMotorSimple.Direction.REVERSE);
                outtake1.setVelocity(targetVel);
                backspinRoller.setVelocity(targetVel);
                outtake2.setVelocity(targetVel);
            }
            if(gamepad1.bWasPressed()){
                targetVel = 900;
                rollerVel = 1860;
                backspinRoller.setDirection(DcMotorSimple.Direction.REVERSE);
                outtake1.setVelocity(targetVel);
                backspinRoller.setVelocity(rollerVel);
                outtake2.setVelocity(targetVel);
            }
            if(gamepad1.yWasPressed()){
                automatedDrive = false;
                follower.breakFollowing();
                follower.startTeleopDrive();
            }

            if(gamepad1.xWasPressed()){
                automatedDrive = false;
                follower.breakFollowing();
                follower.startTeleopDrive();
            }
        }
        if(gamepad1.left_trigger > 0){
            intake.setDirection(DcMotorSimple.Direction.FORWARD);
            intake.setPower(1);
        }
        else if(gamepad1.left_trigger <= 0){
            intake.setPower(0);
        }
        if(gamepad1.right_trigger > 0){
            intake.setDirection(DcMotorSimple.Direction.REVERSE);
            intake.setPower(1);
        }
        if(gamepad1.right_trigger <= 0){
            intake.setPower(0);
        }

        if(gamepad2.rightBumperWasPressed()){
            targetVel+=10;
            outtake1.setVelocity(targetVel);
            outtake2.setVelocity(targetVel);

        }
        if(gamepad2.leftBumperWasPressed()){
            targetVel-=10;
            outtake1.setVelocity(targetVel);
            outtake2.setVelocity(targetVel);
        }
        if(gamepad1.yWasPressed()){
            automatedDrive = true;
            follower.followPath(pathChain2.get());

        }
        if(gamepad1.xWasPressed()){
            automatedDrive = true;
            follower.followPath(pathChain1.get());

        }
        if (automatedDrive && !follower.isBusy()) {
            automatedDrive = false;
            follower.startTeleopDrive();
        }



        telemetryM.debug("position", follower.getPose());
        telemetryM.debug("velocity", follower.getVelocity());
        telemetryM.debug("automatedDrive", automatedDrive);
        telemetry.addData("Motor Power: ", motorPower1);
        telemetry.addData("Indexer Pos: ", fanRotate.getPosition());
        telemetry.addData("Outtake1", outtake1.getVelocity());
        telemetry.addData("Outtake2", outtake2.getVelocity());
        telemetry.addData("rollerVel", backspinRoller.getVelocity());
    }
    public void launchArtifact(){
        if(!artifactRunning) return;
        if(targetVel == 900){
            switch (artifactState) {

                case 0:
                    fanRotate.setPosition(upPos3);
                    artifactTimer.reset();
                    artifactState++;
                    break;

                case 1:
                    if (artifactTimer.milliseconds() > 700) {
                        camUp();
                        artifactTimer.reset();
                        artifactState++;
                    }
                    break;

                case 2:
                    if (artifactTimer.milliseconds() > 700) {
                        fanRotate.setPosition(upPos2);
                        artifactTimer.reset();
                        artifactState++;
                    }
                    break;

                case 3:
                    if (artifactTimer.milliseconds() > 700) {
                        camUp();
                        artifactTimer.reset();
                        artifactState++;
                    }
                    break;

                case 4:
                    if (artifactTimer.milliseconds() > 700) {
                        fanRotate.setPosition(upPos1);
                        artifactTimer.reset();
                        artifactState++;
                    }
                    break;

                case 5:
                    if (artifactTimer.milliseconds() > 700) {
                        camUp();
                        artifactRunning = false; // DONE
                    }
                    break;
            }
        }
        else{
            switch (artifactState) {

                case 0:
                    fanRotate.setPosition(upPos3);
                    artifactTimer.reset();
                    artifactState++;
                    break;

                case 1:
                    if (artifactTimer.milliseconds() > 700) {
                        camUp();
                        artifactTimer.reset();
                        artifactState++;
                    }
                    break;

                case 2:
                    if (artifactTimer.milliseconds() > 450) {
                        fanRotate.setPosition(upPos2);
                        artifactTimer.reset();
                        artifactState++;
                    }
                    break;

                case 3:
                    if (artifactTimer.milliseconds() > 450) {
                        camUp();
                        artifactTimer.reset();
                        artifactState++;
                    }
                    break;

                case 4:
                    if (artifactTimer.milliseconds() > 450) {
                        fanRotate.setPosition(upPos1);
                        artifactTimer.reset();
                        artifactState++;
                    }
                    break;

                case 5:
                    if (artifactTimer.milliseconds() > 450) {
                        camUp();
                        artifactRunning = false; // DONE
                    }
                    break;
            }
        }

    }
    public void startArtifact(){
        artifactRunning = true;
        artifactState = 0;
        artifactTimer.reset();
    }
    public void camUp(){
        if (camPos == 1) {
            camPos = 0;
        } else if (camPos == 0) {
            camPos = 1;
        }
        cam.setPosition(camPos);
    }
}