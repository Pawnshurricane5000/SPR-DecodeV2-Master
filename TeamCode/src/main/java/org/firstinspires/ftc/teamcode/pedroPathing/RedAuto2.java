package org.firstinspires.ftc.teamcode.pedroPathing; // make sure this aligns with class location

import static java.lang.Thread.sleep;

import android.graphics.Color;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.Iterator;
import java.util.LinkedList;
import java.util.Queue;
import java.util.function.Supplier;

@Autonomous(name = "Red Auto Near", group = "Examples")
public class RedAuto2 extends OpMode {
    private enum VisionState {
        MOVE_TURRET_TO_OBELISK,
        DETECT_OBELISK_TAG,
        SWITCH_TO_BLUE,
        TRACK_BLUE_TAG
    }
    private boolean shooting = false;
    private int shootStep = 0;
    private ElapsedTime shootTimer = new ElapsedTime();

    boolean tagSeen = false;
    private Queue<Servo> activeArms = new LinkedList<>();

    private final long ARM_UP_TIME = 200;   // ms to hold up
    private final long ARM_DOWN_TIME = 250; // ms to fully retract

    // Map each servo to its last action timestamp
    private final java.util.Map<Servo, Long> armTimers = new java.util.HashMap<>();
    private VisionState visionState = VisionState.MOVE_TURRET_TO_OBELISK;
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private HuskyLens huskyLens;
    boolean tagLocked = false;
    int lockedTagId = -1;

    private Servo fanRotate, cam, shooterAngle;
    private DcMotorEx outtake1, outtake2, intake;
    private int targetVel = 1350;

    private double maxVelocityOuttake = 2400;
    private double F = 32767.0 / maxVelocityOuttake;
    private double kP = F * 5;
    private double kI = 0;
    private double kD = 0;
    private boolean x = true;

    private Limelight3A limelight;
    private ColorSensor c1, c2, c3;


    ElapsedTime artifactTimer = new ElapsedTime();

    private Servo  arm1, arm2, arm3;
    private boolean shootWindow = false;  // auto says "you may start shooting now"

    // Shooting sequence variables
    private Servo[] shootArms;
    private String[] shootColors = {"UNKNOWN","UNKNOWN","UNKNOWN"}; // detected colors
    private String[] shootPattern = {"GREEN","PURPLE","PURPLE"};    // default pattern

    private DcMotorEx turret;
    private DcMotorSimple rightFront, leftFront, rightRear, leftRear;

    private ElapsedTime searchTimer = new ElapsedTime();
    private boolean[] pathTriggered = new boolean[100]; // support up to 10 pathStates

    private int motorVel = 0;
    private double fastModeMultiplier = .3;
    private int pathState;
    private boolean hasShot = false;

    private final Pose startPose = new Pose(107, 131, Math.toRadians(270)); // Start Pose of our robot.
    private final Pose launchPose = new Pose(81,105, Math.toRadians(270));
    private final Pose order3 = new Pose(89, 79, Math.toRadians(0)); // Middle (Second Set) of Artifacts from the Spike Mark.
    private final Pose order32 = new Pose(123, 79, Math.toRadians(0)); // Middle (Second Set) of Artifacts from the Spike Mark.
    private final Pose order2 = new Pose(89, 55.5, Math.toRadians(0)); // Middle (Second Set) of Artifacts from the Spike Mark.
    private final Pose order2s = new Pose(40,78.5,Math.toRadians(0));
    private final Pose order21 = new Pose(30, 78.5, Math.toRadians(0)); // Middle (Second Set) of Artifacts from the Spike Mark.
    private final Pose order22 = new Pose(128.5, 55.5, Math.toRadians(0));
    private final Pose park = new Pose(123, 100, Math.toRadians(180));
    private final Pose order1s = new Pose(40,54.5,Math.toRadians(0));
    private final Pose order11 = new Pose(36, 54.5, Math.toRadians(0)); // Middle (Second Set) of Artifacts from the Spike Mark.
    private final Pose order12 = new Pose(28, 54.5, Math.toRadians(0)); // Middle (Second Set) of Artifacts from the Spike Mark.
    // Middle (Second Set) of Artifacts from the Spike Mark.
    private Path detect;
    private PathChain launch, launch3, moveToLaunch, moveToPark, moveToLaunch2, moveToOrder3,moveToOrder31,moveToOrder32,moveToLaunch1, moveToOrder2,moveToOrder21,moveToOrder22,moveToOrder2s, parkP, moveToOrder11,moveToOrder12,moveToOrder1s;
    /** This is the main loop of the OpMode, it will run repeatedly after clicking "Play". **/
    public void buildPaths(){
        moveToLaunch = follower.pathBuilder()
                .addPath(new BezierLine(startPose,launchPose))
                .setConstantHeadingInterpolation(startPose.getHeading())
                .build();
        moveToOrder3 = follower.pathBuilder()
                .addPath(new BezierLine(launchPose,order3))
                .setLinearHeadingInterpolation(launchPose.getHeading(), order3.getHeading())
                .build();
        moveToOrder32 = follower.pathBuilder()
                .addPath(new BezierLine(order3,order32))
                .setConstantHeadingInterpolation(order3.getHeading())
                .build();
        moveToLaunch1 = follower.pathBuilder()
                .addPath(new BezierLine(order32,launchPose))
                .setLinearHeadingInterpolation(order32.getHeading(), launchPose.getHeading())
                .build();
        moveToOrder2 = follower.pathBuilder()
                .addPath(new BezierLine(launchPose,order2))
                .setLinearHeadingInterpolation(launchPose.getHeading(), order2.getHeading())
                .build();
        moveToOrder22 = follower.pathBuilder()
                .addPath(new BezierLine(order2,order22))
                .setConstantHeadingInterpolation(order2.getHeading())
                .build();
        moveToLaunch2 = follower.pathBuilder()
                .addPath(new BezierLine(order22,launchPose))
                .setLinearHeadingInterpolation(order22.getHeading(), launchPose.getHeading())
                .build();
        moveToPark = follower.pathBuilder()
                .addPath(new BezierLine(launchPose,park))
                .setConstantHeadingInterpolation(launchPose.getHeading())
                .build();
    }
    public void autonomousPathUpdate() {

        switch (pathState) {

            // ======================================
            // GO TO LAUNCH
            // ======================================
            case 0:
                follower.followPath(moveToLaunch);
                setPathState(1);
                break;

            case 1:
                if (!follower.isBusy()) {
                    moveTurretTo(600);
                    outtake1.setVelocity(1140);
                    outtake2.setVelocity(1140);
                    setPathState(2);
                }
                break;

            // ======================================
            // FIRST SHOT
            // ======================================
            case 2:
                if (Math.abs(turret.getCurrentPosition() - 600) < 15
                        && outtake1.getVelocity() >= 1130) {
                    startShooting();
                    setPathState(3);
                }
                break;

            case 3:
                if (!shooting) {
                    intake.setPower(1);
                    follower.followPath(moveToOrder3);
                    moveTurretTo(600);
                    setPathState(4);
                }
                break;

            case 4:
                if (!follower.isBusy()) {
                    follower.followPath(moveToOrder32);
                    setPathState(5);
                }
                break;

            case 5:
                if (!follower.isBusy()) {
                    outtake1.setVelocity(1140);
                    outtake2.setVelocity(1140);
                    moveTurretTo(600);
                    follower.followPath(moveToLaunch1);
                    setPathState(6);
                }
                break;

            case 6:
                if (!follower.isBusy()
                        && outtake1.getVelocity() >= 1130) {
                    startShooting();
                    setPathState(7);
                }
                break;

            case 7:
                if (!shooting) {
                    intake.setPower(1);
                    moveTurretTo(600);
                    follower.followPath(moveToOrder2);
                    setPathState(8);
                }
                break;

            case 8:
                if (!follower.isBusy()) {
                    follower.followPath(moveToOrder22);
                    setPathState(9);
                }
                break;

            case 9:
                if (!follower.isBusy()) {
                    outtake1.setVelocity(1140);
                    outtake2.setVelocity(1140);
                    moveTurretTo(600);
                    follower.followPath(moveToLaunch2);
                    setPathState(10);
                }
                break;

            case 10:
                if (!follower.isBusy()
                        && outtake1.getVelocity() >= 1130) {
                    startShooting();
                    setPathState(11);
                }
                break;

            case 11:
                if (!shooting) {
                    setPathState(12);
                }
                break;
            case 12:
                if (!follower.isBusy()) {
                    follower.followPath(moveToPark);
                    setPathState(-1);
                }
                break;
            case -1:
                intake.setPower(0);
                outtake1.setVelocity(0);
                moveTurretTo(0);
                outtake2.setVelocity(0);
                follower.breakFollowing();
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
        if (pathState == 1) {
            prepareTurretAndDetect();
        }
        follower.update();

        // Move paths and shooting
        autonomousPathUpdate(); // pathState updates

        updateShooting();

        // continuously call shoot() to progress the sequence
        // Shoot whenever turret is tracking blue tag
        telemetry.addData("pathState", pathState);
        telemetry.addData("shooting", shooting);
        telemetry.addData("shootStep", shootStep);
        telemetry.addData("tagSeen", tagSeen);
        telemetry.addData("turretPos", turret.getCurrentPosition());
        telemetry.update();

    }

    /** This method is called once at the init of the OpMode. **/
    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();
        arm1 = hardwareMap.get(Servo.class, "arm1");
        arm2 = hardwareMap.get(Servo.class, "arm2");
        arm3 = hardwareMap.get(Servo.class, "arm3");
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        outtake1 = hardwareMap.get(DcMotorEx.class, "outtake1");
        outtake2 = hardwareMap.get(DcMotorEx.class, "outtake2");
        turret = hardwareMap.get(DcMotorEx.class, "turret");
        shooterAngle = hardwareMap.get(Servo.class, "shooterAngle");
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        c1 = hardwareMap.get(ColorSensor.class, "c1");
        c2 = hardwareMap.get(ColorSensor.class, "c2");
        c3 = hardwareMap.get(ColorSensor.class, "c3");
        leftFront = hardwareMap.get(DcMotorSimple.class, "leftFront");
        leftRear = hardwareMap.get(DcMotorSimple.class, "leftRear");
        rightRear = hardwareMap.get(DcMotorSimple.class, "rightRear");
        rightFront = hardwareMap.get(DcMotorSimple.class, "rightFront");
        shootArms = new Servo[]{arm1, arm2, arm3};
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        buildPaths();
        outtake1.setVelocityPIDFCoefficients(kP, kI, kD, F);
        outtake2.setVelocityPIDFCoefficients(kP, kI, kD, F);
        outtake2.setDirection(DcMotorSimple.Direction.REVERSE);
        outtake1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        outtake2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turret.setDirection(DcMotorSimple.Direction.REVERSE); // or REVERSE if needed
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turret.setTargetPosition(0);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooterAngle.setPosition(.9);
        turretAtStart = false;
        tagDetected = false;
        switchedToBluePipeline = false;
        visionState = VisionState.MOVE_TURRET_TO_OBELISK;
        arm1.setPosition(0);
        arm2.setPosition(0);
        arm3.setPosition(0);

    }

    /** This method is called continuously after Init while waiting for "play". **/
    @Override
    public void init_loop() {
    }

    /** This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system **/
    @Override
    public void start() {
        limelight.pipelineSwitch(6);
        limelight.start();                  // start camera
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        turretAtStart = false;
        tagDetected = false;
        switchedToBluePipeline = false;
        visionState = VisionState.MOVE_TURRET_TO_OBELISK;
        opmodeTimer.resetTimer();
        setPathState(0);
    }
    private static final double CENTER_ENTER = 2.0; // deg: lock when inside
    private static final double CENTER_EXIT  = 3.0; // deg: unlock when outside
    private static final double TRACK_KP     = 0.01;
    private static final double MAX_POWER    = .4;
    private boolean turretCentered = false;
    private static final double CENTER_OFFSET_DEG = 0; // aim left of tag

    // Class variables
    private boolean tagDetected = false;
    private boolean switchedToBluePipeline = false;
    private int detectedTagId = -1;
    private boolean turretAtStart = false; // <-- new flag

    private static final int TURRET_OBELISK_POS = 800;  // ticks
    private static final double TURRET_MOVE_POWER = 0.4;
    private static final double TURRET_POS_TOL = 10;    // ticks
    private static final double DETECT_TIMEOUT = 2.0;   // seconds to try seeing obelisk

    private ElapsedTime detectTimer = new ElapsedTime();

    private void prepareTurretAndDetect() {
        switch (visionState) {

            case MOVE_TURRET_TO_OBELISK:
                // Point turret toward obelisk and wait until it's there
                turret.setPower(0);
                turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                turretAtStart = true;

                // Start obelisk detection
                limelight.pipelineSwitch(6); // 3-tag obelisk pipeline
                detectTimer.reset();
                visionState = VisionState.DETECT_OBELISK_TAG;
                break;

            case DETECT_OBELISK_TAG:
                LLResult result = limelight.getLatestResult();

                if (result != null && result.isValid()
                        && result.getFiducialResults() != null
                        && !result.getFiducialResults().isEmpty()) {

                    LLResultTypes.FiducialResult tag = result.getFiducialResults().get(0);
                    detectedTagId = tag.getFiducialId();
                    tagDetected = true;

                    telemetry.addData("Obelisk Tag Detected", detectedTagId);

                    turret.setTargetPosition(350);
                    turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    turret.setPower(0.4);

                }
                break;
        }
    }

    private void runTurretTracking() {
        LLResult result = limelight.getLatestResult();
        tagSeen = false;
        double tx = 0;

        if (result != null && result.isValid()
                && result.getFiducialResults() != null
                && !result.getFiducialResults().isEmpty()) {
            tx = result.getFiducialResults().get(0).getTargetXDegrees();
            tagSeen = true;
        }

        double power = 0;
        int pos = turret.getCurrentPosition();
        boolean atLeftLimit = pos <= -995;
        boolean atRightLimit = pos >= 995;

        if (tagSeen) {
            double errorDeg = -tx; // adjust for offset if needed
            power = TRACK_KP * errorDeg;
            power = Math.max(-MAX_POWER, Math.min(MAX_POWER, power));

            if ((atLeftLimit && power < 0) || (atRightLimit && power > 0)) {
                power = 0;
            }
        } else {
            // simple search if tag lost
            power = 0;
        }

        turret.setPower(power);

        telemetry.addData("TX", tx);
        telemetry.addData("Turret Power", power);
        telemetry.update();
    }



    sprTeleopBlue.ArmState armState = sprTeleopBlue.ArmState.IDLE;
    long armTimer = 0;
    Servo activeArm = null;


    public void startShooting() {
        shooting = true;
        shootStep = 0;
        shootTimer.reset();
    }
    public void updateShooting() {

        if (!shooting) return;

        if (shootStep == 0) {
            arm1.setPosition(1);
            shootTimer.reset();
            shootStep = 1;
        }

        else if (shootStep == 1 && shootTimer.milliseconds() >= 400) {
            arm1.setPosition(0);
            shootTimer.reset();
            shootStep = 2;
        }

        else if (shootStep == 2 && shootTimer.milliseconds() >= 200) {
            arm2.setPosition(1);
            shootTimer.reset();
            shootStep = 3;
        }

        else if (shootStep == 3 && shootTimer.milliseconds() >= 400) {
            arm2.setPosition(0);
            shootTimer.reset();
            shootStep = 4;
        }

        else if (shootStep == 4 && shootTimer.milliseconds() >= 200) {
            arm3.setPosition(1);
            shootTimer.reset();
            shootStep = 5;
        }

        else if (shootStep == 5 && shootTimer.milliseconds() >= 400) {
            arm3.setPosition(0);
            shooting = false;
            shootStep = 0;
        }
    }

    private String detectColor1(ColorSensor c) {
        // Check proximity first (distance to object)
        if (c instanceof DistanceSensor) {
            double distance = ((DistanceSensor) c).getDistance(DistanceUnit.MM);
            if (distance > 60) return "NONE"; // No ball in front
        }

        // Convert RGB to HSV
        float[] hsv = new float[3];
        Color.RGBToHSV(c.red(), c.green(), c.blue(), hsv);
        float hue = hsv[0];       // 0-360 degrees
        float sat = hsv[1];       // 0-1
        float val = hsv[2];       // 0-1

        // Green range (tweak if needed)
        if (hue > 150 && hue < 170 && sat > 0.5) return "GREEN";

        // Purple range (tweak if needed)
        if (hue > 180 && sat < .45) return "PURPLE";

        return "UNKNOWN";
    }

    private String detectColor2(ColorSensor c) {
        // Check proximity first (distance to object)
        if (c instanceof DistanceSensor) {
            double distance = ((DistanceSensor) c).getDistance(DistanceUnit.MM);
            if (distance > 70) return "NONE"; // No ball in front
        }

        // Convert RGB to HSV
        float[] hsv = new float[3];
        Color.RGBToHSV(c.red(), c.green(), c.blue(), hsv);
        float hue = hsv[0];       // 0-360 degrees
        float sat = hsv[1];       // 0-1
        float val = hsv[2];       // 0-1

        // Green range (tweak if needed)
        if (hue > 160 && hue < 170 && sat > .6) return "GREEN";

        // Purple range (tweak if needed)
        if (hue > 180 && sat < .53) return "PURPLE";

        return "UNKNOWN";
    }

    private String detectColor3(ColorSensor c) {
        // Check proximity first (distance to object)
        if (c instanceof DistanceSensor) {
            double distance = ((DistanceSensor) c).getDistance(DistanceUnit.MM);
            if (distance > 60) return "NONE"; // No ball in front
        }

        // Convert RGB to HSV
        float[] hsv = new float[3];
        Color.RGBToHSV(c.red(), c.green(), c.blue(), hsv);
        float hue = hsv[0];       // 0-360 degrees
        float sat = hsv[1];       // 0-1
        float val = hsv[2];       // 0-1

        // Green range (tweak if needed)
        if (hue > 150 && hue < 160 && sat > 0.6) return "GREEN";

        // Purple range (tweak if needed)
        if (hue > 200 && sat < .5) return "PURPLE";

        return "UNKNOWN";
    }

    /** We do not use this because everything should automatically disable **/
    @Override
    public void stop() {}

    public Pose getFinalPose(){
        return order2;
    }
    private void moveTurretTo(int target) {
        turret.setTargetPosition(target);
        turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turret.setPower(0.4);
    }
}