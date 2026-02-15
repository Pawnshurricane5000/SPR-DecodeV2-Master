package org.firstinspires.ftc.teamcode.pedroPathing; // make sure this aligns with class location

import android.graphics.Color;

import com.pedropathing.follower.Follower;
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

import java.util.LinkedList;
import java.util.Queue;

@Autonomous(name = "Blue Auto Far Shoot 3", group = "Examples")
public class BlueAuto1Shoot3 extends OpMode {
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

    private final Pose startPose = new Pose(60, 6, Math.toRadians(270)); // Start Pose of our robot.
    private final Pose order3 = new Pose(45, 30.5, Math.toRadians(180)); // Middle (Second Set) of Artifacts from the Spike Mark.
    private final Pose order32 = new Pose(11.5, 30.5, Math.toRadians(180)); // Middle (Second Set) of Artifacts from the Spike Mark.
    private final Pose order2 = new Pose(45, 54.5, Math.toRadians(180)); // Middle (Second Set) of Artifacts from the Spike Mark.
    private final Pose order2s = new Pose(40,78.5,Math.toRadians(180));
    private final Pose order21 = new Pose(30, 78.5, Math.toRadians(180)); // Middle (Second Set) of Artifacts from the Spike Mark.
    private final Pose order22 = new Pose(11.5, 54.5, Math.toRadians(180));
    private final Pose park = new Pose(40, 6, Math.toRadians(180));
    private final Pose order1s = new Pose(40,54.5,Math.toRadians(180));
    private final Pose order11 = new Pose(36, 54.5, Math.toRadians(180)); // Middle (Second Set) of Artifacts from the Spike Mark.
    private final Pose order12 = new Pose(28, 54.5, Math.toRadians(180)); // Middle (Second Set) of Artifacts from the Spike Mark.
    // Middle (Second Set) of Artifacts from the Spike Mark.
    private Path detect;
    private PathChain launch, launch3, moveToPark, moveToLaunch2, moveToOrder3,moveToOrder31,moveToOrder32,moveToLaunch1, moveToOrder2,moveToOrder21,moveToOrder22,moveToOrder2s, parkP, moveToOrder11,moveToOrder12,moveToOrder1s;
    /** This is the main loop of the OpMode, it will run repeatedly after clicking "Play". **/
    public void buildPaths(){
       moveToOrder3 = follower.pathBuilder()
                .addPath(new BezierLine(startPose,order3))
                .setLinearHeadingInterpolation(startPose.getHeading(), order3.getHeading())
                .build();
        moveToOrder32 = follower.pathBuilder()
                .addPath(new BezierLine(order3,order32))
                .setConstantHeadingInterpolation(order3.getHeading())
                .build();
        moveToLaunch1 = follower.pathBuilder()
                .addPath(new BezierLine(order32,startPose))
                .setLinearHeadingInterpolation(order32.getHeading(), startPose.getHeading())
                .build();
        moveToOrder2 = follower.pathBuilder()
                .addPath(new BezierLine(startPose,order2))
                .setLinearHeadingInterpolation(startPose.getHeading(), order2.getHeading())
                .build();
        moveToOrder22 = follower.pathBuilder()
                .addPath(new BezierLine(order2,order22))
                .setConstantHeadingInterpolation(order2.getHeading())
                .build();
        moveToLaunch2 = follower.pathBuilder()
                .addPath(new BezierLine(order22,startPose))
                .setLinearHeadingInterpolation(order22.getHeading(), startPose.getHeading())
                .build();
        moveToPark = follower.pathBuilder()
                .addPath(new BezierLine(startPose,park))
                .setConstantHeadingInterpolation(startPose.getHeading())
                .build();
    }
    public void autonomousPathUpdate() throws InterruptedException {

        switch (pathState) {            case 0: // Start first shot
            // Spin up flywheels
            outtake1.setVelocity(1300);
            outtake2.setVelocity(1300);

            // Start shooting once at speed
            if (!hasShot && outtake1.getVelocity() >= 1270) {
                startShooting();
                hasShot = true;
            }

            // Move on once shooting is finished
            if (hasShot && !shooting) {
                setPathState(6);
            }
            break;

            case 1: // Drive to the first collection line
                if (!pathTriggered[1]) {
                    intake.setPower(1);
                    follower.followPath(moveToOrder3);
                    pathTriggered[1] = true;
                }

                if (!follower.isBusy() && pathTriggered[1]) {
                    pathTriggered[1] = false;
                    setPathState(10); // Transition to the next state for the sideways move
                }
                break;

            case 10: // Move sideways to collect first set of artifacts
                if (!pathTriggered[10]) {
                    moveTurretTo(210);
                    follower.followPath(moveToOrder32);
                    pathTriggered[10] = true;
                }

                if (!follower.isBusy() && pathTriggered[10]) {
                    pathTriggered[10] = false;
                    hasShot = false; // Reset for the next shot
                    setPathState(2); // Now go back to launch
                }
                break;

            case 2: // Return to launch for the second shot
                if (!pathTriggered[2]) {
                    follower.followPath(moveToLaunch1);
                    pathTriggered[2] = true;
                }

                // When path is done, start the shooting sequence
                if (!follower.isBusy() && pathTriggered[2]) {
                    pathTriggered[2] = false;
                    setPathState(20); // Transition to a dedicated shooting state
                }
                break;

            case 20: // Perform the second shot
                outtake1.setVelocity(1300);
                outtake2.setVelocity(1300);

                if (!hasShot && outtake1.getVelocity() >= 1270) {
                    startShooting();
                    hasShot = true;
                }

                if (hasShot && !shooting) {
                    setPathState(6); // Move on to the next collection cycle
                }
                break;

            case 3: // Drive to the second collection line
                if (!pathTriggered[3]) {
                    hasShot = false; // Reset for the final shot cycle
                    intake.setPower(1);
                    follower.followPath(moveToOrder2);
                    pathTriggered[3] = true;
                }

                if (!follower.isBusy() && pathTriggered[3]) {
                    pathTriggered[3] = false;
                    setPathState(4); // Transition to the sideways move
                }
                break;

            case 4: // Move sideways to collect the second set of artifacts
                if (!pathTriggered[4]) {
                    follower.followPath(moveToOrder22);
                    pathTriggered[4] = true;
                }

                if (!follower.isBusy() && pathTriggered[4]) {
                    moveTurretTo(210);
                    pathTriggered[4] = false;
                    setPathState(40); // Transition to the return path
                }
                break;

            case 40: // Return to launch for the final shot
                if (!pathTriggered[40]) {
                    follower.followPath(moveToLaunch2);
                    pathTriggered[40] = true;
                }

                if (!follower.isBusy() && pathTriggered[40]) {
                    pathTriggered[40] = false;
                    setPathState(5); // Transition to the final shooting state
                }
                break;

            case 5: // Perform the final shot
                outtake1.setVelocity(1300);
                outtake2.setVelocity(1300);

                // Start shooting once at speed
                if (!hasShot && outtake1.getVelocity() >= 1270) {
                    startShooting();
                    hasShot = true;
                }

                // Check if the shooting is done
                if (hasShot && !shooting) {
                    setPathState(6); // End of autonomous routine
                }
                break;
            case 6: // Perform the final shot
                if (!follower.isBusy()) {
                    follower.followPath(moveToPark);
                    setPathState(-1); // Transition to the return path
                }
                break;
            case -1:
                // Finished
                outtake1.setVelocity(0);
                outtake2.setVelocity(0);
                moveTurretTo(0);
                intake.setPower(0);
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
        if (pathState == 0) {
            prepareTurretAndDetect();
        }
        follower.update();

        // Move paths and shooting
        try {
            autonomousPathUpdate(); // pathState updates
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
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
        turret.setDirection(DcMotorSimple.Direction.FORWARD); // or REVERSE if needed
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turret.setTargetPosition(0);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooterAngle.setPosition(.7);
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

                    turret.setTargetPosition(230);
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

        else if (shootStep == 1 && shootTimer.milliseconds() >= 300) {
            arm1.setPosition(0);
            shootTimer.reset();
            shootStep = 2;
        }

        else if (shootStep == 2 && shootTimer.milliseconds() >= 400) {
            arm2.setPosition(1);
            shootTimer.reset();
            shootStep = 3;
        }

        else if (shootStep == 3 && shootTimer.milliseconds() >= 300) {
            arm2.setPosition(0);
            shootTimer.reset();
            shootStep = 4;
        }

        else if (shootStep == 4 && shootTimer.milliseconds() >= 400) {
            arm3.setPosition(1);
            shootTimer.reset();
            shootStep = 5;
        }

        else if (shootStep == 5 && shootTimer.milliseconds() >= 300) {
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