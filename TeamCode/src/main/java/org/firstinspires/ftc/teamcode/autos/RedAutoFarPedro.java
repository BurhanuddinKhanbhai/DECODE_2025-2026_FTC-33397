package org.firstinspires.ftc.teamcode.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.*;
import com.pedropathing.paths.*;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.HoodSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeFeederSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;

@Autonomous(name = "RedAutoFar (FixedRPM)", group = "Pedro Pathing")
public class RedAutoFarPedro extends LinearOpMode {

    // =========================
    // FAR RED START / SHOOT
    // =========================
    // You said -19, 47 is the far spot. We start already rotated "right 30°".
    // If your "right 30°" is the opposite direction, flip 150 to 210.
    private static final Pose START = new Pose(-19, 47, Math.toRadians(0));
    private static final Pose SHOOT_POSE = new Pose(-10, 47, Math.toRadians(-33));

    // =========================
    // INTAKE / LINEUP POSES (reusing your close coords)
    // =========================
    private static final Pose SECOND_SHOT_LINEUP = new Pose(40, -32, Math.toRadians(-90));
    private static final Pose THIRD_SHOT_LINEUP  = new Pose(0, -32, Math.toRadians(-90));

    private static final Pose INTAKE_ROW2_POSE   = new Pose(60, -70, Math.toRadians(-90));
    private static final Pose INTAKE_ROW3_POSE   = new Pose(-10, -70, Math.toRadians(-90));

    // =========================
    // Subsystems
    // =========================
    private Follower follower;
    private ShooterSubsystem shooter;
    private HoodSubsystem hood;
    private IntakeFeederSubsystem intakeFeeder;

    // =========================
    // Tuning
    // =========================
    private static final double RPM_TOL = 75;
    private static final long SPINUP_TIMEOUT_MS = 1800;
    private static final long FEED_MS = 850;
    private static final double INTAKE_FEED_PWR = 1.0;

    // FIXED FAR SHOT SETTINGS (no limelight)
    private static final double FIXED_TARGET_RPM = 3500; // <-- change this to your tested far RPM
    private static final double FIXED_HOOD_POS   = .56;

    @Override
    public void runOpMode() {

        // ---------- Hardware ----------
        DcMotorEx leftFly  = hardwareMap.get(DcMotorEx.class, "flywheel2");
        DcMotorEx rightFly = hardwareMap.get(DcMotorEx.class, "flywheel");

        DcMotorEx intake = hardwareMap.get(DcMotorEx.class, "intakeMotor");
        DcMotorEx feeder = hardwareMap.get(DcMotorEx.class, "intakeMotor2");

        Servo hoodServo = hardwareMap.get(Servo.class, "hood");

        shooter = new ShooterSubsystem(leftFly, rightFly);
        hood = new HoodSubsystem(hoodServo);
        intakeFeeder = new IntakeFeederSubsystem(intake, feeder);

        // ---------- Pedro ----------
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(START);

        // =========================
        // Build Paths
        // =========================

        // ROW 3 FIRST
        PathChain firstShot     = line(START,SHOOT_POSE);
        PathChain toThirdLineup = line(SHOOT_POSE, THIRD_SHOT_LINEUP);
        PathChain lineupToRow3  = line(THIRD_SHOT_LINEUP, INTAKE_ROW3_POSE);
        PathChain backToShoot3  = line(INTAKE_ROW3_POSE, SHOOT_POSE);

        // ROW 2 SECOND
        PathChain toSecondLineup = line(SHOOT_POSE, SECOND_SHOT_LINEUP);
        PathChain lineupToRow2   = line(SECOND_SHOT_LINEUP, INTAKE_ROW2_POSE);
        PathChain backToShoot2   = line(INTAKE_ROW2_POSE, SHOOT_POSE);

        telemetry.addLine("RedAutoFar Ready (FixedRPM)");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        hood.setTargetPos(FIXED_HOOD_POS);   // ← ADD THIS

        // =========================
        // AUTO SEQUENCE
        // =========================

        // Shoot preload (already at shoot pose)

        followAndWait(firstShot);
        shootBurst3Fixed();

        // Row 3 first
        followAndWait(toThirdLineup);
        setIntake(true);
        followAndWait(lineupToRow3);
        setIntake(false);

        followAndWait(backToShoot3);
        shootBurst3Fixed();

        // Row 2 second
        followAndWait(toSecondLineup);
        setIntake(true);
        followAndWait(lineupToRow2);
        setIntake(false);

        followAndWait(backToShoot2);
        shootBurst3Fixed();

        safeStopAll();
    }

    // =========================
    // Path Helper
    // =========================
    private PathChain line(Pose a, Pose b) {
        return follower.pathBuilder()
                .addPath(new BezierLine(a, b))
                .setLinearHeadingInterpolation(a.getHeading(), b.getHeading())
                .build();
    }

    private void followAndWait(PathChain path) {
        follower.followPath(path);
        while (opModeIsActive() && follower.isBusy()) {
            follower.update();
            shooter.update();
            hood.update();
            intakeFeeder.update();
            idle();
        }
    }

    // =========================
    // Fixed shooting (no Limelight)
    // =========================
    private void shootBurst3Fixed() {

        shooter.setEnabled(true);

        shooter.setTargetRPM(FIXED_TARGET_RPM);
        hood.setTargetPos(FIXED_HOOD_POS);

        ElapsedTime spin = new ElapsedTime();
        spin.reset();

        while (opModeIsActive() && spin.milliseconds() < SPINUP_TIMEOUT_MS) {
            shooter.update();
            hood.update();

            boolean ready = shooter.atSpeed(RPM_TOL);

            telemetry.addLine("=== FIXED SHOOTER ===");
            telemetry.addData("Target RPM", "%.0f", FIXED_TARGET_RPM);
            telemetry.addData("Current RPM", "%.0f", shooter.getRPM());
            telemetry.addData("Hood", "%.2f", FIXED_HOOD_POS);
            telemetry.addData("At Speed?", ready);
            telemetry.update();

            if (ready) break;
            idle();
        }

        ElapsedTime feed = new ElapsedTime();
        feed.reset();

        while (opModeIsActive() && feed.milliseconds() < FEED_MS) {
            intakeFeeder.setIntakePower(INTAKE_FEED_PWR);
            intakeFeeder.setFeederPower(INTAKE_FEED_PWR);

            shooter.update();
            intakeFeeder.update();

            telemetry.addLine("=== FEEDING ===");
            telemetry.addData("RPM", "%.0f", shooter.getRPM());
            telemetry.update();
            idle();
        }

        intakeFeeder.setIntakePower(0);
        intakeFeeder.setFeederPower(0);
        intakeFeeder.update();

        sleep(100);
    }

    private void setIntake(boolean on) {
        intakeFeeder.setIntakePower(on ? 1.0 : 0.0);
        intakeFeeder.setFeederPower(0.0);
        intakeFeeder.update();
    }

    private void safeStopAll() {
        shooter.setEnabled(false);
        intakeFeeder.setIntakePower(0);
        intakeFeeder.setFeederPower(0);
        intakeFeeder.update();
    }
}