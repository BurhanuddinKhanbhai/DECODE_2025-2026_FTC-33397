package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.subsystems.*;
import org.firstinspires.ftc.teamcode.util.MathUtil;
import org.firstinspires.ftc.teamcode.util.ShooterModel;

@TeleOp(name="TeleOp Main (Crash-Proof, No Angle Debug)", group="TeleOp")
public class TeleOp_MainSubsystems extends OpMode {

    private DriveSubsystem drive;
    private ShooterSubsystem shooter;
    private HoodSubsystem hood;
    private IntakeFeederSubsystem intakeFeeder;
    private LogitechCameraSubsystem logitech;

    // Driver controls
    private boolean autoAimEnabled = true;
    private double manualTargetRPM = 4000;

    // Shooter gating (aim gate removed)
    private static final double RPM_TOL = 75;

    // Fixed-shot override (HOLD)
    private static final double FIXED_SHOT_RPM = 3900;
    private static final double FIXED_HOOD_POS = 0.75;

    // Toggle edge detection (gamepad2)
    private boolean lastStart2 = false;
    private boolean lastBack2 = false;

    // Repeat-rate limiter for dpad tuning
    private long lastAdjustMs = 0;
    private static final long ADJUST_PERIOD_MS = 140;

    // Logitech camera safety / fallback
    private long cameraFaultUntilMs = 0;
    private static final long CAMERA_FAULT_COOLDOWN_MS = 600;
    private double lastGoodDistanceIn = 70;           // fallback distance for autoaim
    private long lastGoodDistanceMs = 0;
    private static final long GOOD_DIST_STALE_MS = 1000;

    @Override
    public void init() {
        // Drive motors
        DcMotorEx fl = hardwareMap.get(DcMotorEx.class, "frontLeftDrive");
        DcMotorEx fr = hardwareMap.get(DcMotorEx.class, "frontRightDrive");
        DcMotorEx bl = hardwareMap.get(DcMotorEx.class, "backLeftDrive");
        DcMotorEx br = hardwareMap.get(DcMotorEx.class, "backRightDrive");

        // Shooter motors
        DcMotorEx leftFly = hardwareMap.get(DcMotorEx.class, "flywheel2");
        DcMotorEx rightFly = hardwareMap.get(DcMotorEx.class, "flywheel");

        // Intake/feeder motors
        DcMotorEx intake = hardwareMap.get(DcMotorEx.class, "intakeMotor");
        DcMotorEx feeder = hardwareMap.get(DcMotorEx.class, "intakeMotor2");

        // Hood servo
        Servo hoodServo = hardwareMap.get(Servo.class, "hood");

        // Logitech webcam
        WebcamName logitechCamera = hardwareMap.get(WebcamName.class, "logitech");

        drive = new DriveSubsystem(fl, fr, bl, br);
        shooter = new ShooterSubsystem(leftFly, rightFly);
        hood = new HoodSubsystem(hoodServo);
        intakeFeeder = new IntakeFeederSubsystem(intake, feeder);
        logitech = new LogitechCameraSubsystem(logitechCamera);
        logitech.init();

        telemetry.addLine("TeleOp ready (Logitech camera, Crash-Proof)");
        telemetry.addLine("GP1: drive, A=shoot hold (autoaim), B=FIXED SHOT hold, X=feeder reverse, triggers=intake");
        telemetry.addLine("GP1: Y=override run both forward");
        telemetry.addLine("GP2: RB enable shooter, LB disable shooter, Start toggle AutoAim, Back toggle PIDF/FF");
        telemetry.update();
    }

    @Override
    public void loop() {
        // =======================
        // Safe camera update (never crash)
        // =======================
        safeCameraUpdate();

        // Read camera safely (even if blocked)
        CameraRead cameraRead = safeCameraRead();

        // =======================
        // Drive (gamepad1)
        // =======================
        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;

        y = (Math.abs(y) < 0.05) ? 0 : y;
        x = (Math.abs(x) < 0.05) ? 0 : x;
        rx = (Math.abs(rx) < 0.05) ? 0 : rx;

        drive.drive(y, x, rx);

        // =======================
        // Shooter enable/disable (gamepad2)
        // =======================
        if (gamepad2.right_bumper) shooter.setEnabled(true);
        if (gamepad2.left_bumper)  shooter.setEnabled(false);

        // =======================
        // Toggle AutoAim + Tuning (gamepad2)
        // =======================
        boolean start2 = gamepad2.start;
        if (start2 && !lastStart2) autoAimEnabled = !autoAimEnabled;
        lastStart2 = start2;

        boolean back2 = gamepad2.back;
        if (back2 && !lastBack2) shooter.setTuningMode(!shooter.isTuningMode());
        lastBack2 = back2;

        // =======================
        // Dpad tuning + manual RPM adjust (gamepad2)
        // =======================
        handleLiveTuningAndRpmAdjust();

        // =======================
        // Fixed shot override (HOLD B)
        // =======================
        boolean fixedShotHeld = gamepad1.b;

        // =======================
        // Setpoints (Fixed overrides AutoAim)
        // =======================
        double targetRPM;
        double hoodPos;

        if (fixedShotHeld) {
            // HARD OVERRIDE: ignore camera/AutoAim entirely while held
            targetRPM = FIXED_SHOT_RPM;
            hoodPos = FIXED_HOOD_POS;

            // Make sure shooter is enabled while holding fixed-shot
            shooter.setEnabled(true);

        } else {
            // Normal behavior (autoaim / manual)
            if (autoAimEnabled) {
                boolean distFresh = (System.currentTimeMillis() - lastGoodDistanceMs) <= GOOD_DIST_STALE_MS;
                if (distFresh) {
                    targetRPM = ShooterModel.rpmFromDistance(lastGoodDistanceIn);
                    hoodPos   = ShooterModel.hoodFromDistance(lastGoodDistanceIn);
                } else {
                    targetRPM = manualTargetRPM;
                    hoodPos   = hood.getTargetPos();
                }
            } else {
                targetRPM = manualTargetRPM;
                hoodPos   = hood.getTargetPos();
            }
        }

        shooter.setTargetRPM(targetRPM);
        hood.setTargetPos(hoodPos);

        // =======================
        // Intake/Feeder (gamepad1)
        // Aim gate removed: feeder depends only on RPM readiness.
        // =======================

        // Y override: force both forward no matter what
        if (gamepad1.y) {
            intakeFeeder.setIntakePower(0.75);
            intakeFeeder.setFeederPower(0.75);

        } else if (fixedShotHeld) {
            // HOLD B: fixed shot sequence
            intakeFeeder.setIntakePower(1.0);

            boolean allowFeed = shooter.atSpeed(RPM_TOL);
            intakeFeeder.setFeederPower(allowFeed ? 1.0 : 0.0);

        } else {
            // Hold A: shoot-hold (auto enable shooter, intake forward, feed when at speed)
            if (gamepad1.a) {
                shooter.setEnabled(true);

                intakeFeeder.setIntakePower(1.0);

                boolean allowFeed = shooter.atSpeed(RPM_TOL);
                intakeFeeder.setFeederPower(allowFeed ? 1.0 : 0.0);

            } else {
                // Normal intake: triggers
                double in = gamepad1.right_trigger;
                double out = gamepad1.left_trigger;

                if (in > 0.05) intakeFeeder.setIntakePower(in);
                else if (out > 0.05) intakeFeeder.setIntakePower(-out);
                else intakeFeeder.setIntakePower(0.0);

                // Feeder reverse moved to X (since B is fixed shot now)
                if (gamepad1.x) intakeFeeder.setFeederPower(-1.0);
                else intakeFeeder.setFeederPower(0.0);
            }
        }

        // =======================
        // Update subsystems
        // =======================
        shooter.update();
        hood.update();
        intakeFeeder.update();

        // =======================
        // Telemetry
        // =======================
        telemetry.addData("FixedShot(B held)", fixedShotHeld);
        telemetry.addData("AutoAim", autoAimEnabled);
        telemetry.addData("Shooter enabled", shooter.isEnabled());
        telemetry.addData("Mode", shooter.isTuningMode() ? "FF_ONLY (kV)" : "PIDF");
        telemetry.addData("RPM cur/target", "%.0f / %.0f", shooter.getRPM(), shooter.getTargetRPM());
        telemetry.addData("AtSpeed", shooter.atSpeed(RPM_TOL));
        telemetry.addData("Power", "%.3f", shooter.getLastPower());
        telemetry.addData("Hood", "%.3f", hood.getTargetPos());

        telemetry.addData("Camera fault", (System.currentTimeMillis() < cameraFaultUntilMs));
        telemetry.addData("Camera hasTarget", cameraRead.hasTarget);
        telemetry.addData("Camera pipeline", cameraRead.pipeline);
        telemetry.addData("Camera tx/ty", "%.2f / %.2f", cameraRead.tx, cameraRead.ty);
        telemetry.addData("Dist(lastGood)", "%.1f", lastGoodDistanceIn);

        telemetry.update();
    }

    // =========================
    // SAFE CAMERA WRAPPERS
    // =========================

    private void safeCameraUpdate() {
        long now = System.currentTimeMillis();
        if (now < cameraFaultUntilMs) return;

        try {
            logitech.update();
        } catch (Exception e) {
            cameraFaultUntilMs = now + CAMERA_FAULT_COOLDOWN_MS;
        }
    }

    private CameraRead safeCameraRead() {
        CameraRead out = new CameraRead();

        long now = System.currentTimeMillis();
        if (now < cameraFaultUntilMs) {
            out.hasTarget = false;
            out.pipeline = -1;
            out.tx = 0;
            out.ty = 0;
            return out;
        }

        try {
            out.hasTarget = logitech.hasTarget();
            out.pipeline = logitech.getPipelineIndex();
            out.tx = logitech.getTxDeg();
            out.ty = logitech.getTyDeg();

            // Update last-good distance only when a target exists
            if (out.hasTarget) {
                double d = logitech.getDistanceInches();
                if (!Double.isNaN(d) && !Double.isInfinite(d) && d > 0.1 && d < 250.0) {
                    lastGoodDistanceIn = d;
                    lastGoodDistanceMs = now;
                }
            }
        } catch (Exception e) {
            cameraFaultUntilMs = now + CAMERA_FAULT_COOLDOWN_MS;
            out.hasTarget = false;
            out.pipeline = -1;
            out.tx = 0;
            out.ty = 0;
        }

        return out;
    }

    @Override
    public void stop() {
        if (logitech != null) {
            logitech.close();
        }
    }

    private static class CameraRead {
        boolean hasTarget = false;
        int pipeline = -1;
        double tx = 0.0;
        double ty = 0.0;
    }

    // =========================
    // TUNING / RPM ADJUST
    // =========================
    private void handleLiveTuningAndRpmAdjust() {
        long now = System.currentTimeMillis();
        if (now - lastAdjustMs < ADJUST_PERIOD_MS) return;

        boolean up = gamepad2.dpad_up;
        boolean down = gamepad2.dpad_down;
        if (!up && !down) return;

        int dir = up ? +1 : -1;

        boolean fine = gamepad2.left_trigger > 0.5;

        if (gamepad2.x) {
            double step = fine ? 0.000002 : 0.00001;
            shooter.kP = MathUtil.clip(shooter.kP + dir * step, 0.0, 0.01);
            lastAdjustMs = now;
            return;
        }

        if (gamepad2.y) {
            double step = fine ? 0.000002 : 0.00001;
            shooter.kD = MathUtil.clip(shooter.kD + dir * step, 0.0, 0.05);
            lastAdjustMs = now;
            return;
        }

        if (gamepad2.a) {
            double step = fine ? 0.0000002 : 0.000001;
            shooter.kV = MathUtil.clip(shooter.kV + dir * step, 0.0, 0.01);
            lastAdjustMs = now;
            return;
        }

        double rpmStep = fine ? 10 : 25;
        manualTargetRPM = MathUtil.clip(manualTargetRPM + dir * rpmStep, 0.0, 6500.0);
        lastAdjustMs = now;
    }
}
