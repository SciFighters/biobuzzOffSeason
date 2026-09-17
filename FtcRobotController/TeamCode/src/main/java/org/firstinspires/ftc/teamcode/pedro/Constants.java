package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {
    public static MecanumConfig driveConfig = new MecanumConfig(
            c -> {
                c.frontLeftName.set("frontLeft");
                c.backLeftName.set("rearLeft");
                c.frontRightName.set("frontRight");
                c.backRightName.set("rearRight");

                c.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);
                c.backLeftDirection.set(DcMotorSimple.Direction.REVERSE);
                c.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);
                c.backRightDirection.set(DcMotorSimple.Direction.FORWARD);
            }
    );
    public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
        c.name.set("pinpoint");
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        c.xPodOffset.set(1.25);
        c.yPodOffset.set(-5.15);
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
    });

    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.2138701921658848);
                Controller secondaryTranslationalForward = Controller.proportional(0.0790193212885103);
                Controller primaryTranslationalLateral = Controller.proportional(0.38823400204767305);
                Controller secondaryTranslationalLateral = Controller.proportional(0.1434420899530234);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.016780532024505256));
                c.brake.set(Controller.proportionalFeedforward(0.014263452220829467));

                c.headingFeedback.set(Controller.proportional(3.1455222744182576));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.033881085567348, 0.013486643229220533));

                c.linearBrakeCoefficients.set(Matrix.diag(0.0627930595279697, 0.034541119274650804));
                c.quadraticBrakeCoefficients.set(Matrix.diag(0.0029067592209674237, 0.003247599753186755));

                c.maxAchievableForwardVelocity.set(62.08259490971689);
                c.maxAchievableStrafeVelocity.set(49.29470041523689);
                c.naturalForwardDeceleration.set(37.90848211445049);
                c.naturalStrafeDeceleration.set(54.871617546208334);
            }
    );
    public static Follower create(HardwareMap h) {
        // TeleOp only; supply a tuned algorithm before using paths, hold, isBusy, debug
        return new Follower(new PinpointLocalizer(h, localizerConfig), new Mecanum(h, driveConfig), null);
    }
}
