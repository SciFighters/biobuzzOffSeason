package org.firstinspires.ftc.teamcode.subSystems;

// Run from teamcode:
// javac -d /tmp/fly-check subSystems/FlySub.java subSystems/FlySubCheck.java
// java -cp /tmp/fly-check org.firstinspires.ftc.teamcode.subSystems.FlySubCheck
public final class FlySubCheck {

    public static void main(String[] args) {
        FlySub flySub = new FlySub();
        checkHeading(flySub, 0);
        advance(flySub, 0, 1000);
        checkHeading(flySub, 0);
        advance(flySub, Math.PI / 2, 100);
        checkHeading(flySub, Math.PI / 2);
        advance(flySub, 0, 1000);
        checkHeading(flySub, Math.PI / 2);
        advance(flySub, Math.PI / 2, 200);
        checkHeading(flySub, -Math.PI / 2);
        advance(flySub, -Math.PI / 2, 300);
        checkHeading(flySub, 0);

        double before = flySub.getHeading();
        if (flySub.update(Double.NaN, 0.01) || flySub.update(1, 1)
                || flySub.update(1, 0) || flySub.update(1, -0.01)
                || flySub.update(Double.POSITIVE_INFINITY, 0.01)
                || flySub.update(1, Double.NaN) || flySub.getHeading() != before) {
            throw new AssertionError("Invalid input must be rejected without changing state");
        }
        flySub.setRecurrent(false);
        advance(flySub, 0, 100);
        if (!Double.isNaN(flySub.getHeading()) || flySub.getStrength() >= 0.02) {
            throw new AssertionError("Ablation must erase usable heading memory");
        }
        flySub.setRecurrent(true);
        flySub.reset();
        advance(flySub, -0.3, 175);
        checkHeading(flySub, -0.525);
        advance(flySub, 0, 1000);
        checkHeading(flySub, -0.525);
        flySub.reset();
        double expected = 0;
        double[] intervals = { 0.005, 0.02, 0.05, 0.01 };
        for (int i = 0; i < 400; i++) {
            double dt = intervals[i % intervals.length];
            if (!flySub.update(3, dt)) {
                throw new AssertionError("Variable step rejected");
            }
            expected += 3 * dt;
        }
        // Allow 10 degrees of drift over four turns.
        double drift = Math.abs(FlySub.wrap(flySub.getHeading() - expected));
        if (!Double.isFinite(drift) || drift > Math.toRadians(10)) {
            throw new AssertionError("Excessive multi-turn drift: " + Math.toDegrees(drift));
        }
        flySub.reset();
        flySub.setPosition(0, 0);
        double[] action = new double[3];
        flySub.getNetworkAction(action);
        checkPower(action);
        if (sum(action) < 0.99 || flySub.getNetworkPower() != FlySub.minNetworkPower
                || flySub.getConfidence() != 0 || flySub.reward()) {
            throw new AssertionError("Untrained exploration must start slowly without confidence");
        }
        flySub.recordAction(1, 0, 0);
        advance(flySub, 0, 10);
        flySub.setPosition(24, 0);
        flySub.recordAction(0, -1, 0);
        advance(flySub, 0, 10);
        if (!flySub.reward() || flySub.reward()) {
            throw new AssertionError("Reward must credit history once");
        }
        flySub.getNetworkAction(action);
        if (action[1] != -1 || action[0] != 0) {
            throw new AssertionError("Recent strafe must be learned at its field position");
        }
        flySub.setPosition(0, 0);
        flySub.getNetworkAction(action);
        if (action[0] != 1 || action[1] != 0 || flySub.getConfidence() <= 0
                || flySub.getNetworkPower() <= FlySub.minNetworkPower) {
            throw new AssertionError("Earlier movement must receive credit at its original position");
        }
        flySub.setPosition(0, 24);
        if (flySub.getConfidence() != 0) {
            throw new AssertionError("Field Y must distinguish states");
        }
        flySub.setPosition(-24, 0);
        if (flySub.getConfidence() != 0) {
            throw new AssertionError("New positions must start with low confidence");
        }
        flySub.setPosition(0, 0);
        advance(flySub, Math.PI, 100);
        if (flySub.getConfidence() != 0) {
            throw new AssertionError("Heading must distinguish states at the same position");
        }
        advance(flySub, -Math.PI, 100);
        double oldPower = flySub.getNetworkPower();
        for (int i = 0; i < 20; i++) {
            flySub.recordAction(1, 0, 0);
            if (!flySub.reward()) {
                throw new AssertionError("Fresh reward rejected");
            }
        }
        if (flySub.getNetworkPower() <= oldPower || flySub.getNetworkPower() != FlySub.maxPower
                || flySub.getConfidence() != 1) {
            throw new AssertionError("Repeated rewards must raise speed to the cap");
        }
        flySub.reset();
        flySub.setPosition(0, 0);
        flySub.recordAction(1, 0, 0);
        advance(flySub, 0, 50);
        flySub.recordAction(0, 0, 1);
        flySub.reward();
        flySub.getNetworkAction(action);
        if (action[0] <= 0 || action[2] <= action[0]) {
            throw new AssertionError("Reward must include movement and favor more recent turning");
        }
        checkPower(action);
        flySub.recordAction(-1, 1, -1);
        advance(flySub, 0, 201);
        if (flySub.reward()) {
            throw new AssertionError("Expired actions must not receive rewards");
        }
        flySub.recordAction(-1, 1, -1);
        flySub.forgetAction();
        if (flySub.reward()) {
            throw new AssertionError("Cancelled history must not receive rewards");
        }
        flySub.reset();
        flySub.setPosition(0, 0);
        flySub.recordAction(0.4, -0.2, 0.1);
        flySub.reward();
        flySub.getNetworkAction(action);
        if (action[0] <= 0 || action[1] >= 0 || action[2] <= 0
                || Math.abs(action[0] + 2 * action[1]) > 1e-9
                || Math.abs(action[0] - 4 * action[2]) > 1e-9) {
            throw new AssertionError("Reward must preserve the driving combination");
        }
        checkPower(action);
        flySub.recordAction(Double.NaN, 0, 0);
        if (flySub.reward()) {
            throw new AssertionError("Invalid action accepted");
        }
        flySub.recordAction(1, 0, 0);
        flySub.setPosition(Double.NaN, 0);
        flySub.getNetworkAction(action);
        if (sum(action) != 0 || flySub.reward()) {
            throw new AssertionError("Invalid position must stop output and clear history");
        }
        flySub.reset();
        flySub.setPosition(0, 0);
        if (flySub.getConfidence() != 0 || flySub.getRewards() != 0 || flySub.reward()) {
            throw new AssertionError("Reset must clear policy, confidence, and history");
        }
        flySub.setRecurrent(false);
        advance(flySub, 0, 100);
        flySub.recordAction(1, 0, 0);
        flySub.getNetworkAction(action);
        if (sum(action) != 0 || flySub.reward()) {
            throw new AssertionError("Faded heading must disable network output and reward");
        }
        System.out.println("FlySub checks passed: compass, pose learning, recent rewards, exploration, confidence, bounds, reset.");
    }

    private static double sum(double[] action) {
        return Math.abs(action[0]) + Math.abs(action[1]) + Math.abs(action[2]);
    }

    private static void checkPower(double[] action) {
        if (!Double.isFinite(sum(action)) || sum(action) > 1 + 1e-9) {
            throw new AssertionError("Combined driving exceeds the power budget");
        }
    }

    private static void advance(FlySub flySub, double turnRate, int steps) {
        for (int i = 0; i < steps; i++) {
            if (!flySub.update(turnRate, 0.01)) {
                throw new AssertionError("Valid step rejected");
            }
        }
    }

    private static void checkHeading(FlySub flySub, double expected) {
        double error = Math.abs(FlySub.wrap(flySub.getHeading() - expected));
        if (!Double.isFinite(error) || error > Math.toRadians(5)) {
            throw new AssertionError("Expected " + expected + ", got " + flySub.getHeading());
        }
    }
}
