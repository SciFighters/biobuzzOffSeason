package org.firstinspires.ftc.teamcode.subSystems;

import java.util.ArrayDeque;
import java.util.HashMap;
import java.util.Map;
import java.util.Random;

public final class FlySub {

    public static final int neuronCount = 32;
    public static final double maxStepS = 0.1;
    private static final double timeConstantS = 0.1;
    public static final double learningRate = 0.25;
    public static final double rewardWindowS = 2.0;
    public static final double samplePeriodS = 0.1;
    public static final double positionCellIn = 12.0;
    public static final int headingBins = 8;
    public static final double minNetworkPower = 0.04;
    public static final double maxPower = 0.25;
    public static final double confidenceStep = 0.1;

    private final double[] activity = new double[neuronCount];
    private final double[] shifted = new double[neuronCount];
    private final double[] cos = new double[neuronCount];
    private final double[] sin = new double[neuronCount];
    private final Map<String, double[]> policy = new HashMap<>();
    private final ArrayDeque<Action> history = new ArrayDeque<>();
    private final Random random = new Random();
    private final double[] exploration = new double[3];

    private double timeS;
    private double lastSampleS = -samplePeriodS;
    private double nextExploreS;
    private double fieldX;
    private double fieldY;
    private boolean positionValid;
    private int rewards;

    private static class Action {
        final String state;
        final double timeS;
        final double[] movement;

        Action(String state, double timeS, double forward, double strafe, double turn) {
            this.state = state;
            this.timeS = timeS;
            this.movement = new double[] { forward, strafe, turn };
        }
    }

    private boolean recurrent = true;

    public FlySub() {
        for (int i = 0; i < neuronCount; i++) {
            double angle = i * 2 * Math.PI / neuronCount;
            cos[i] = Math.cos(angle);
            sin[i] = Math.sin(angle);
        }

        reset();
    }

    // New home invalidates learned heading associations.
    public void reset() {
        forgetAction();
        policy.clear();
        rewards = 0;
        timeS = 0;
        nextExploreS = 0;
        positionValid = false;
        for (int i = 0; i < neuronCount; i++) {
            activity[i] = Math.max(0, cos[i]);
        }
    }

    // Positive rate is CCW in rad/s; dt is in seconds.
    // False leaves state unchanged; stop and reset after missed motion.
    public boolean update(double turnRate, double dt) {
        if (!Double.isFinite(turnRate) || !Double.isFinite(dt)
                || dt <= 0 || dt > maxStepS) {
            return false;
        }

        double rotation = turnRate * dt;
        if (!Double.isFinite(rotation)) {
            return false;
        }

        timeS += dt;
        pruneHistory();

        int steps = (int) Math.ceil(dt / 0.01);
        double blend = 1 - Math.exp(-dt / steps / timeConstantS);
        double shift = wrap(rotation) * neuronCount / (2 * Math.PI * steps);

        for (int step = 0; step < steps; step++) {
            double total = 0;
            double x = 0;
            double y = 0;

            for (int i = 0; i < neuronCount; i++) {
                double source = i - shift;
                int left = (int) Math.floor(source);
                double fraction = source - left;
                int index = Math.floorMod(left, neuronCount);

                shifted[i] = activity[index] * (1 - fraction)
                        + activity[(index + 1) % neuronCount] * fraction;
                total += shifted[i];
                x += shifted[i] * cos[i];
                y += shifted[i] * sin[i];
            }

            for (int i = 0; i < neuronCount; i++) {
                // Weights: (6*cos(angle_i-angle_j)-1)/N; local excitation, distant inhibition.
                double input = recurrent
                        ? (6 * (x * cos[i] + y * sin[i]) - total) / neuronCount : 0;
                double target = Math.max(0, Math.min(1, input));
                activity[i] = shifted[i] + blend * (target - shifted[i]);
            }
        }

        return true;
    }

    // Signed radians; NaN means memory faded.
    public double getHeading() {
        double x = 0;
        double y = 0;

        for (int i = 0; i < neuronCount; i++) {
            x += activity[i] * cos[i];
            y += activity[i] * sin[i];
        }

        return Math.hypot(x, y) / neuronCount < 0.02 ? Double.NaN : Math.atan2(y, x);
    }

    // Absolute strength detects faded memory.
    public double getStrength() {
        double x = 0;
        double y = 0;

        for (int i = 0; i < neuronCount; i++) {
            x += activity[i] * cos[i];
            y += activity[i] * sin[i];
        }

        return Math.hypot(x, y) / neuronCount;
    }

    public void setRecurrent(boolean enabled) {
        recurrent = enabled;
    }

    public boolean isRecurrent() {
        return recurrent;
    }

    public boolean setPosition(double xIn, double yIn) {
        positionValid = Double.isFinite(xIn) && Double.isFinite(yIn);
        if (positionValid) {
            fieldX = xIn;
            fieldY = yIn;
        } else {
            forgetAction();
        }
        return positionValid;
    }

    private String getState() {
        double heading = getHeading();
        if (!positionValid || !Double.isFinite(heading)) {
            return null;
        }
        int bin = Math.floorMod((int) Math.round(heading * headingBins / (2 * Math.PI)), headingBins);
        return (long) Math.floor(fieldX / positionCellIn) + ":"
                + (long) Math.floor(fieldY / positionCellIn) + ":" + bin;
    }

    // Store direction; confidence sets network speed.
    public void recordAction(double forward, double strafe, double turn) {
        String state = getState();
        double total = Math.abs(forward) + Math.abs(strafe) + Math.abs(turn);
        if (state == null || !Double.isFinite(total) || total < 0.05
                || timeS - lastSampleS + 1e-9 < samplePeriodS) {
            return;
        }
        pruneHistory();
        history.addLast(new Action(state, timeS, forward / total, strafe / total, turn / total));
        lastSampleS = timeS;
    }

    // Credit recent actions at their original positions; newer actions count more.
    public boolean reward() {
        pruneHistory();
        if (getState() == null || history.isEmpty()) {
            return false;
        }
        Map<String, double[]> credits = new HashMap<>();
        for (Action action : history) {
            double weight = 1 - (timeS - action.timeS) / rewardWindowS;
            double[] credit = credits.computeIfAbsent(action.state, key -> new double[5]);
            for (int axis = 0; axis < 3; axis++) {
                credit[axis] += weight * action.movement[axis];
            }
            credit[3] += weight;
            credit[4] = Math.max(credit[4], weight);
        }
        for (Map.Entry<String, double[]> entry : credits.entrySet()) {
            double[] credit = entry.getValue();
            double[] action = policy.computeIfAbsent(entry.getKey(), key -> new double[4]);
            double blend = action[3] == 0 ? 1 : learningRate;
            for (int axis = 0; axis < 3; axis++) {
                action[axis] += blend * (credit[axis] / credit[3] - action[axis]);
            }
            action[3] = Math.min(1, action[3] + confidenceStep * credit[4]);
        }
        rewards++;
        forgetAction();
        return true;
    }

    // Untrained states explore slowly, changing direction every 0.75 seconds.
    public void getNetworkAction(double[] output) {
        if (output == null || output.length < 3) {
            throw new IllegalArgumentException("Three action outputs required");
        }
        String state = getState();
        if (state == null) {
            output[0] = output[1] = output[2] = 0;
            return;
        }
        double[] action = policy.get(state);
        if (action != null) {
            System.arraycopy(action, 0, output, 0, 3);
        } else {
            if (timeS >= nextExploreS) {
                double total = 0;
                for (int axis = 0; axis < 3; axis++) {
                    exploration[axis] = random.nextDouble() * 2 - 1;
                    total += Math.abs(exploration[axis]);
                }
                if (total < 1e-9) {
                    exploration[0] = 1;
                    total = 1;
                }
                for (int axis = 0; axis < 3; axis++) {
                    exploration[axis] /= total;
                }
                nextExploreS = timeS + 0.75;
            }
            System.arraycopy(exploration, 0, output, 0, 3);
        }
    }

    public double getConfidence() {
        double[] action = policy.get(getState());
        return action == null ? 0 : action[3];
    }

    public double getNetworkPower() {
        return minNetworkPower + (maxPower - minNetworkPower) * getConfidence();
    }

    private void pruneHistory() {
        while (!history.isEmpty() && timeS - history.peekFirst().timeS >= rewardWindowS) {
            history.removeFirst();
        }
    }

    public void forgetAction() {
        history.clear();
        lastSampleS = -samplePeriodS;
    }

    public int getRewards() {
        return rewards;
    }

    public static double wrap(double radians) {
        return Math.atan2(Math.sin(radians), Math.cos(radians));
    }
}
