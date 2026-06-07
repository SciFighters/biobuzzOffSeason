package org.firstinspires.ftc.teamcode.Utilities.pid;

import java.util.ArrayDeque;
import java.util.Deque;

/**
 * Tracks PID performance statistics.
 *
 * <p>This class is purely informational and has no effect
 * on PID calculations.</p>
 *
 * <p>Statistics include:</p>
 *
 * <ul>
 *     <li>Total calculations</li>
 *     <li>Current calculations per second</li>
 *     <li>Average absolute error</li>
 *     <li>Maximum absolute error</li>
 * </ul>
 */
public class PIDStatistics {

    /**
     * One second in nanoseconds.
     */
    private static final long ONE_SECOND_NANOS =
            1_000_000_000L;

    /*--------------------------------------------------------------*/
    /* CALCULATION TRACKING                                         */
    /*--------------------------------------------------------------*/

    /**
     * Total number of PID calculations performed.
     */
    private long calculationCount = 0;

    /**
     * Stores timestamps of recent calculations.
     *
     * Used to compute rolling update rate.
     */
    private final Deque<Long> calculationTimes =
            new ArrayDeque<>();

    /*--------------------------------------------------------------*/
    /* ERROR TRACKING                                               */
    /*--------------------------------------------------------------*/

    /**
     * Running sum of absolute errors.
     */
    private double totalAbsoluteError = 0.0;

    /**
     * Largest absolute error observed.
     */
    private double maxError = 0.0;

    /*--------------------------------------------------------------*/
    /* UPDATE                                                       */
    /*--------------------------------------------------------------*/

    /**
     * Updates all statistics.
     *
     * This should be called once every PID calculation.
     *
     * @param error Current PID error.
     */
    public void update(double error) {

        long now = System.nanoTime();

        calculationCount++;

        double absoluteError = Math.abs(error);

        totalAbsoluteError += absoluteError;

        maxError = Math.max(maxError, absoluteError);

        calculationTimes.addLast(now);

        removeOldTimestamps(now);
    }

    /**
     * Removes timestamps older than one second.
     */
    private void removeOldTimestamps(long now) {

        while (!calculationTimes.isEmpty()
                && now - calculationTimes.peekFirst()
                > ONE_SECOND_NANOS) {

            calculationTimes.removeFirst();
        }
    }

    /*--------------------------------------------------------------*/
    /* GETTERS                                                      */
    /*--------------------------------------------------------------*/

    /**
     * Returns the total number of PID calculations performed.
     */
    public long getCalculationCount() {
        return calculationCount;
    }

    /**
     * Returns the current rolling calculations per second.
     *
     * Uses only the last second of activity.
     */
    public double getCalculationsPerSecond() {
        return calculationTimes.size();
    }

    /**
     * Returns the average absolute error seen over
     * the lifetime of this statistics object.
     */
    public double getAverageError() {

        if (calculationCount == 0) {
            return 0.0;
        }

        return totalAbsoluteError /
                calculationCount;
    }

    /**
     * Returns the largest absolute error ever observed.
     */
    public double getMaxError() {
        return maxError;
    }

    /*--------------------------------------------------------------*/
    /* RESET                                                        */
    /*--------------------------------------------------------------*/

    /**
     * Clears all statistics.
     */
    public void reset() {

        calculationCount = 0;

        totalAbsoluteError = 0.0;

        maxError = 0.0;

        calculationTimes.clear();
    }
}