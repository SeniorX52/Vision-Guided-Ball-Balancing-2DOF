package com.android.ball_balancing_system;

import org.apache.commons.math3.analysis.interpolation.SplineInterpolator;
import org.apache.commons.math3.analysis.polynomials.PolynomialSplineFunction;
import java.util.ArrayList;
import java.util.List;

public class PathSmoother {

    /**
     * Simplifies and smooths a mouse movement path
     * @param points Original mouse points (will be modified)
     * @param targetPointCount Desired number of points in simplified path (e.g., 20-50)
     * @param smoothingPasses Number of smoothing iterations (e.g., 2-3)
     */
    public static void simplifyAndSmoothPath(List<double[]> points, int targetPointCount, int smoothingPasses) {
        if (points.size() <= targetPointCount) {
            // Already simple enough, just smooth it
            smoothPath(points);
            return;
        }
        List<double[]> simplified = simplifyPath(points, targetPointCount);
        for (int i = 0; i < smoothingPasses; i++) {
            smoothPath(simplified);
        }
        points.clear();
        points.addAll(simplified);
    }

    /**
     * Reduces the number of points while preserving overall shape
     */
    private static List<double[]> simplifyPath(List<double[]> points, int targetPointCount) {
        List<double[]> simplified = new ArrayList<>();
        int keepEveryN = Math.max(1, points.size() / targetPointCount);

        // Always keep first and last points
        simplified.add(points.get(0));

        // Sample points at regular intervals
        for (int i = 1; i < points.size() - 1; i++) {
            if (i % keepEveryN == 0 || isSignificantChange(points, i)) {
                simplified.add(points.get(i));
            }
        }

        simplified.add(points.get(points.size()-1));
        return simplified;
    }

    /**
     * Checks if a point represents a significant direction change
     */
    private static boolean isSignificantChange(List<double[]> points, int index) {
        if (index < 1 || index >= points.size() - 1) return false;

        double[] prev = points.get(index-1);
        double[] curr = points.get(index);
        double[] next = points.get(index+1);

        double dx1 = curr[0] - prev[0];
        double dy1 = curr[1] - prev[1];
        double dx2 = next[0] - curr[0];
        double dy2 = next[1] - curr[1];

        double len1 = Math.sqrt(dx1*dx1 + dy1*dy1);
        double len2 = Math.sqrt(dx2*dx2 + dy2*dy2);
        if (len1 == 0 || len2 == 0) return false;

        dx1 /= len1; dy1 /= len1;
        dx2 /= len2; dy2 /= len2;
        double dot = dx1*dx2 + dy1*dy2;
        return dot < 0.707; // cos(45°) ≈ 0.707
    }

    /**
     * Creates a smooth spline through the points
     */
    private static void smoothPath(List<double[]> points) {
        if (points.size() < 3) return;
        try{
            double[] t = new double[points.size()];
            for (int i = 1; i < t.length; i++) {
                double dx = points.get(i)[0] - points.get(i-1)[0];
                double dy = points.get(i)[1] - points.get(i-1)[1];
                t[i] = t[i-1] + Math.sqrt(dx*dx + dy*dy);
            }
            double[] x = points.stream().mapToDouble(p -> p[0]).toArray();
            double[] y = points.stream().mapToDouble(p -> p[1]).toArray();
            SplineInterpolator interpolator = new SplineInterpolator();
            PolynomialSplineFunction xSpline = interpolator.interpolate(t, x);
            PolynomialSplineFunction ySpline = interpolator.interpolate(t, y);
            List<double[]> smoothed = new ArrayList<>();
            double totalT = t[t.length-1];
            for (double ti = 0; ti <= totalT; ti += totalT/(points.size()*2)) {
                smoothed.add(new double[]{xSpline.value(ti), ySpline.value(ti)});
            }

            points.clear();
            points.addAll(smoothed);
        }
        catch (Exception _){

        }
    }
}