package frc.Lib;

import java.util.ArrayList;

/** Add your docs here. */
public class BezierPath {
    private final AdvancedPose2D[] m_points;
    public BezierPath(AdvancedPose2D... Points) {
        m_points = Points;
    }

    public AdvancedPose2D[] getControlPoints() {
        return m_points;
    }

    public static double factorial(int k) {
        long m_k = 1;
        for (int i = 1; i <= k; i++) {
            m_k *= i;
        }
        return m_k;
    }

    public static double binomialCoefficient(int n, int i) {
        return factorial(n) / (factorial(i) * factorial(n - i));
    }

    public AdvancedPose2D outputPoint(double t) {
        double x = 0;
        double y = 0;
        int n = m_points.length - 1;
            
        for (int i = 0; i <= n; i++) {
            x += binomialCoefficient(n, i) * Math.pow((1 - t), (n - i)) * Math.pow(t, i) * m_points[i].getX();
            y += binomialCoefficient(n, i) * Math.pow((1 - t), (n - i)) * Math.pow(t, i) * m_points[i].getY();
        }

        return new AdvancedPose2D(x, y);
    }

    public ArrayList<AdvancedPose2D> outputPath(double numIntermediatePoints) {
        ArrayList<AdvancedPose2D> path = new ArrayList<AdvancedPose2D>();

        double interval = 1 / (numIntermediatePoints + 1);
        int n = m_points.length - 1;
        double x = 0;
        double y = 0;
        
        for (double t = 0; t <= 1; t += interval) {
            x = 0;
            y = 0;
            
            for (int i = 0; i <= n; i++) {
                x += binomialCoefficient(n, i) * Math.pow((1 - t), (n - i)) * Math.pow(t, i) * m_points[i].getX();
                y += binomialCoefficient(n, i) * Math.pow((1 - t), (n - i)) * Math.pow(t, i) * m_points[i].getY();
            }

            path.add(new AdvancedPose2D(x, y));
        }

        return path;
    }
}
