package frc.Lib;

import java.util.ArrayList;

/** Add your docs here. */
public class Bezier {
    private final AdvancedPose2D[] m_points;
    public Bezier(AdvancedPose2D... points) {
        m_points = points;
    }

    public double factorial(int k) {
        int m_k = 1;
        for (int i = 0; i < k; i++) {
            m_k *= (k - i);
        }
        return m_k;
    }

    public double binomialCoefficient(int n, int i) {
        return factorial(n) / (factorial(i) * factorial(n - 1));
    }

    public AdvancedPose2D outputPoint(double t) {
        double x = 0;
        double y = 0;

        int n = m_points.length;
        for (int i = 0; i < n; i++) {
            x = binomialCoefficient(n, i) * Math.pow((1 - t), (n - i)) * Math.pow(t, i) * m_points[i].getX();
            y = binomialCoefficient(n, i) * Math.pow((1 - t), (n - i)) * Math.pow(t, i) * m_points[i].getY();
        }

        return new AdvancedPose2D(x, y);
    }

    public ArrayList<AdvancedPose2D> outputPath(double interval, double maxT) {
        ArrayList<AdvancedPose2D> path = new ArrayList<AdvancedPose2D>();
        
        for (double t = 0; t < Math.floor(maxT / interval); t += interval) {
            path.add(outputPoint(t));
        }

        return path;
    }
}
