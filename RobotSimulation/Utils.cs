using System;

namespace RobotSimulation
{

    static class Utils
    {
        public static Random random = new Random();

        public static double SampleGaussian(double mean, double stddev)
        {
            // The method requires sampling from a uniform random of (0,1]
            // but Random.NextDouble() returns a sample of [0,1).
            double x1 = 1 - Utils.random.NextDouble();
            double x2 = 1 - Utils.random.NextDouble();

            double y1 = Math.Sqrt(-2.0 * Math.Log(x1)) * Math.Cos(2.0 * Math.PI * x2);
            return y1 * stddev + mean;
        }

        public static double Lag(double lag, double sampleTime, double prev_y, double prev_x)
        {
            // Models a transmitance G(s) = 1 / (lag * s + 1)
            return 1 / lag * ((lag - sampleTime) * prev_y + sampleTime * prev_x);
        }

        public static double PTPDistance(Point p1, Point p2)
        {
            return Math.Sqrt(Math.Pow(p1.x - p2.x, 2) + Math.Pow(p1.y - p2.y, 2));
        }
    }

    class PID
    {
        private double u_k_1 = 0;
        private double e_k_1 = 0;
        private double e_k_2 = 0;
        public double K = 6;
        public double Ti = 15;
        public double Td = 0.1;

        public void Reset()
        {
            u_k_1 = 0;
            e_k_1 = 0;
            e_k_2 = 0;
        }

        public double MV(double PV, double STPT, double sampleTime)
        {
            // Here implement regulation algorithm
            double r0 = K * (1 + (sampleTime / (2 * Ti)) + (Td / sampleTime));
            double r1 = K * ((sampleTime / (2 * Ti)) - 2 * (Td / sampleTime) - 1);
            double r2 = K * Td / sampleTime;
            double e = STPT - PV;
            u_k_1 = u_k_1 + r0 * e + r1 * e_k_1 + r2 * e_k_2;
            e_k_2 = e_k_1;
            e_k_1 = e;
            return u_k_1;
        }

        public double MV(double e, double sampleTime)
        {
            // Here implement regulation algorithm
            double r0 = K * (1 + (sampleTime / (2 * Ti)) + (Td / sampleTime));
            double r1 = K * ((sampleTime / (2 * Ti)) - 2 * (Td / sampleTime) - 1);
            double r2 = K * Td / sampleTime;
            u_k_1 = u_k_1 + r0 * e + r1 * e_k_1 + r2 * e_k_2;
            e_k_2 = e_k_1;
            e_k_1 = e;
            return u_k_1;
        }
    }
}
