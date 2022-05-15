using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace RobotSimulation
{
    class ProximitySensor
    {
        public NonLinearSensor sensor = new NonLinearSensor();
        public Point position = new Point();
        public Point relativePosition = new Point();
        public Point hitPoint = new Point();
        public Point actualHitPoint = new Point();
        public bool isHit = false;
        public double range = 2;
        public double rotationSpeed = 0;

        public ProximitySensor()
        {
            //Setting default deviation model
            sensor.stddev = 1;
            sensor.stddevFunc = x =>
            {
                //return 0.0993 + 0.3309 * x - 0.5143 * Math.Pow(x, 2) + 0.3022 * Math.Pow(x, 3) - 0.0706 * Math.Pow(x, 4) + 0.0073 * Math.Pow(x, 5) - 0.0003 * Math.Pow(x, 6);
                return 0.33;
            };
        }

        public double Measure(List<Obstacle> obstacles)
        {
            double min_distance = double.MaxValue;
            foreach (Obstacle obstacle in obstacles)
            {

                double x1 = position.x;
                double y1 = position.y;
                double x2 = x1 + range * Math.Cos(position.orientation);
                double y2 = y1 + range * Math.Sin(position.orientation);
                double x3 = obstacle.p1.x;
                double y3 = obstacle.p1.y;
                double x4 = obstacle.p2.x;
                double y4 = obstacle.p2.y;
                // calculate the direction of the lines
                double uA = ((x4 - x3) * (y1 - y3) - (y4 - y3) * (x1 - x3)) / ((y4 - y3) * (x2 - x1) - (x4 - x3) * (y2 - y1));
                double uB = ((x2 - x1) * (y1 - y3) - (y2 - y1) * (x1 - x3)) / ((y4 - y3) * (x2 - x1) - (x4 - x3) * (y2 - y1));

                // if uA and uB are between 0-1, lines are colliding
                if (uA >= 0 && uA <= 1 && uB >= 0 && uB <= 1)
                {
                    //Calculate the hitPoint
                    Point tempPoint = new Point();
                    tempPoint.x = x1 + (uA * (x2 - x1));
                    tempPoint.y = y1 + (uA * (y2 - y1));
                    double distance = Math.Sqrt(Math.Pow(position.x - tempPoint.x, 2) + Math.Pow(position.y - tempPoint.y, 2));
                    if (distance < min_distance)
                    {
                        min_distance = distance;
                        actualHitPoint.x = tempPoint.x;
                        actualHitPoint.y = tempPoint.y;
                    }
                }
            }
            isHit = !(min_distance > range);
            double d = sensor.Measure(min_distance);
            double ratio = d / min_distance;
            hitPoint.x = position.x + ((actualHitPoint.x - position.x) * ratio);
            hitPoint.y = position.y + ((actualHitPoint.y - position.y) * ratio);
            relativePosition.orientation += rotationSpeed;
            return d;
        }
    }

    class Obstacle
    {
        public Point p1;
        public Point p2;
        public Obstacle(double x1, double y1, double x2, double y2)
        {
            p1.x = x1;
            p1.y = y1;
            p2.x = x2;
            p2.y = y2;
        }
        public Obstacle(Point p1_, Point p2_)
        {
            p1 = p1_;
            p2 = p2_;
        }
    }
}
