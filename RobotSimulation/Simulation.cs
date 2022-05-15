using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace RobotSimulation
{
    interface IPhysicsComponent
    {
        void Update(double sampleTime);
    }

    class Robot : IPhysicsComponent
    {
        public struct Wheels {
            public Wheel left;
            public Wheel right;
        };
        public struct Controllers
        {
            public WheelController left;
            public WheelController right;
        };
        public Wheels wheels;
        public Controllers controllers;
        public double wheelDistance;
        public Point position;
        public List<ProximitySensor> proximitySensors = new List<ProximitySensor>();

        public Robot(Point position_=new Point())
        {
            position = position_;
            wheels.left = new Wheel();
            wheels.left.encoder = new Sensor();
            wheels.right = new Wheel();
            wheels.right.encoder = new Sensor();
        }

        public void Update(double sampleTime)
        {
            //Here I define the physics model of the robot;
            double r = 0.5 * (wheels.left.diameter + wheels.right.diameter);
            double dpl = wheels.left.GetActualSpeed() * sampleTime;
            double dpr = wheels.right.GetActualSpeed() * sampleTime;
            double dp = r * (dpl + dpr) * 0.5;
            double dth = r * (dpl - dpr) / wheelDistance;
            position.x += dp * Math.Cos(position.orientation + dth * 0.5);
            position.y += dp * Math.Sin(position.orientation + dth * 0.5);
            position.orientation += dth;

            //Moving attached sensors
            foreach (ProximitySensor sensor in proximitySensors)
            {
                sensor.position.x = position.x + sensor.relativePosition.x * Math.Cos(position.orientation) - sensor.relativePosition.y * Math.Sin(position.orientation);
                sensor.position.y = position.y + sensor.relativePosition.x * Math.Sin(position.orientation) + sensor.relativePosition.y * Math.Cos(position.orientation);
                sensor.position.orientation = position.orientation + sensor.relativePosition.orientation;
            }
        }

        public void Tick(double sampleTime)
        {
            //Controllers tick
            controllers.left.Step(sampleTime);
            controllers.right.Step(sampleTime);
        }
    }
    

    struct Point
    {
        public double x;
        public double y;
        public double orientation;
        public Point(double x_=0, double y_=0, double orientation_=0)
        {
            x = x_;
            y = y_;
            orientation = orientation_;
        }
    }

    class Wheel : IPhysicsComponent
    {
        public double diameter;
        public double maxAcceleration;
        public double maxSpeed;
        public Sensor encoder;

        private double speed;
        private double effort = 0;
        private double acceleration = 0;

        public double GetSpeed()
        {
            return encoder.Measure(speed);
        }

        public double GetAcceleration()
        {
            return encoder.Measure(acceleration);
        }

        public double GetActualSpeed()
        {
            return speed;
        }

        public void SetEffort(double value)
        {
            //Desired acceleration of wheel motor
            effort = value;
        }

        public double GetEffort() { return effort; }

        public void Update(double sampleTime)
        {
            //Here I define the physics model of a wheel;
            acceleration = Utils.Lag(0.17, sampleTime, acceleration, effort);
            if (acceleration > maxAcceleration) acceleration = maxAcceleration;
            if (acceleration < -maxAcceleration) acceleration = -maxAcceleration;
            speed += acceleration * sampleTime;
            if (speed > maxSpeed) speed = maxSpeed;
            if (speed < -maxSpeed) speed = -maxSpeed;
            //speed = Utils.Lag(0.8, sampleTime, speed, effort);
        }
    }

    interface ISensor
    {
        double Measure(double value);
    }

    class Sensor : ISensor
    {
        public double stddev;
        public double Measure(double value)
        {
            return Utils.SampleGaussian(value, stddev);
        }
    }

    class NonLinearSensor : ISensor
    {
        public Func<double, double> stddevFunc;
        public double stddev;


        public double Measure(double value)
        {
            return Utils.SampleGaussian(value,stddev * Math.Min(1, Math.Max(0, stddev * stddevFunc(value))));
        }
    }
}
