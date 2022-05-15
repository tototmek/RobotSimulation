using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace RobotSimulation
{
    class RobotController
    {
        public Robot robot;
        public double linearSpeed;
        public double angularSpeed;
        public double maxLinearSpeed;
        public double maxAngularSpeed;

        public RobotController(Robot robot)
        {
            this.robot = robot;
        }

        public void SetCommand(double linVel, double angVel)
        {
            //Calculate maxVelocities
            double sum = Math.Abs(linVel) + Math.Abs(angVel);
            double linRatio;
            double angRatio;
            if (sum != 0)
            {
                linRatio = Math.Abs(linVel) / (sum);
                angRatio = Math.Abs(angVel) / (sum);
            }
            else
            {
                linRatio = 0;
                angRatio = 0;
            }
            maxLinearSpeed = 0.5 * linRatio * robot.wheels.left.maxSpeed * robot.wheels.left.diameter;
            maxAngularSpeed = 0.5 * angRatio * (robot.wheels.left.maxSpeed + robot.wheels.right.maxSpeed) * robot.wheels.left.diameter / robot.wheelDistance;

            //Clamp velocities
            linVel = Math.Min(maxLinearSpeed, Math.Max(-maxLinearSpeed, linVel));
            angVel = Math.Min(maxAngularSpeed, Math.Max(-maxAngularSpeed, angVel));

            //Calculate steering
            double leftSpeed = 2 * linVel / robot.wheels.left.diameter;
            double rightSpeed = 2 * linVel / robot.wheels.right.diameter;
            double dW = angVel * robot.wheelDistance / robot.wheels.left.diameter;
            leftSpeed += dW;
            rightSpeed -= dW;
            robot.controllers.left.SetWheelCommand(leftSpeed);
            robot.controllers.right.SetWheelCommand(rightSpeed);
            linearSpeed = linVel;
            angularSpeed = angVel;
        }
    }

    class PathFollower
    {
        public Path path;
        public RobotController controller;
        public PID linearPID = new PID();
        public PID angularPID = new PID();
        public Point currentPoint;
        public double pointAchievedThreshold = 0.5;

        public PathFollower(RobotController controller, Path path)
        {
            this.path = path;
            this.controller = controller;
            linearPID.K = 2;
            linearPID.Ti = 5;
            linearPID.Td = 0;
            angularPID.K = 12;
            angularPID.Ti = 25;
            angularPID.Td = 0.3;
        }

        public void Step(double sampleTime)
        {
            if (!path.HasPoints()) {
                controller.SetCommand(0, 0);
                return;
            };
            currentPoint = path.GetFirst();
            //Calculate distance to first point
            double distance = Utils.PTPDistance(controller.robot.position, currentPoint);
            //Check if point is achieved
            if (distance < pointAchievedThreshold)
            {
                linearPID.Reset();
                angularPID.Reset();
                path.RemoveFirst();
                if (path.HasPoints())
                    currentPoint = path.GetFirst();
                return;
            }

            //Calculate target orientation
            double angle = controller.robot.position.orientation;
            double targetAngle = Math.Atan2(currentPoint.y - controller.robot.position.y, currentPoint.x - controller.robot.position.x);
            double dth = Math.Atan2(Math.Sin(targetAngle - angle), Math.Cos(targetAngle - angle));
            controller.SetCommand(linearPID.MV(distance, sampleTime), angularPID.MV(dth, sampleTime));
        }
    }

    class Path
    {
        private List<Point> points;

        public Path()
        {
            points = new List<Point>();
        }
        
        public void AddPoint(Point point)
        {
            points.Add(point);
        }

        public Point GetFirst()
        {
            return points[0];
        }

        public bool HasPoints()
        {
            return points.Count > 0;
        }

        public void RemoveFirst()
        {
            if (points.Count > 0)
            {
                points.RemoveAt(0);
            }
        }

        public List<Point> GetPoints()
        {
            return points;
        }

        public void Reverse()
        {
            points.Reverse();
        }
    }
}
