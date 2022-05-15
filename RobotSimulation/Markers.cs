using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Media;

namespace RobotSimulation
{
    abstract class Marker
    {
        protected List<Drawable> elements = new List<Drawable>();

        public void AddTo(GraphicsLayer graphics)
        {
            foreach (Drawable element in elements)
            {
                graphics.Add(element);
            }
        }

        public void RemoveFrom(GraphicsLayer graphics)
        {
            foreach (Drawable element in elements)
            {
                graphics.Remove(element);
            }
        }

        public abstract void Update();
    }

    class RobotMarker : Marker
    {
        private Robot robot;
        private dCircle circle;
        private dRect wheelLeft;
        private dRect wheelRight;

        public RobotMarker(Robot robot)
        {
            this.robot = robot;
            circle = new dCircle();
            circle.SetRadius(robot.wheelDistance * 0.5);
            wheelLeft = new dRect();
            wheelLeft.SetSize(robot.wheels.left.diameter, 0.25);
            wheelRight = new dRect();
            wheelRight.SetSize(robot.wheels.right.diameter, 0.25);
            elements.Add(circle);
            elements.Add(wheelLeft);
            elements.Add(wheelRight);
        }

        public override void Update()
        {
            circle.SetPosition(robot.position.x, robot.position.y);
            double l = robot.wheelDistance * 0.5;
            wheelLeft.SetPosition(
                new Point(
                    robot.position.x + l * Math.Cos(robot.position.orientation + 1.57),
                    robot.position.y + l * Math.Sin(robot.position.orientation + 1.57),
                    robot.position.orientation));
            wheelRight.SetPosition(
                new Point(
                    robot.position.x + l * Math.Cos(robot.position.orientation - 1.57),
                    robot.position.y + l * Math.Sin(robot.position.orientation - 1.57),
                    robot.position.orientation));
        }

    }

    class ProximitySensorMarker : Marker
    {
        private ProximitySensor sensor;
        private dLine line;
        private dPoint point;

        public ProximitySensorMarker(ProximitySensor sensor)
        {
            this.sensor = sensor;
            line = new dLine();
            point = new dPoint();
            elements.Add(point);
            elements.Add(line);
        }

        public override void Update()
        {
            line.SetP1(
                sensor.position.x,
                sensor.position.y);
            line.SetP2(
                sensor.position.x + sensor.range * Math.Cos(sensor.position.orientation),
                sensor.position.y + sensor.range * Math.Sin(sensor.position.orientation));
            if (sensor.isHit)
            {
                point.SetPosition(sensor.hitPoint.x, sensor.hitPoint.y);
                line.SetP2(sensor.actualHitPoint.x, sensor.actualHitPoint.y);
            } else
            {
                point.SetPosition(135531, 144415);
            }
        }
    }

    class ObstacleMarker : Marker
    {
        private dLine line;
        private Obstacle obstacle;

        public ObstacleMarker(Obstacle obstacle)
        {
            this.obstacle = obstacle;
            line = new dLine();
            line.SetColor(Brushes.Black);
            elements.Add(line);
        }

        public override void Update()
        {
            line.SetP1(
                obstacle.p1.x,
                obstacle.p1.y);
            line.SetP2(
                obstacle.p2.x,
                obstacle.p2.y);
        }
    }

    class PathMarker : Marker
    {
        public Path path;
        private int maxLength = 32;

        public PathMarker(Path path)
        {
            this.path = path;
            for (int i = 0; i < maxLength; i++)
            {
                //Add a line
                dLine line = new dLine();
                line.SetColor(Brushes.Yellow);
                elements.Add(line);
            }
        }

        public override void Update()
        {
            var pathPoints = path.GetPoints();
            int lineCount = pathPoints.Count - 1;
            for (int i = 0; i < Math.Min(lineCount, maxLength); i++)
            {
                ((dLine)elements[i]).SetP1(pathPoints[i].x, pathPoints[i].y);
                ((dLine)elements[i]).SetP2(pathPoints[i + 1].x, pathPoints[i + 1].y);
            }
        }
    }

    class PathFollowerMarker : Marker
    {
        public PathFollower pathFollower;
        private dLine line;

        public PathFollowerMarker (PathFollower pathFollower)
        {
            this.pathFollower = pathFollower;
            line = new dLine();
            line.SetColor(Brushes.Yellow);
            line.SetDashLine(3);
            elements.Add(line);
        }

        public override void Update()
        {
            if (pathFollower.path.HasPoints())
            {
                line.SetP2(pathFollower.controller.robot.position.x, pathFollower.controller.robot.position.y);
                line.SetP1(pathFollower.currentPoint.x, pathFollower.currentPoint.y);
            } else
            {
                line.SetP1(2137, 2137);
                line.SetP2(2137, 2137);
            }
        }
    }
}
