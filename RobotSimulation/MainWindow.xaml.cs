using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Threading;
using System.Windows;
using System.Windows.Input;
using System.Windows.Threading;

namespace RobotSimulation
{
    /// <summary>
    /// Interaction logic for MainWindow.xaml
    /// </summary>
    public partial class MainWindow : Window
    {
        private Robot robot;
        private RobotController robotController;
        private PathFollower pathFollower;
        private PathFinder pathFinder;
        private List<IPhysicsComponent> physics;
        private GraphicsLayer graphics;
        private List<Obstacle> obstacles = new List<Obstacle>();
        private Point target = new Point();

        private Marker path_marker;

        const double PHYSICS_SAMPLE_TIME = 0.01;
        const double ROBOT_SYSTEM_TICK = 0.01;
        const double ROBOT_MAPPING_TICK = 0.333;
        double TIME_SCALE = 2.5;

        private List<Marker> markers = new List<Marker>();

        private MappingSystem mappingSystem;
        private MappingSystemVisualization mapSysViz;

        private void CanvasMouseDown(object sender, MouseButtonEventArgs e)
        {
            graphics.MouseDown(sender, e);
        }

        private void CanvasMouseUp(object sender, MouseButtonEventArgs e)
        {
            graphics.MouseUp(sender, e);
        }

        private void CanvasMouseRightDown(object sender, MouseButtonEventArgs e)
        {
            //Add a point to robot Trajectory
            target = graphics.MousePointToWorld(Mouse.GetPosition(graphics.GetCanvas()));
            Trace.WriteLine("Clicked");
            Trace.WriteLine(target.x + ", " + target.y);
            //pathFollower.path.AddPoint(point);
        }

        private void CanvasMouseLeave(object sender, MouseEventArgs e)
        {
            graphics.MouseUp(sender, null);
        }

        private void CanvasMouseWheel(object sender, MouseWheelEventArgs e)
        {
            graphics.MouseWheel(sender, e);
        }

        public MainWindow()
        {
            InitializeComponent();

            DispatcherTimer physicsTimer = new DispatcherTimer();
            physicsTimer.Tick += new EventHandler(SimulationStep);
            physicsTimer.Interval = TimeSpan.FromMilliseconds(PHYSICS_SAMPLE_TIME * 1000);
            physicsTimer.Start();

            DispatcherTimer UITimer = new DispatcherTimer();
            UITimer.Tick += new EventHandler(UpdateUI);
            UITimer.Interval = TimeSpan.FromMilliseconds(100);
            UITimer.Start();

            DispatcherTimer robotTick = new DispatcherTimer();
            robotTick.Tick += new EventHandler(RobotTick);
            robotTick.Interval = TimeSpan.FromMilliseconds(ROBOT_SYSTEM_TICK * 1000);
            robotTick.Start();

            DispatcherTimer robotMappingTick = new DispatcherTimer();
            robotMappingTick.Tick += new EventHandler(RobotMappingTick);
            robotMappingTick.Interval = TimeSpan.FromMilliseconds(ROBOT_MAPPING_TICK * 1000);
            robotMappingTick.Start();

            robot = new Robot();
            //Setting wheel parameters
            robot.wheels.left.diameter = 0.50;
            robot.wheels.left.maxSpeed = 4;
            robot.wheels.left.maxAcceleration = 6;
            robot.wheels.left.encoder.stddev = 0.005;
            robot.wheels.right.diameter = 0.50;
            robot.wheels.right.maxSpeed = 4;
            robot.wheels.right.maxAcceleration = 6;
            robot.wheels.right.encoder.stddev = 0.005;
            robot.wheelDistance = 0.667;

            //Attaching sensors to robot
            ProximitySensor s1 = new ProximitySensor();
            robot.proximitySensors.Add(s1);
            ProximitySensor s2 = new ProximitySensor();
            robot.proximitySensors.Add(s2);
            ProximitySensor s3 = new ProximitySensor();
            robot.proximitySensors.Add(s3);
            s1.relativePosition = new Point(0.33, 0, 0);
            s2.relativePosition = new Point(0.16, 0.33, 1);
            s3.relativePosition = new Point(0.16, -0.33, -1);
            s1.range = 6;
            s2.range = 4;
            s3.range = 4;
            double stddev = 1;
            s1.sensor.stddev = stddev;
            s2.sensor.stddev = stddev;
            s3.sensor.stddev = stddev;
            ProximitySensor lidar1 = new ProximitySensor();
            robot.proximitySensors.Add(lidar1);
            lidar1.relativePosition = new Point(0, 0, 0);
            lidar1.sensor.stddev = stddev * 0.5;
            lidar1.range = 15;
            lidar1.rotationSpeed = 0.0333;
            ProximitySensor lidar2 = new ProximitySensor();
            robot.proximitySensors.Add(lidar2);
            lidar2.relativePosition = new Point(0, 0, 0);
            lidar2.sensor.stddev = stddev * 0.5;
            lidar2.range = 15;
            lidar2.rotationSpeed = -0.0333;

            //Creating controller for wheels
            WheelController controllerLeft = new WheelController(robot.wheels.left);
            WheelController controllerRight = new WheelController(robot.wheels.right);
            robot.controllers.left = controllerLeft;
            robot.controllers.right = controllerRight;

            //Creating robot controller instance
            robotController = new RobotController(robot);
            pathFollower = new PathFollower(robotController, new Path());
            pathFinder = new PathFinder();

            //Create graphics abstraction layer
            graphics = new GraphicsLayer(visualization);


            //Adding objects to physics scene
            physics = new List<IPhysicsComponent>();
            physics.Add(robot);
            physics.Add(robot.wheels.left);
            physics.Add(robot.wheels.right);
            //Adding walls
            Point p1 = new Point(1, 1);
            Point p2 = new Point(1, 12);
            Point p3 = new Point(1, 20);
            Point p4 = new Point(10, 20);
            Point p5 = new Point(16, 20);
            Point p6 = new Point(16, 12);
            Point p7 = new Point(20, 12);
            Point p8 = new Point(20, 8);
            Point p9 = new Point(20, 1);
            Point p10 = new Point(6, 1);
            Point p11 = new Point(6, 8);
            Point p12 = new Point(16, 8);
            Point p13 = new Point(18, 8);
            Point p14 = new Point(14, 12);
            Point p15 = new Point(12, 12);
            Point p16 = new Point(10, 12);
            Point p17 = new Point(6, 12);
            Point p18 = new Point(4, 12);
            obstacles.Add(new Obstacle(p1, p3));
            obstacles.Add(new Obstacle(p3, p5));
            obstacles.Add(new Obstacle(p6, p5));
            obstacles.Add(new Obstacle(p7, p14));
            obstacles.Add(new Obstacle(p7, p9));
            obstacles.Add(new Obstacle(p1, p9));
            obstacles.Add(new Obstacle(p10, p11));
            obstacles.Add(new Obstacle(p12, p11));
            obstacles.Add(new Obstacle(p13, p8));
            obstacles.Add(new Obstacle(p2, p18));
            obstacles.Add(new Obstacle(p17, p15));
            obstacles.Add(new Obstacle(p4, p16));
            foreach (Obstacle obstacle in obstacles)
            {
                Marker marker = new ObstacleMarker(obstacle);
                markers.Add(marker);
                marker.AddTo(graphics);
            }


            //Add markers
            path_marker = new PathMarker(pathFollower.path);
            markers.Add(path_marker);
            path_marker.AddTo(graphics);
            Marker path_follower_marker = new PathFollowerMarker(pathFollower);
            markers.Add(path_follower_marker);
            path_follower_marker.AddTo(graphics);
            Marker robot_marker = new RobotMarker(robot);
            markers.Add(robot_marker);
            robot_marker.AddTo(graphics);
            Marker s1_marker = new ProximitySensorMarker(s1);
            Marker s2_marker = new ProximitySensorMarker(s2);
            Marker s3_marker = new ProximitySensorMarker(s3);
            markers.Add(s1_marker);
            markers.Add(s2_marker);
            markers.Add(s3_marker);
            s1_marker.AddTo(graphics);
            s2_marker.AddTo(graphics);
            s3_marker.AddTo(graphics);
            Marker lidar1_marker = new ProximitySensorMarker(lidar1);
            markers.Add(lidar1_marker);
            lidar1_marker.AddTo(graphics);
            Marker lidar2_marker = new ProximitySensorMarker(lidar2);
            markers.Add(lidar2_marker);
            lidar2_marker.AddTo(graphics);
            //Initialize map
            mappingSystem = new MappingSystem(42, 0.5);
            mapSysViz = new MappingSystemVisualization(mappingSystem, MapCanvas);
            mapSysViz.scale = 11;
            mapSysViz.Reset();
            

            //Program loop
            InitUI();
            robot.position.x = 9;
            robot.position.y = 4;

        }


        private void SimulationStep(object sender, EventArgs e)
        {
            foreach (IPhysicsComponent component in physics)
            {
                component.Update(PHYSICS_SAMPLE_TIME * TIME_SCALE);
            }
            //Update visualization
            foreach (Marker marker in markers)
            {
                marker.Update();
            }
            graphics.Update();
        }

        private void UpdateUI(object sender, EventArgs e)
        {
            //Update UI
            speedLabel.Content =
                "Speed:\tL: " +
                string.Format("{0:0.00}", robot.wheels.left.GetSpeed()) +
                ",\t R: " +
                string.Format("{0:0.00}", robot.wheels.right.GetSpeed());

            effortLabel.Content = 
                "Effort:\tL: " +
                string.Format("{0:0.00}", robot.wheels.left.GetEffort()) +
                ",\t R: " +
                string.Format("{0:0.00}", robot.wheels.right.GetEffort());

            setPointLabel.Content =
                "Setpoint:\tL: " +
                string.Format("{0:0.00}", robot.controllers.left.stpt) +
                ",\t R: " +
                string.Format("{0:0.00}", robot.controllers.right.stpt);

            double rectSize = 87;
            leftSpeedRect.Height = rectSize * Math.Abs(robot.wheels.left.GetSpeed()) / robot.wheels.left.maxSpeed;
            rightSpeedRect.Height = rectSize * Math.Abs(robot.wheels.right.GetSpeed()) / robot.wheels.right.maxSpeed;
            leftAccelerationRect.Height = rectSize * Math.Abs(robot.wheels.left.GetAcceleration()) / robot.wheels.left.maxAcceleration;
            rightAccelerationRect.Height = rectSize * Math.Abs(robot.wheels.right.GetAcceleration()) / robot.wheels.right.maxAcceleration;
            leftSTPTRect.Height = rectSize * Math.Abs(robot.controllers.left.stpt) / robot.wheels.left.maxSpeed;
            rightSTPTRect.Height = rectSize * Math.Abs(robot.controllers.right.stpt) / robot.wheels.right.maxSpeed;

            //Sliders set the velocities
            //robotController.SetCommand(linearSpeedSlider.Value, angularSpeedSlider.Value);
            TIME_SCALE = TimeScaleSlider.Value;
            TimeScaleLabel.Content = "Simulation Time Scale: x" + string.Format("{0:0.00}", TimeScaleSlider.Value);

            mapSysViz.Update();
        }

        private void RobotTick(object sender, EventArgs e)
        {
            robot.Tick(ROBOT_SYSTEM_TICK * TIME_SCALE);
            foreach(ProximitySensor sensor in robot.proximitySensors)
            {
                double distance = sensor.Measure(obstacles);
                if (sensor.isHit)
                {
                    mappingSystem.ConsiderReading(sensor.position, sensor.hitPoint, distance, sensor.sensor.stddevFunc);
                }
            }
            pathFollower.Step(ROBOT_SYSTEM_TICK * TIME_SCALE);
        }

        private void RobotMappingTick(object sender, EventArgs e)
        {
            pathFollower.path = pathFinder.OptimizePath(
                pathFinder.FindPath(robot.position, target, mappingSystem),
                obstacles,
                3);
            ((PathMarker)path_marker).SetPath(pathFollower.path);
        }

        private void InitUI()
        {
            //linearSpeedSlider.Minimum = -robotController.maxLinearSpeed;
            //linearSpeedSlider.Maximum = robotController.maxLinearSpeed;
            //angularSpeedSlider.Minimum = -robotController.maxAngularSpeed;
            //angularSpeedSlider.Maximum = robotController.maxAngularSpeed;
            //linearSpeedSlider.Value = 0;
            //angularSpeedSlider.Value = 0;
        }

        private void Map_Reset_Button_Click(object sender, RoutedEventArgs e)
        {
            mappingSystem.Reset();
            mapSysViz.Reset();
        }
    }
}
