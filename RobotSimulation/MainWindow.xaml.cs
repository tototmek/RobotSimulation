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
        private List<IPhysicsComponent> physics;
        private GraphicsLayer graphics;
        private List<Obstacle> obstacles = new List<Obstacle>();

        const double PHYSICS_SAMPLE_TIME = 0.01;
        const double ROBOT_SYSTEM_TICK = 0.01;

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
            Point point = graphics.MousePointToWorld(Mouse.GetPosition(graphics.GetCanvas()));
            Trace.WriteLine("Clicked");
            Trace.WriteLine(point.x + ", " + point.y);
            pathFollower.path.AddPoint(point);
            Trace.WriteLine(pathFollower.path.GetPoints().Count);
            Trace.WriteLine(((PathMarker)markers[12]).path.GetPoints().Count);
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

            robot = new Robot();
            //Setting wheel parameters
            robot.wheels.left.diameter = 0.6667;
            robot.wheels.left.maxSpeed = 2;
            robot.wheels.left.maxAcceleration = 4;
            robot.wheels.left.encoder.stddev = 0.005;
            robot.wheels.right.diameter = 0.6667;
            robot.wheels.right.maxSpeed = 2;
            robot.wheels.right.maxAcceleration = 4;
            robot.wheels.right.encoder.stddev = 0.005;
            robot.wheelDistance = 1;

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
            lidar1.rotationSpeed = 0.033;

            //Creating controller for wheels
            WheelController controllerLeft = new WheelController(robot.wheels.left);
            WheelController controllerRight = new WheelController(robot.wheels.right);
            robot.controllers.left = controllerLeft;
            robot.controllers.right = controllerRight;

            //Creating robot controller instance
            robotController = new RobotController(robot);
            pathFollower = new PathFollower(robotController, new Path());

            //Create graphics abstraction layer
            graphics = new GraphicsLayer(visualization);


            //Adding objects to physics scene
            physics = new List<IPhysicsComponent>();
            physics.Add(robot);
            physics.Add(robot.wheels.left);
            physics.Add(robot.wheels.right);
            //Adding walls
            Point p1 = new Point(1, 1);
            Point p2 = new Point(15, 1);
            Point p3 = new Point(15, 10);
            Point p7 = new Point(13, 5);
            Point p8 = new Point(15, 5);
            Point p4 = new Point(7, 10);
            Point p5 = new Point(7, 6);
            Point p6 = new Point(1, 6);
            Obstacle ob1 = new Obstacle(p1, p2);
            Obstacle ob2 = new Obstacle(p2, p3);
            Obstacle ob3 = new Obstacle(p3, p4);
            Obstacle ob4 = new Obstacle(p4, p5);
            Obstacle ob5 = new Obstacle(p5, p6);
            Obstacle ob6 = new Obstacle(p6, p1);
            Obstacle ob7 = new Obstacle(p7, p8);
            obstacles.Add(ob1);
            obstacles.Add(ob2);
            obstacles.Add(ob3);
            obstacles.Add(ob4);
            obstacles.Add(ob5);
            obstacles.Add(ob6);
            obstacles.Add(ob7);
            foreach (Obstacle obstacle in obstacles)
            {
                Marker marker = new ObstacleMarker(obstacle);
                markers.Add(marker);
                marker.AddTo(graphics);
            }


            //Add markers
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
            Marker path_marker = new PathMarker(pathFollower.path);
            markers.Add(path_marker);
            path_marker.AddTo(graphics);
            Marker path_follower_marker = new PathFollowerMarker(pathFollower);
            markers.Add(path_follower_marker);
            path_follower_marker.AddTo(graphics);

            //Initialize map
            mappingSystem = new MappingSystem(32, 0.5);
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
                component.Update(PHYSICS_SAMPLE_TIME);
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
            

            mapSysViz.Update();
        }

        private void RobotTick(object sender, EventArgs e)
        {
            robot.Tick(ROBOT_SYSTEM_TICK);
            foreach(ProximitySensor sensor in robot.proximitySensors)
            {
                double distance = sensor.Measure(obstacles);
                if (sensor.isHit)
                {
                    mappingSystem.ConsiderReading(sensor.position, sensor.hitPoint, distance, sensor.sensor.stddevFunc);
                }
            }
            pathFollower.Step(ROBOT_SYSTEM_TICK);
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
