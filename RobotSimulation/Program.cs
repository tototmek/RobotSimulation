using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows;

namespace RobotSimulation
{
    static class Simulation
    {
        public static void Start(object window_)
        {
            MainWindow window = (MainWindow)window_;

            //Setting simulation parameters
            const double PHYSICS_SAMPLE_TIME = 0.02;

            Robot robot = new Robot();
            //Setting wheel parameters
            robot.wheels.left.diameter = 0.4;
            robot.wheels.left.maxSpeed = 1;
            robot.wheels.left.maxAcceleration = 0.3;
            robot.wheels.right.diameter = 0.4;
            robot.wheels.right.maxSpeed = 1;
            robot.wheels.right.maxAcceleration = 0.3;
            robot.wheelDistance = 0.5;

            //Adding objects to physics scene
            List<IPhysicsComponent> physics = new List<IPhysicsComponent>();
            physics.Add(robot);
            physics.Add(robot.wheels.left);
            physics.Add(robot.wheels.right);

            //Creating controller for wheels
            WheelController controllerLeft = new WheelController(robot.wheels.left);
            WheelController controllerRight = new WheelController(robot.wheels.right);

            //Program loop
            controllerLeft.SetWheelCommand(0.1);
            controllerRight.SetWheelCommand(0.8);
            controllerLeft.Step(0.4);
            controllerRight.Step(0.4);
            while (true)
            {
                foreach (IPhysicsComponent component in physics)
                {
                    component.Update(PHYSICS_SAMPLE_TIME);
                }
                System.Threading.Thread.Sleep((int)(PHYSICS_SAMPLE_TIME * 1000));

                //Manage visualisation
                //window.robot.Height = robot.position.x;
            }

        }
    }
}
