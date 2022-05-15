using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace RobotSimulation
{
    interface IOnboardComponent
    {
        void Step(double sampleTime);
    }
    class WheelController : IOnboardComponent
    {
        public Wheel wheel;
        public double stpt;
        public PID pid;

        public WheelController(Wheel wheel_)
        {
            wheel = wheel_;
            pid = new PID();
            pid.K = 6;
            pid.Ti = 15;
            pid.Td = 0.1;
        }

        public void SetWheelCommand(double value)
        {
            stpt = value;
        }

        public void Step(double sampleTime)
        {
            wheel.SetEffort(pid.MV(wheel.GetSpeed(), stpt, sampleTime));
        }
    }
}
