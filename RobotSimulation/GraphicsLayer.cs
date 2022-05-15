using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Controls;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Shapes;

namespace RobotSimulation
{
    abstract class Drawable {

        protected Shape avatar;

        public virtual void Add(Canvas canvas)
        {
            canvas.Children.Add(avatar);
        }

        public abstract void Update(View view);

        public virtual void Remove(Canvas canvas)
        {
            canvas.Children.Remove(avatar);
        }

        public virtual void SetColor(SolidColorBrush color)
        {
            avatar.Stroke = color;
        }

        public virtual void SetStrokeThickness(double thickness)
        {
            avatar.StrokeThickness = thickness;
        }

        public virtual void SetDashLine(double interval)
        {
            avatar.StrokeDashArray = new DoubleCollection() { interval };
        }
    }

    struct View
    {
        public Point pan;
        public double zoom;

        public View(double zoom_ = 1, Point pan_ = new Point()) {
            pan = pan_;
            zoom = zoom_;
        }
    }

    class GraphicsLayer
    {
        public static double WORLD_TO_VIS = 50;
        private List<Drawable> objects = new List<Drawable>();
        private Canvas canvas;
        private View view;
        private Point mousePosition;
        private Point deltaMouse;
        private bool mouseDrag = false;

        public GraphicsLayer(Canvas canvas)
        {
            this.canvas = canvas;
            view = new View();
            view.zoom = 1;
            view.pan = new Point();
            view.pan.x = 0;
            view.pan.y = 0;
        }

        public Canvas GetCanvas()
        {
            return canvas;
        }

        public void Add<T>(T obj) where T : Drawable
        {
            objects.Add(obj);
            obj.Add(canvas);
            obj.Update(view);
        }

        public void Remove<T>(T obj) where T : Drawable
        {
            objects.Remove(obj);
            obj.Remove(canvas);
        }

        public void Update()
        {
            //handle mouse
            if (mouseDrag)
            {
                System.Windows.Point newMousePosition = Mouse.GetPosition(canvas);
                deltaMouse.x = (newMousePosition.X - mousePosition.x) / (WORLD_TO_VIS * view.zoom );
                deltaMouse.y = (newMousePosition.Y - mousePosition.y) / (WORLD_TO_VIS * view.zoom );
                mousePosition.x = newMousePosition.X;
                mousePosition.y = newMousePosition.Y;
                view.pan.x += deltaMouse.x;
                view.pan.y += deltaMouse.y;
            }
            //update objects
            foreach (Drawable obj in objects) {
                obj.Update(view);
            }
        }

        public void MouseDown(object sender, MouseButtonEventArgs e)
        {
            mouseDrag = true;
            System.Windows.Point newMousePosition = Mouse.GetPosition(canvas);
            mousePosition.x = newMousePosition.X;
            mousePosition.y = newMousePosition.Y;
        }

        public void MouseUp(object sender, MouseButtonEventArgs e)
        {
            mouseDrag = false;
        }

        public void MouseWheel(object sender, MouseWheelEventArgs e)
        {
            if (e.Delta > 0)
            {
                view.zoom *= 1.1;
                System.Windows.Point newMousePosition = Mouse.GetPosition(canvas);
                view.pan.x -= newMousePosition.X / view.zoom / WORLD_TO_VIS * 0.066;
                view.pan.y -= newMousePosition.Y / view.zoom / WORLD_TO_VIS * 0.066;
            }
            else if (e.Delta < 0)
            {
                view.zoom *= 0.92;
                System.Windows.Point newMousePosition = Mouse.GetPosition(canvas);
                view.pan.x += newMousePosition.X / view.zoom / WORLD_TO_VIS * 0.066;
                view.pan.y += newMousePosition.Y / view.zoom / WORLD_TO_VIS * 0.066;
            }
        }

        public Point MousePointToWorld(System.Windows.Point mousePoint)
        {
            Point point = new Point();
            point.x = -view.pan.x + mousePoint.X / (WORLD_TO_VIS * view.zoom);
            point.y = -view.pan.y + mousePoint.Y / (WORLD_TO_VIS * view.zoom);
            return point;
        }
    }

    class dCircle : Drawable
    {
        private Point position;
        private double radius;
        private View view;

        public dCircle()
        {
            avatar = new Ellipse();
            avatar.Fill = new SolidColorBrush(Colors.White);
            avatar.Stroke = Brushes.LightGray;
            avatar.StrokeThickness = 2;
        }
        public void SetRadius(double value)
        {
            radius = value;
            avatar.Width = radius * view.zoom * 2 * GraphicsLayer.WORLD_TO_VIS;
            avatar.Height = radius * view.zoom * 2 * GraphicsLayer.WORLD_TO_VIS;
        }

        public void SetPosition(double x, double y)
        {
            position.x = x;
            position.y = y;
            Canvas.SetLeft(avatar, (position.x + view.pan.x - radius) * view.zoom * GraphicsLayer.WORLD_TO_VIS);
            Canvas.SetTop(avatar, (position.y + view.pan.y - radius) * view.zoom * GraphicsLayer.WORLD_TO_VIS);
        }

        public override void Update(View view)
        {
            this.view = view;
            SetRadius(radius);
            SetPosition(position.x, position.y);
        }
    }

    class dLine : Drawable
    {
        private Point p1 = new Point();
        private Point p2 = new Point();
        private View view;

        public dLine()
        {
            avatar = new Line();
            avatar.Stroke = Brushes.Red;
            avatar.StrokeThickness = 2;
        }

        public void SetP1(double x, double y)
        {
            p1.x = x;
            p1.y = y;
            ((Line)avatar).X1 = (p1.x + view.pan.x) * GraphicsLayer.WORLD_TO_VIS * view.zoom;
            ((Line)avatar).Y1 = (p1.y + view.pan.y) * GraphicsLayer.WORLD_TO_VIS * view.zoom;
        }

        public void SetP2(double x, double y)
        {
            p2.x = x;
            p2.y = y;
            ((Line)avatar).X2 = (p2.x + view.pan.x) * GraphicsLayer.WORLD_TO_VIS * view.zoom;
            ((Line)avatar).Y2 = (p2.y + view.pan.y) * GraphicsLayer.WORLD_TO_VIS * view.zoom;
        }

        public override void Update(View view)
        {
            this.view = view;
            SetP1(p1.x, p1.y);
            SetP2(p2.x, p2.y);
        }

    }

    class dRect : Drawable
    {
        private Point position = new Point();
        private double width;
        private double height;
        private View view;

        public dRect()
        {
            avatar = new Rectangle();
            avatar.LayoutTransform = new RotateTransform(position.orientation, 0, 0);
            avatar.Fill = new SolidColorBrush(Colors.White);
            avatar.Stroke = Brushes.LightGray;
            avatar.StrokeThickness = 2;
            SetSize(3, 2);
        }

        public void SetSize(double width, double height)
        {
            this.width = width;
            this.height = height;
        }

        public void SetPosition(Point position)
        {
            this.position = position;
        }

        public override void Update(View view)
        {
            this.view = view;
            avatar.Width = width * view.zoom * GraphicsLayer.WORLD_TO_VIS;
            avatar.Height = height * view.zoom * GraphicsLayer.WORLD_TO_VIS;
            ((RotateTransform)avatar.LayoutTransform).Angle = position.orientation * 57.2957795;
            var bounds = avatar.LayoutTransform.TransformBounds(new System.Windows.Rect(new System.Windows.Size(avatar.Width, avatar.Height)));
            double dx = bounds.Width * 0.5;
            double dy = bounds.Height * 0.5;
            Canvas.SetLeft(avatar, (position.x + view.pan.x) * view.zoom * GraphicsLayer.WORLD_TO_VIS - dx);
            Canvas.SetTop(avatar, (position.y + view.pan.y) * view.zoom * GraphicsLayer.WORLD_TO_VIS - dy);

        }
    }

    class dPoint : Drawable
    {
        private Point position = new Point();
        private Line avatar1;
        private Line avatar2;
        private View view;
        private double SIZE = 3;

        public dPoint()
        {
            avatar1 = new Line();
            avatar1.Stroke = Brushes.DarkBlue;
            avatar1.StrokeThickness = 0.5;
            avatar2 = new Line();
            avatar2.Stroke = Brushes.DarkBlue;
            avatar2.StrokeThickness = 0.5;
        }

        public void SetPosition(double x, double y)
        {
            position.x = x;
            position.y = y;
            double screen_x = (position.x + view.pan.x) * GraphicsLayer.WORLD_TO_VIS * view.zoom;
            double screen_y = (position.y + view.pan.y) * GraphicsLayer.WORLD_TO_VIS * view.zoom;
            
            avatar1.X1 = screen_x - SIZE * view.zoom;
            avatar1.X2 = screen_x + SIZE * view.zoom;
            avatar1.Y1 = screen_y - SIZE * view.zoom;
            avatar1.Y2 = screen_y + SIZE * view.zoom;
            avatar2.X1 = screen_x + SIZE * view.zoom;
            avatar2.X2 = screen_x - SIZE * view.zoom;
            avatar2.Y1 = screen_y - SIZE * view.zoom;
            avatar2.Y2 = screen_y + SIZE * view.zoom;
        }

        public override void Add(Canvas canvas)
        {
            canvas.Children.Add(avatar1);
            canvas.Children.Add(avatar2);
        }

        public override void Update(View view)
        {
            this.view = view;
            SetPosition(position.x, position.y);
        }

        public override void Remove(Canvas canvas)
        {
            canvas.Children.Remove(avatar1);
            canvas.Children.Remove(avatar2);
        }
    }
}
