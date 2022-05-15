using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Controls;
using System.Windows.Media;
using System.Windows.Shapes;

namespace RobotSimulation
{
    class MappingSystemVisualization
    {
        Canvas canvas;
        MappingSystem map;
        public double scale = 20;
        List<List<Rectangle>> rectangles;

        public MappingSystemVisualization(MappingSystem map, Canvas canvas)
        {
            this.map = map;
            this.canvas = canvas;
        }

        public void Update()
        {
            for (int i = 0; i < map.nTiles; i++)
            {
                for (int j = 0; j < map.nTiles; j++)
                {
                    rectangles[i][j].Fill = new SolidColorBrush(Color.FromRgb(
                        Convert.ToByte(255),
                        Convert.ToByte(255 * (1 - map.tileArray[i][j].Bel)),
                        Convert.ToByte(255 * (1 - map.tileArray[i][j].Bel))));
                }
            }
        }

        public void Reset()
        {
            canvas.Children.Clear();

            //Add squares to canvas
            rectangles = new List<List<Rectangle>>();
            for (int i = 0; i < map.nTiles; i++)
            {
                List<Rectangle> temp_list = new List<Rectangle>();
                for (int j = 0; j < map.nTiles; j++)
                {
                    Rectangle rectangle = new Rectangle();
                    rectangle.Fill = Brushes.White;
                    rectangle.Stroke = Brushes.White;
                    rectangle.StrokeThickness = 0.5;
                    rectangle.Width = map.tileSize * scale;
                    rectangle.Height = rectangle.Width;
                    Canvas.SetLeft(rectangle, j * map.tileSize * scale);
                    Canvas.SetTop(rectangle, i * map.tileSize * scale);
                    temp_list.Add(rectangle);
                    canvas.Children.Add(rectangle);
                }
                rectangles.Add(temp_list);
            }
        }
    }
}
