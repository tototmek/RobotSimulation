using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace RobotSimulation
{
    class MappingSystem
    {
        public List<List<MapTile>> tileArray;
        public double nTiles;
        public double tileSize;
        private double sampleCoeff = 0.05;

        public MappingSystem(double nTiles, double tileSize)
        {
            this.nTiles = nTiles;
            this.tileSize = tileSize;
            Reset();
        }

        public void ConsiderReading(Point rayOrigin, Point hitPoint, double distance, Func<double, double> stddevFunc)
        {
            
            //Find squares intersectiong with line
            double p1_x = rayOrigin.x;
            double p1_y = rayOrigin.y;
            double p2_x = hitPoint.x;
            double p2_y = hitPoint.y;
            double m = (p1_y - p2_y) / (p1_x - p2_x);
            double im = 1 / m;
            double n = p1_y - m * p2_x;
            double x0;
            double x1;
            double y0;
            double y1;
            double dx1;
            double dx2;
            Func<double, double> y = x_ => {
                return m * x_ + n;
            };
            Func<double, double> x = y_ => {
                return (y_ - n) * im;
            };
            for (int i = 0; i < nTiles; i++)
            {
                for (int j = 0; j < nTiles; j++)
                {
                    x0 = (j + 0.25) * tileSize;
                    x1 = (j + 0.75) * tileSize;
                    y0 = (i + 0.25) * tileSize;
                    y1 = (i + 0.75) * tileSize;
                    dx1 = ((j + 0.5) * tileSize) - rayOrigin.x;
                    dx2 = hitPoint.x - rayOrigin.x;
                    if ((((y(x0) < y1 && y(x0) > y0) || (y(x1) < y1 && y(x1) > y0) || (x(y0) > x0 && x(y0) < x1) || (x(y1) > x0 && x(y1) < x1))) && dx1 * dx2 > 0)
                    {
                        double dist = Math.Sqrt(Math.Pow(rayOrigin.x - ((j + 0.5) * tileSize), 2) + Math.Pow(rayOrigin.y - ((i + 0.5) * tileSize), 2));
                        double bell = tileArray[i][j].Bel;
                        tileArray[i][j] = new MapTile(bell - bell * 0.33 * sampleCoeff / (1 - stddevFunc(dist)));
                        //tileArray[i][j] = new MapTile(1);
                    }
                }
            }

            //Set bel from new squares
            int hitTileX = (int)Math.Floor(hitPoint.x / tileSize);
            int hitTileY = (int)Math.Floor(hitPoint.y / tileSize);
            if (!(hitTileX < nTiles && hitTileY < nTiles && hitTileX >= 0 && hitTileY >= 0)) return;
            double bel = tileArray[hitTileY][hitTileX].Bel;
            tileArray[hitTileY][hitTileX] = new MapTile(bel + (1 - bel) * sampleCoeff / (1 - stddevFunc(distance)));

        }

        public void Reset()
        {
            tileArray = new List<List<MapTile>>(); 
            for (int i = 0; i < nTiles; i++)
            {
                List<MapTile> temp_list = new List<MapTile>();
                for (int j = 0; j < nTiles; j++)
                {
                    temp_list.Add(new MapTile());
                }
                tileArray.Add(temp_list);
            }
        }
    }

    struct MapTile
    {
        public double Bel;
        public MapTile(double Bel_ = 0)
        {
            Bel = Bel_;
        }
    }
}
