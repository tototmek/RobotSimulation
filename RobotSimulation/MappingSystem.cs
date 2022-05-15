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
                        tileArray[i][j].Bel = bell - bell * 0.33 * sampleCoeff / (1 - stddevFunc(dist));
                        //tileArray[i][j] = new MapTile(1);
                    }
                }
            }

            //Set bel from new squares
            int hitTileX = (int)Math.Floor(hitPoint.x / tileSize);
            int hitTileY = (int)Math.Floor(hitPoint.y / tileSize);
            if (!(hitTileX < nTiles && hitTileY < nTiles && hitTileX >= 0 && hitTileY >= 0)) return;
            double bel = tileArray[hitTileY][hitTileX].Bel;
            tileArray[hitTileY][hitTileX].Bel = bel + (1 - bel) * sampleCoeff / (1 - stddevFunc(distance));
        }

        public MapTile TileFromWorldPoint(double x, double y)
        {
            return tileArray[(int)Math.Floor(y / tileSize)][(int)Math.Floor(x / tileSize)];
        }
        public Point TileToWorldPoint(MapTile tile)
        {
            return new Point(
                    (tile.gridY + 0.5) * tileSize,
                    (tile.gridX + 0.5) * tileSize);
        }

        public void Reset()
        {
            tileArray = new List<List<MapTile>>(); 
            for (int i = 0; i < nTiles; i++)
            {
                List<MapTile> temp_list = new List<MapTile>();
                for (int j = 0; j < nTiles; j++)
                {
                    temp_list.Add(new MapTile(i, j));
                }
                tileArray.Add(temp_list);
            }
        }
    }

    class MapTile
    {
        public double Bel;
        public int gridX;
        public int gridY;
        public int gCost;
        public int hCost;
        public MapTile parent;

        public MapTile(int gridX, int gridY, double Bel = 0)
        {
            this.Bel = Bel;
            this.gridX = gridX;
            this.gridY = gridY;
        }

        public int fCost
        {
            get
            {
                return gCost + hCost;
            }
        }

        public bool walkable
        {
            get
            {
                return Bel <= 0.05;
            }
        }
    }

    class PathFinder
    {
        public Path FindPath(Point startPos, Point targetPos, MappingSystem map)
        {
            
            MapTile startNode = map.TileFromWorldPoint(startPos.x, startPos.y);
            MapTile targetNode = map.TileFromWorldPoint(targetPos.x, targetPos.y);

            List<MapTile> openSet = new List<MapTile>();
            HashSet<MapTile> closedSet = new HashSet<MapTile>();
            openSet.Add(startNode);

            while (openSet.Count > 0)
            {
                MapTile node = openSet[0];
                for (int i = 1; i < openSet.Count; i++)
                {
                    if (openSet[i].fCost < node.fCost || openSet[i].fCost == node.fCost)
                    {
                        if (openSet[i].hCost < node.hCost)
                            node = openSet[i];
                    }
                }

                openSet.Remove(node);
                closedSet.Add(node);

                if (node == targetNode)
                {
                    return RetracePath(startNode, targetNode, map);
                }

                foreach (MapTile neighbour in GetNeighbours(node, map))
                {
                    if (!neighbour.walkable || closedSet.Contains(neighbour))
                    {
                        continue;
                    }

                    int newCostToNeighbour = node.gCost + GetDistance(node, neighbour);
                    if (newCostToNeighbour < neighbour.gCost || !openSet.Contains(neighbour))
                    {
                        neighbour.gCost = newCostToNeighbour;
                        neighbour.hCost = GetDistance(neighbour, targetNode);
                        neighbour.parent = node;

                        if (!openSet.Contains(neighbour))
                            openSet.Add(neighbour);
                    }
                }
            }
            Trace.WriteLine("Path not found!");
            return new Path();
        }

        public List<MapTile> GetNeighbours(MapTile tile, MappingSystem map)
        {
            List<MapTile> neighbours = new List<MapTile>();

            for (int x = -1; x <= 1; x++)
            {
                for (int y = -1; y <= 1; y++)
                {
                    if (x == 0 && y == 0)
                        continue;

                    int checkX = tile.gridX + x;
                    int checkY = tile.gridY + y;

                    if (checkX >= 0 && checkX < map.nTiles && checkY >= 0 && checkY < map.nTiles)
                    {
                        neighbours.Add(map.tileArray[checkX][checkY]);
                    }
                }
            }

            return neighbours;
        }

        private Path RetracePath(MapTile start, MapTile end, MappingSystem map)
        {
            Path path = new Path();
            MapTile currentNode = end;

            while (currentNode != start)
            {
                path.AddPoint(map.TileToWorldPoint(currentNode));
                currentNode = currentNode.parent;
            }
            path.Reverse();
            return path;
        }

        int GetDistance(MapTile nodeA, MapTile nodeB)
        {
            int dstX = Math.Abs(nodeA.gridX - nodeB.gridX);
            int dstY = Math.Abs(nodeA.gridY - nodeB.gridY);

            if (dstX > dstY)
                return 14 * dstY + 10 * (dstX - dstY);
            return 14 * dstX + 10 * (dstY - dstX);
        }
    }
}
