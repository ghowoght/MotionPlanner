using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Forms;
using System.Threading;

namespace MotionPlanner
{
    public partial class MotionPlanner : Form
    {
        public MotionPlanner()
        {
            InitializeComponent();
        }

        private void MotionPlanner_Load(object sender, EventArgs e)
        {
            //GridMap gridMap = new GridMap("../../map.txt");
            GridMap gridMap = new GridMap(20, 40, "../../map.txt");
            Painter.PainterInit(pcb_display, gridMap);

            this.pcb_display.MouseClick += new MouseEventHandler(Painter.pcb_MouseClick);
            this.pcb_display.MouseMove += new MouseEventHandler(Painter.pcb_MouseMove);


            gridMap.Reset("../../blank_map.txt");
            //BFS bfs = new BFS(gridMap);
            //new Thread(bfs.Search)
            //{
            //    IsBackground = true
            //}.Start();

            /*GBFS gbfs = new GBFS(gridMap);
            new Thread(gbfs.Search)
            {
                IsBackground = true
            }.Start();*/

            /*DFS dfs = new DFS(gridMap);
            new Thread(dfs.Search)
            {
                IsBackground = true
            }.Start();*/


            //Dijkstra dijkstra = new Dijkstra(gridMap);
            //new Thread(dijkstra.Search)
            //{
            //    IsBackground = true
            //}.Start();

            //Thread.Sleep(2000);
            //gridMap.Reset("../../map_01.txt");

            Astar astar = new Astar(gridMap);
            new Thread(astar.Search)
            {
                IsBackground = true
            }.Start();


            //JPS jps = new JPS(gridMap);
            //new Thread(jps.Search)
            //{
            //    IsBackground = true
            //}.Start();

        }

        //private void btn_genMap_Click(object sender, EventArgs e)
        //{
        //    MapGenerator mapGen = new MapGenerator();
        //    mapGen.Show();
        //}
    }
}
