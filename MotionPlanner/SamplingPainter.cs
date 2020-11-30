using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Forms;

namespace MotionPlanner
{
    class SamplingPainter
    {
        private PictureBox pcb;
        private GridMap gridMap;

        public SamplingPainter(PictureBox pcb, GridMap map)
        {
            this.pcb = pcb;
            this.gridMap = map;
        }
    }
}
