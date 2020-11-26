using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Forms;

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
            GridMap gridMap = new GridMap("./map.txt");
            Painter.PainterInit(pcb_display, gridMap);            

            Painter.PaintMap(gridMap);

            this.pcb_display.MouseClick += new MouseEventHandler(Painter.pcb_MouseClick);
            this.pcb_display.MouseMove += new MouseEventHandler(Painter.pcb_MouseMove);

        }

    }
}
