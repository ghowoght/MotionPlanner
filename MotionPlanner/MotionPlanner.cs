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
            Painter.Paint(pcb_display);
            this.pcb_display.MouseClick += new MouseEventHandler(pcb_MouseClick);
            this.pcb_display.MouseMove += new MouseEventHandler(pcb_MouseMove);
        }
        public void pcb_MouseMove(object sender, MouseEventArgs e)
        {
            int ex = e.X;
            int ey = e.Y;

            Painter.pcb_MouseMove(pcb_display, e);

        }

        public void pcb_MouseClick(object sender, MouseEventArgs e)
        {
            int ex = e.X;
            int ey = e.Y;
            Painter.pcb_MouseClick(pcb_display, e);
        }

    }
}
