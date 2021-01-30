
namespace MotionPlanner
{
    partial class MotionPlanner
    {
        /// <summary>
        /// 必需的设计器变量。
        /// </summary>
        private System.ComponentModel.IContainer components = null;

        /// <summary>
        /// 清理所有正在使用的资源。
        /// </summary>
        /// <param name="disposing">如果应释放托管资源，为 true；否则为 false。</param>
        protected override void Dispose(bool disposing)
        {
            if (disposing && (components != null))
            {
                components.Dispose();
            }
            base.Dispose(disposing);
        }

        #region Windows 窗体设计器生成的代码

        /// <summary>
        /// 设计器支持所需的方法 - 不要修改
        /// 使用代码编辑器修改此方法的内容。
        /// </summary>
        private void InitializeComponent()
        {
            this.pcb_display = new System.Windows.Forms.PictureBox();
            ((System.ComponentModel.ISupportInitialize)(this.pcb_display)).BeginInit();
            this.SuspendLayout();
            // 
            // pcb_display
            // 
            this.pcb_display.Location = new System.Drawing.Point(4, 4);
            this.pcb_display.Margin = new System.Windows.Forms.Padding(4, 5, 4, 5);
            this.pcb_display.Name = "pcb_display";
            this.pcb_display.Size = new System.Drawing.Size(1600, 800);
            this.pcb_display.TabIndex = 0;
            this.pcb_display.TabStop = false;
            // 
            // MotionPlanner
            // 
            this.AutoScaleDimensions = new System.Drawing.SizeF(12F, 24F);
            this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
            this.AutoSize = true;
            this.AutoSizeMode = System.Windows.Forms.AutoSizeMode.GrowAndShrink;
            this.ClientSize = new System.Drawing.Size(1610, 983);
            this.Controls.Add(this.pcb_display);
            this.Margin = new System.Windows.Forms.Padding(4, 5, 4, 5);
            this.Name = "MotionPlanner";
            this.Text = "MotionPlanner";
            this.Load += new System.EventHandler(this.MotionPlanner_Load);
            ((System.ComponentModel.ISupportInitialize)(this.pcb_display)).EndInit();
            this.ResumeLayout(false);

        }

        #endregion

        private System.Windows.Forms.PictureBox pcb_display;
    }
}

