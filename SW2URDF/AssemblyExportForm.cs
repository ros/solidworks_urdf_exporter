using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Windows.Forms;

using SolidWorks.Interop.sldworks;

namespace SW2URDF
{
    public partial class AssemblyExportForm : Form
    {
        public AssyExporter mAssyExporter;
        public AssemblyExportForm(ISldWorks iSwApp)
        {
            InitializeComponent();
            //mAssyExporter = new AssyExporter(iSwApp);
        }

        //Joint form configuration controls
        private void AssemblyExportForm_Load(object sender, EventArgs e)
        {
            //for (int i = 0; i < mAssyExporter.mLinks.Count; i++)
            //{
            //    checkedListBox1.Items.Add(mAssyExporter.mLinks[i].name);
            //}
        }

        private void button_link_next_Click(object sender, EventArgs e)
        {
            panel_links.Visible = true;
        }

        private void button_link_cancel_Click(object sender, EventArgs e)
        {
            this.Close();
        }

        private void checkedListBox1_SelectedIndexChanged(object sender, EventArgs e)
        {

        }

        private void groupBox1_Enter(object sender, EventArgs e)
        {

        }

        //Joint form configuration controls

        private void button_joint_cancel_Click(object sender, EventArgs e)
        {
            this.Close();
        }

        private void button_joint_previous_Click(object sender, EventArgs e)
        {
            panel_joint.Visible = false;
        }

        private void button_joint_next_Click(object sender, EventArgs e)
        {
            panel_mesh.Visible = true;
        }

        //Mesh form configuration controls
        private void button_mesh_finish_Click(object sender, EventArgs e)
        {

            this.Close();
        }

        private void button_mesh_previous_Click(object sender, EventArgs e)
        {
            panel_mesh.Visible = false;
        }

        private void button_mesh_cancel_Click(object sender, EventArgs e)
        {
            this.Close();
        }

        private void trackBar_mesh_collision_Scroll(object sender, EventArgs e)
        {

        }

        private void trackBar_mesh_visual_Scroll(object sender, EventArgs e)
        {

        }

        private void button_select_Click(object sender, EventArgs e)
        {
            for (int i = 0; i < checkedListBox1.Items.Count; i++)
            {
                checkedListBox1.SetItemChecked(i, true);
            }
        }

        private void button_deselect_Click(object sender, EventArgs e)
        {
            for (int i = 0; i < checkedListBox1.Items.Count; i++)
            {
                checkedListBox1.SetItemChecked(i, false);
            }
        }

        private void button_links_previous_Click(object sender, EventArgs e)
        {
            panel_links.Visible = false;
        }

        private void button_links_next_Click(object sender, EventArgs e)
        {
            panel_joint.Visible = true;
        }

        private void button_links_cancel_Click(object sender, EventArgs e)
        {
            this.Close();
        }







    }
}
