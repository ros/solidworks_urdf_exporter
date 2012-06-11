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
    public partial class PartExportForm : Form
    {
        public PartExporter mPartExporter;
        public PartExportForm(ISldWorks iSwApp)
        {
            InitializeComponent();
            mPartExporter = new PartExporter(iSwApp);
        }

        private void label2_Click(object sender, EventArgs e)
        {

        }

        private void openFileDialog1_FileOk(object sender, CancelEventArgs e)
        {

        }

        private void textBox_inertial_origin_x_TextChanged(object sender, EventArgs e)
        {

        }

        private void textBox_inertial_origin_y_TextChanged(object sender, EventArgs e)
        {

        }

        private void textBox_inertial_origin_z_TextChanged(object sender, EventArgs e)
        {

        }

        private void textBox_inertial_origin_roll_TextChanged(object sender, EventArgs e)
        {

        }

        private void textBox_inertial_origin_pitch_TextChanged(object sender, EventArgs e)
        {

        }

        private void textBox_inertial_origin_yaw_TextChanged(object sender, EventArgs e)
        {

        }

        private void textBox_mass_TextChanged(object sender, EventArgs e)
        {

        }

        private void textBox_ixx_TextChanged(object sender, EventArgs e)
        {

        }

        private void textBox_ixy_TextChanged(object sender, EventArgs e)
        {

        }

        private void textBox_ixz_TextChanged(object sender, EventArgs e)
        {

        }

        private void textBox_iyy_TextChanged(object sender, EventArgs e)
        {

        }

        private void textBox_iyz_TextChanged(object sender, EventArgs e)
        {

        }

        private void textBox_izz_TextChanged(object sender, EventArgs e)
        {

        }

        private void textBox_visual_origin_x_TextChanged(object sender, EventArgs e)
        {

        }

        private void textBox_visual_origin_y_TextChanged(object sender, EventArgs e)
        {

        }

        private void textBox_visual_origin_z_TextChanged(object sender, EventArgs e)
        {

        }

        private void textBox_visual_origin_roll_TextChanged(object sender, EventArgs e)
        {

        }

        private void textBox_visual_origin_pitch_TextChanged(object sender, EventArgs e)
        {

        }

        private void textBox_visual_origin_yaw_TextChanged(object sender, EventArgs e)
        {

        }

        private void trackBar_visual_Scroll(object sender, EventArgs e)
        {

        }

        private void comboBox_materials_SelectedIndexChanged(object sender, EventArgs e)
        {

        }

        private void domainUpDown_red_SelectedItemChanged(object sender, EventArgs e)
        {

        }

        private void domainUpDown_green_SelectedItemChanged(object sender, EventArgs e)
        {

        }

        private void domainUpDown_blue_SelectedItemChanged(object sender, EventArgs e)
        {

        }

        private void domainUpDown_alpha_SelectedItemChanged(object sender, EventArgs e)
        {

        }

        private void textBox_texture_TextChanged(object sender, EventArgs e)
        {

        }

        private void button_texturebrowse_Click(object sender, EventArgs e)
        {

        }

        private void textBox_collision_origin_x_TextChanged(object sender, EventArgs e)
        {

        }

        private void textBox_collision_origin_y_TextChanged(object sender, EventArgs e)
        {

        }

        private void textBox_collision_origin_z_TextChanged(object sender, EventArgs e)
        {

        }

        private void textBox_collision_origin_roll_TextChanged(object sender, EventArgs e)
        {

        }

        private void textBox_collision_origin_pitch_TextChanged(object sender, EventArgs e)
        {

        }

        private void textBox_collision_origin_yaw_TextChanged(object sender, EventArgs e)
        {

        }

        private void trackBar_collision_Scroll(object sender, EventArgs e)
        {

        }

        private void button_finish_Click(object sender, EventArgs e)
        {
            mPartExporter.exportLink();
            this.Close();
        }

        private void button_cancel_Click(object sender, EventArgs e)
        {
            this.Close();
        }

        private void PartExportForm_Load(object sender, EventArgs e)
        {
            textBox_collision_origin_x.Text = mPartExporter.mLink.origin_collision[0].ToString();
            textBox_collision_origin_y.Text = mPartExporter.mLink.origin_collision[1].ToString();
            textBox_collision_origin_z.Text = mPartExporter.mLink.origin_collision[2].ToString();
            textBox_collision_origin_roll.Text = "0";
            textBox_collision_origin_pitch.Text = "0";
            textBox_collision_origin_yaw.Text = "0";

            textBox_visual_origin_x.Text = mPartExporter.mLink.origin_visual[0].ToString();
            textBox_visual_origin_y.Text = mPartExporter.mLink.origin_visual[1].ToString();
            textBox_visual_origin_z.Text = mPartExporter.mLink.origin_visual[2].ToString();
            textBox_visual_origin_roll.Text = "0";
            textBox_visual_origin_pitch.Text = "0";
            textBox_visual_origin_yaw.Text = "0";

            textBox_inertial_origin_x.Text = mPartExporter.mLink.origin_inertial[0].ToString();
            textBox_inertial_origin_y.Text = mPartExporter.mLink.origin_inertial[1].ToString();
            textBox_inertial_origin_z.Text = mPartExporter.mLink.origin_inertial[2].ToString();
            textBox_inertial_origin_roll.Text = "0";
            textBox_inertial_origin_pitch.Text = "0";
            textBox_inertial_origin_yaw.Text = "0";

            textBox_mass.Text = mPartExporter.mLink.mass.ToString();

            textBox_ixx.Text = mPartExporter.mLink.moment[0].ToString();
            textBox_ixy.Text = mPartExporter.mLink.moment[1].ToString();
            textBox_ixz.Text = mPartExporter.mLink.moment[2].ToString();
            textBox_iyy.Text = mPartExporter.mLink.moment[4].ToString();
            textBox_iyz.Text = mPartExporter.mLink.moment[5].ToString();
            textBox_izz.Text = mPartExporter.mLink.moment[8].ToString();

        }
    }
}
