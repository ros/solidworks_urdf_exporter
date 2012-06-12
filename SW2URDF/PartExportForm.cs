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

        #region Basic event handelers
        private void textBox_name_TextChanged(object sender, EventArgs e)
        {

        }

        private void checkBox_static_CheckedChanged(object sender, EventArgs e)
        {

        }
        private void saveFileDialog_save_as_FileOk(object sender, CancelEventArgs e)
        {

        }

        private void textBox_save_as_TextChanged(object sender, EventArgs e)
        {

        }

        private void button_savename_browse_Click(object sender, EventArgs e)
        {

        }
        #endregion

        #region Inertial event handelers
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
        #endregion

        #region Visual event handlers
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
        #endregion
        
        #region Collision event handlers
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
        #endregion


        #region Form event handlers
        private void button_finish_Click(object sender, EventArgs e)
        {
            
            double value;

            mPartExporter.mLink.name = textBox_name.Text;

            mPartExporter.mLink.origin_inertial[0] = (Double.TryParse(textBox_inertial_origin_x.Text, out value)) ? value : 0;
            mPartExporter.mLink.origin_inertial[1] = (Double.TryParse(textBox_inertial_origin_y.Text, out value)) ? value : 0;
            mPartExporter.mLink.origin_inertial[2] = (Double.TryParse(textBox_inertial_origin_z.Text, out value)) ? value : 0;
            mPartExporter.mLink.origin_inertial[3] = (Double.TryParse(textBox_inertial_origin_roll.Text, out value)) ? value : 0;
            mPartExporter.mLink.origin_inertial[4] = (Double.TryParse(textBox_inertial_origin_pitch.Text, out value)) ? value : 0;
            mPartExporter.mLink.origin_inertial[5] = (Double.TryParse(textBox_inertial_origin_yaw.Text, out value)) ? value : 0;

            mPartExporter.mLink.origin_visual[0] = (Double.TryParse(textBox_visual_origin_x.Text, out value)) ? value : 0;
            mPartExporter.mLink.origin_visual[1] = (Double.TryParse(textBox_visual_origin_y.Text, out value)) ? value : 0;
            mPartExporter.mLink.origin_visual[2] = (Double.TryParse(textBox_visual_origin_z.Text, out value)) ? value : 0;
            mPartExporter.mLink.origin_visual[3] = (Double.TryParse(textBox_visual_origin_roll.Text, out value)) ? value : 0;
            mPartExporter.mLink.origin_visual[4] = (Double.TryParse(textBox_visual_origin_pitch.Text, out value)) ? value : 0;
            mPartExporter.mLink.origin_visual[5] = (Double.TryParse(textBox_visual_origin_yaw.Text, out value)) ? value : 0;

            mPartExporter.mLink.origin_collision[0] = (Double.TryParse(textBox_collision_origin_x.Text, out value)) ? value : 0;
            mPartExporter.mLink.origin_collision[1] = (Double.TryParse(textBox_collision_origin_y.Text, out value)) ? value : 0;
            mPartExporter.mLink.origin_collision[2] = (Double.TryParse(textBox_collision_origin_z.Text, out value)) ? value : 0;
            mPartExporter.mLink.origin_collision[3] = (Double.TryParse(textBox_collision_origin_roll.Text, out value)) ? value : 0;
            mPartExporter.mLink.origin_collision[4] = (Double.TryParse(textBox_collision_origin_pitch.Text, out value)) ? value : 0;
            mPartExporter.mLink.origin_collision[5] = (Double.TryParse(textBox_collision_origin_yaw.Text, out value)) ? value : 0;

            mPartExporter.mLink.mass = (Double.TryParse(textBox_mass.Text, out value)) ? value : 0;

            mPartExporter.mLink.moment[0] = (Double.TryParse(textBox_ixx.Text, out value)) ? value : 0;
            mPartExporter.mLink.moment[1] = (Double.TryParse(textBox_ixy.Text, out value)) ? value : 0;
            mPartExporter.mLink.moment[2] = (Double.TryParse(textBox_ixz.Text, out value)) ? value : 0;
            mPartExporter.mLink.moment[4] = (Double.TryParse(textBox_iyy.Text, out value)) ? value : 0;
            mPartExporter.mLink.moment[5] = (Double.TryParse(textBox_iyz.Text, out value)) ? value : 0;
            mPartExporter.mLink.moment[8] = (Double.TryParse(textBox_izz.Text, out value)) ? value : 0;

            mPartExporter.mLink.material = comboBox_materials.Text;
            mPartExporter.mLink.texture = textBox_texture.Text;

            mPartExporter.mLink.rgba[0] = (Double.TryParse(domainUpDown_red.Text, out value)) ? value : 0;
            mPartExporter.mLink.rgba[1] = (Double.TryParse(domainUpDown_green.Text, out value)) ? value : 0;
            mPartExporter.mLink.rgba[2] = (Double.TryParse(domainUpDown_blue.Text, out value)) ? value : 0;
            mPartExporter.mLink.rgba[4] = (Double.TryParse(domainUpDown_alpha.Text, out value)) ? value : 0;

            mPartExporter.mLink.gazebo_static = checkBox_static.Checked;
            mPartExporter.mSavePath = textBox_save_as.Text;
        //public int mesh_triangles_visual
        //{ get; set; }
        //public int mesh_triangles_collision
        //{ get; set; }

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
        #endregion



    }
}
