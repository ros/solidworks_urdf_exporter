using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Windows.Forms;
using System.IO;

using SolidWorks.Interop.sldworks;

namespace SW2URDF
{
    public partial class PartExportForm : Form
    {
        public SW2URDFExporter Exporter;
        public PartExportForm(ISldWorks iSwApp)
        {
            InitializeComponent();
            Exporter = new SW2URDFExporter(iSwApp);
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
            SaveFileDialog saveFileDialog1 = new SaveFileDialog();
            saveFileDialog1.RestoreDirectory = true;
            saveFileDialog1.InitialDirectory = Path.GetDirectoryName(textBox_save_as.Text);
            saveFileDialog1.FileName = Path.GetFileName(textBox_save_as.Text);
            if (saveFileDialog1.ShowDialog() == DialogResult.OK)
            {
                textBox_save_as.Text = saveFileDialog1.FileName;
            }
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
            OpenFileDialog openFileDialog1 = new OpenFileDialog();
            openFileDialog1.RestoreDirectory = true;
            openFileDialog1.InitialDirectory = Path.GetDirectoryName(textBox_save_as.Text);
            if (openFileDialog1.ShowDialog() == DialogResult.OK)
            {
                textBox_texture.Text = openFileDialog1.FileName;
            }
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
            Exporter.mPackageName = Path.GetFileName(textBox_save_as.Text);
            Exporter.mSavePath = Path.GetDirectoryName(textBox_save_as.Text);
            Exporter.mRobot.BaseLink.name = Exporter.mPackageName;

            Exporter.mRobot.BaseLink.Inertial.Origin.update(textBox_inertial_origin_x, 
                                                            textBox_inertial_origin_y, 
                                                            textBox_inertial_origin_z, 
                                                            textBox_inertial_origin_roll,
                                                            textBox_inertial_origin_pitch,
                                                            textBox_inertial_origin_yaw);

            Exporter.mRobot.BaseLink.Inertial.Inertia.update(textBox_ixx, 
                                                             textBox_ixy, 
                                                             textBox_ixz, 
                                                             textBox_iyy,
                                                             textBox_iyz, 
                                                             textBox_izz);


            Exporter.mRobot.BaseLink.Inertial.Mass.update(textBox_mass);

            Exporter.mRobot.BaseLink.Visual.Origin.update(textBox_visual_origin_x, 
                                                          textBox_visual_origin_y, 
                                                          textBox_visual_origin_z, 
                                                          textBox_visual_origin_roll, 
                                                          textBox_visual_origin_pitch, 
                                                          textBox_visual_origin_yaw);


            Exporter.mRobot.BaseLink.Visual.Material.name = comboBox_materials.Text;
            Exporter.mRobot.BaseLink.Visual.Material.Texture.wFilename = textBox_texture.Text;

            Exporter.mRobot.BaseLink.Visual.Material.Color.update(domainUpDown_red, 
                                                                  domainUpDown_green, 
                                                                  domainUpDown_blue, 
                                                                  domainUpDown_alpha);

            Exporter.mRobot.BaseLink.Collision.Origin.update(textBox_collision_origin_x, 
                                                             textBox_collision_origin_y, 
                                                             textBox_collision_origin_z, 
                                                             textBox_collision_origin_roll,
                                                             textBox_collision_origin_pitch, 
                                                             textBox_collision_origin_yaw);

            Exporter.mRobot.BaseLink.STLQualityFine = radioButton_fine.Checked;

            Exporter.exportLink(checkBox_rotate.Checked);
            this.Close();
        }

        private void button_cancel_Click(object sender, EventArgs e)
        {
            this.Close();
        }

        private void PartExportForm_Load(object sender, EventArgs e)
        {
            Exporter.createRobotFromActiveModel();
            textBox_save_as.Text = Exporter.mSavePath + "\\" + Exporter.mPackageName;

            Exporter.mRobot.BaseLink.Visual.Origin.fillBoxes(textBox_collision_origin_x, 
                                                             textBox_collision_origin_y, 
                                                             textBox_collision_origin_z, 
                                                             textBox_collision_origin_roll,
                                                             textBox_collision_origin_pitch, 
                                                             textBox_collision_origin_yaw, 
                                                             "G5");

            Exporter.mRobot.BaseLink.Visual.Origin.fillBoxes(textBox_visual_origin_x, 
                                                             textBox_visual_origin_y,
                                                             textBox_visual_origin_z, 
                                                             textBox_visual_origin_roll,
                                                             textBox_visual_origin_pitch, 
                                                             textBox_visual_origin_yaw,
                                                             "G5");

            Exporter.mRobot.BaseLink.Visual.Material.Color.fillBoxes(domainUpDown_red, 
                                                                     domainUpDown_green, 
                                                                     domainUpDown_blue, 
                                                                     domainUpDown_alpha, 
                                                                     "G5");

            Exporter.mRobot.BaseLink.Inertial.Origin.fillBoxes(textBox_inertial_origin_x, 
                                                               textBox_inertial_origin_y, 
                                                               textBox_inertial_origin_z,
                                                               textBox_inertial_origin_roll,
                                                               textBox_inertial_origin_pitch, 
                                                               textBox_inertial_origin_yaw, 
                                                               "G5");
            
            Exporter.mRobot.BaseLink.Inertial.Mass.fillBoxes(textBox_mass, "G5");

            Exporter.mRobot.BaseLink.Inertial.Inertia.fillBoxes(textBox_ixx, 
                                                                textBox_ixy,
                                                                textBox_ixz,
                                                                textBox_iyy, 
                                                                textBox_iyz, 
                                                                textBox_izz, 
                                                                "G5");
        }
        #endregion

    }
}
