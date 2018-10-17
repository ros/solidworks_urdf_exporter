/*
Copyright (c) 2015 Stephen Brawner

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
*/

using SolidWorks.Interop.sldworks;
using SW2URDF.URDFExport;
using System;
using System.IO;
using System.Windows.Forms;

namespace SW2URDF.UI
{
    public partial class PartExportForm : Form
    {
        public ExportHelper Exporter;

        public PartExportForm(SldWorks iSwApp)
        {
            InitializeComponent();
            Exporter = new ExportHelper(iSwApp);
        }

        #region Basic event handelers

        private void ButtonSaveNameBrowseClick(object sender, EventArgs e)
        {
            SaveFileDialog saveFileDialog1 = new SaveFileDialog
            {
                RestoreDirectory = true,
                InitialDirectory = Path.GetDirectoryName(textBox_save_as.Text),
                FileName = Path.GetFileName(textBox_save_as.Text)
            };

            if (saveFileDialog1.ShowDialog() == DialogResult.OK)
            {
                textBox_save_as.Text = saveFileDialog1.FileName;
            }
        }

        #endregion Basic event handelers

        private void ButtonTextureBrowseClick(object sender, EventArgs e)
        {
            OpenFileDialog openFileDialog1 = new OpenFileDialog
            {
                RestoreDirectory = true,
                InitialDirectory = Path.GetDirectoryName(textBox_save_as.Text)
            };

            if (openFileDialog1.ShowDialog() == DialogResult.OK)
            {
                textBox_texture.Text = openFileDialog1.FileName;
            }
        }

        #region Form event handlers

        private void ButtonFinishClick(object sender, EventArgs e)
        {
            Exporter.PackageName = Path.GetFileName(textBox_save_as.Text);
            Exporter.SavePath = Path.GetDirectoryName(textBox_save_as.Text);
            Exporter.URDFRobot.BaseLink.Name = Exporter.PackageName;

            Exporter.URDFRobot.BaseLink.Inertial.Origin.Update(textBox_inertial_origin_x,
                                                            textBox_inertial_origin_y,
                                                            textBox_inertial_origin_z,
                                                            textBox_inertial_origin_roll,
                                                            textBox_inertial_origin_pitch,
                                                            textBox_inertial_origin_yaw);

            Exporter.URDFRobot.BaseLink.Inertial.Inertia.Update(textBox_ixx,
                                                             textBox_ixy,
                                                             textBox_ixz,
                                                             textBox_iyy,
                                                             textBox_iyz,
                                                             textBox_izz);

            Exporter.URDFRobot.BaseLink.Inertial.Mass.Update(textBox_mass);

            Exporter.URDFRobot.BaseLink.Visual.Origin.Update(textBox_visual_origin_x,
                                                          textBox_visual_origin_y,
                                                          textBox_visual_origin_z,
                                                          textBox_visual_origin_roll,
                                                          textBox_visual_origin_pitch,
                                                          textBox_visual_origin_yaw);

            Exporter.URDFRobot.BaseLink.Visual.Material.Name = comboBox_materials.Text;
            Exporter.URDFRobot.BaseLink.Visual.Material.Texture.wFilename = textBox_texture.Text;

            Exporter.URDFRobot.BaseLink.Visual.Material.Color.Update(domainUpDown_red,
                                                                  domainUpDown_green,
                                                                  domainUpDown_blue,
                                                                  domainUpDown_alpha);

            Exporter.URDFRobot.BaseLink.Collision.Origin.Update(textBox_collision_origin_x,
                                                             textBox_collision_origin_y,
                                                             textBox_collision_origin_z,
                                                             textBox_collision_origin_roll,
                                                             textBox_collision_origin_pitch,
                                                             textBox_collision_origin_yaw);

            Exporter.URDFRobot.BaseLink.STLQualityFine = radioButton_fine.Checked;

            Exporter.ExportLink(checkBox_rotate.Checked);
            Close();
        }

        private void ButtonCancelClick(object sender, EventArgs e)
        {
            Close();
        }

        private void PartExportFormLoad(object sender, EventArgs e)
        {
            Exporter.CreateRobotFromActiveModel();
            textBox_save_as.Text = Exporter.SavePath + "\\" + Exporter.PackageName;

            Exporter.URDFRobot.BaseLink.Visual.Origin.FillBoxes(textBox_collision_origin_x,
                                                             textBox_collision_origin_y,
                                                             textBox_collision_origin_z,
                                                             textBox_collision_origin_roll,
                                                             textBox_collision_origin_pitch,
                                                             textBox_collision_origin_yaw,
                                                             "G5");

            Exporter.URDFRobot.BaseLink.Visual.Origin.FillBoxes(textBox_visual_origin_x,
                                                             textBox_visual_origin_y,
                                                             textBox_visual_origin_z,
                                                             textBox_visual_origin_roll,
                                                             textBox_visual_origin_pitch,
                                                             textBox_visual_origin_yaw,
                                                             "G5");

            Exporter.URDFRobot.BaseLink.Visual.Material.Color.FillBoxes(domainUpDown_red,
                                                                     domainUpDown_green,
                                                                     domainUpDown_blue,
                                                                     domainUpDown_alpha,
                                                                     "G5");

            Exporter.URDFRobot.BaseLink.Inertial.Origin.FillBoxes(textBox_inertial_origin_x,
                                                               textBox_inertial_origin_y,
                                                               textBox_inertial_origin_z,
                                                               textBox_inertial_origin_roll,
                                                               textBox_inertial_origin_pitch,
                                                               textBox_inertial_origin_yaw,
                                                               "G5");

            Exporter.URDFRobot.BaseLink.Inertial.Mass.FillBoxes(textBox_mass, "G5");

            Exporter.URDFRobot.BaseLink.Inertial.Inertia.FillBoxes(textBox_ixx,
                                                                textBox_ixy,
                                                                textBox_ixz,
                                                                textBox_iyy,
                                                                textBox_iyz,
                                                                textBox_izz,
                                                                "G5");
        }

        #endregion Form event handlers
    }
}