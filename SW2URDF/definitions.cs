using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace SW2URDF
{
    public class joint
    {
        public joint()
        {
        }
        public string name
        { get; set; }
        public int type
        { get; set; }
        public double[] origin
        { get; set; }
        public string parent
        { get; set; }
        public string child
        { get; set; }
        public double[] axis
        { get; set; }
        public double calibration_rising
        { get; set; }
        public double calibration_falling
        { get; set; }
        public double damping
        { get; set; }
        public double friction
        { get; set; }
        public double[] limits
        { get; set; }
        public double[] safety_controller
        { get; set; }
    }

    //public class link
    //{
    //    public link()
    //    {
    //    }
    //    public string name
    //    { get; set; }
    //    public string meshName
    //    { get; set; }
    //    public double[] origin_inertial
    //    { get; set; }
    //    public double[] origin_visual
    //    { get; set; }
    //    public double[] origin_collision
    //    { get; set; }
    //    public double mass
    //    { get; set; }
    //    public double[] moment
    //    { get; set; }
    //    public string material
    //    { get; set; }
    //    public double[] rgba
    //    { get; set; }
    //    public int mesh_triangles_visual
    //    { get; set; }
    //    public int mesh_triangles_collision
    //    { get; set; }
    //    public string texture
    //    { get; set; }
    //    public bool gazebo_static
    //    { get; set; }
    //}
}
