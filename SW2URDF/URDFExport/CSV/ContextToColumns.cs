using System.Collections.Specialized;

namespace SW2URDF.URDFExport.CSV
{
    public static class ContextToColumns
    {
        public static readonly string KEY_NAME = "Link Name";
        public static readonly string KEY_PARENT_LINK = "Parent";

        /// <summary>
        /// Provides both the column order of CSV files that are written, but also a lookup between the
        /// URDF name of field and a common sense name
        /// </summary>
        public static OrderedDictionary Dictionary = new OrderedDictionary()
        {
            { "Link.name", KEY_NAME },
            { "Link.Inertial.Origin.xyz.x", "Center of Mass X" },
            {"Link.Inertial.Origin.xyz.y",  "Center of Mass Y" },
            {"Link.Inertial.Origin.xyz.z", "Center of Mass Z" },
            {"Link.Inertial.Origin.rpy.r", "Center of Mass Roll" },
            {"Link.Inertial.Origin.rpy.p", "Center of Mass Pitch" },
            {"Link.Inertial.Origin.rpy.y", "Center of Mass Yaw" },
            {"Link.Inertial.Mass.value", "Mass"},
            {"Link.Inertial.Inertia.ixx", "Moment Ixx"},
            {"Link.Inertial.Inertia.ixy", "Moment Ixy"},
            {"Link.Inertial.Inertia.ixz", "Moment Ixz"},
            {"Link.Inertial.Inertia.iyy", "Moment Iyy"},
            {"Link.Inertial.Inertia.iyz", "Moment Iyz"},
            {"Link.Inertial.Inertia.izz", "Moment Izz"},
            {"Link.Visual.Origin.xyz.x", "Visual X"},
            {"Link.Visual.Origin.xyz.y",  "Visual Y" },
            {"Link.Visual.Origin.xyz.z",  "Visual Z"},
            {"Link.Visual.Origin.rpy.r",  "Visual Roll"},
            {"Link.Visual.Origin.rpy.p",  "Visual Pitch"},
            {"Link.Visual.Origin.rpy.y",  "Visual Yaw"},
            {"Link.Visual.Geometry.Mesh.filename", "Mesh Filename"},
            {"Link.Visual.Material.Color.rgba.r", "Color Red"},
            {"Link.Visual.Material.Color.rgba.g", "Color Green"},
            {"Link.Visual.Material.Color.rgba.b", "Color Blue"},
            {"Link.Visual.Material.Color.rgba.a", "Color Alpha"},
            {"Link.Collision.Origin.xyz.x", "Collision X"},
            {"Link.Collision.Origin.xyz.y", "Collision Y"},
            {"Link.Collision.Origin.xyz.z", "Collision Z"},
            {"Link.Collision.Origin.rpy.r", "Collision Roll"},
            {"Link.Collision.Origin.rpy.p", "Collision Pitch"},
            {"Link.Collision.Origin.rpy.y", "Collision Yaw"},
            {"Link.Collision.Geometry.Mesh.filename","Collision Mesh Filename"},
            {"Link.Visual.Material.name","Material Name"},
            {"Link.SWComponents","SW Components"},
            { "Link.CoordSysName","Coordinate System"},
            { "Link.AxisName","Axis Name"},
            { "Link.Joint.name","Joint Name"},
            { "Link.Joint.type","Joint Type"},
            { "Link.Joint.Origin.xyz.x","Joint Origin X"},
            { "Link.Joint.Origin.xyz.y","Joint Origin Y"},
            { "Link.Joint.Origin.xyz.z","Joint Origin Z"},
            { "Link.Joint.Origin.rpy.r","Joint Origin Roll"},
            { "Link.Joint.Origin.rpy.p","Joint Origin Pitch"},
            { "Link.Joint.Origin.rpy.y","Joint Origin Yaw"},
            { "Link.Joint.ParentLink.link",KEY_PARENT_LINK},
            { "Link.Joint.Axis.xyz.x", "Joint Axis X"},
            { "Link.Joint.Axis.xyz.y", "Joint Axis Y"},
            { "Link.Joint.Axis.xyz.z", "Joint Axis Z"},
            { "Link.Joint.Limit.effort", "Limit Effort"},
            { "Link.Joint.Limit.velocity", "Limit Velocity"},
            { "Link.Joint.Limit.lower", "Limit Lower"},
            { "Link.Joint.Limit.upper", "Limit Upper"},
            { "Link.Joint.Calibration.rising", "Calibration rising"},
            { "Link.Joint.Calibration.falling", "Calibration falling" },
            { "Link.Joint.Dynamics.damping", "Dynamics Damping" },
            { "Link.Joint.Dynamics.friction", "Dynamics Friction" },
            { "Link.Joint.SafetyController.soft_upper" ,"Safety Soft Upper"},
            { "Link.Joint.SafetyController.soft_lower", "Safety Soft Lower" },
            { "Link.Joint.SafetyController.k_position","Safety K Position"},
            { "Link.Joint.SafetyController.k_velocity","Safety K Velocity"},
        };
    }
}