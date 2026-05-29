# Manual testing guide

How to load this fork's add-in into SolidWorks and verify the changes on this
branch. Everything here needs SolidWorks installed locally (it is a COM add-in).

## 1. Build

Open `SW2URDF.sln` in Visual Studio and build the **Debug** configuration, or
from a shell:

```powershell
$msb = "C:\Program Files\Microsoft Visual Studio\18\Community\MSBuild\Current\Bin\MSBuild.exe"
& $msb SW2URDF\SW2URDF.csproj /p:Configuration=Debug /p:SolutionDir="$PWD\"
```

Output: `SW2URDF\bin\Debug\SW2URDF.dll`.

## 2. Register the add-in (needs administrator)

SolidWorks loads the add-in from `HKEY_CLASSES_ROOT`, so registration needs
admin rights. Two options:

- **Visual Studio**: launch VS *as Administrator*, then build — the post-build
  step runs RegAsm for you. To debug, set the SW2URDF project's
  Debug > Start external program to
  `C:\Program Files\SOLIDWORKS Corp\SOLIDWORKS\SLDWORKS.exe` and press F5.
- **Script** (build normally, register once): from any shell run
  ```powershell
  pwsh -File scripts\register_addin.ps1
  ```
  It self-elevates. Use `-Unregister` to remove it, `-Configuration Release`
  for a release build.

Then start (or restart) SolidWorks — "SW2URDF Exporter" appears in the task pane.

## 3. Test models

Sample assemblies live under `examples/`:

| Model | Notes |
|-------|-------|
| `3_DOF_ARM` | simple serial arm |
| `4_WHEELER` | branched tree |
| `ORIGINAL_3_DOF_ARM` | **has nested subassemblies** (`Arm_link.SLDASM`) — use for depth tests |
| `TOY_BLOCK` | single part |

## 4. What to verify per change

### #1 — ROS 2 output (`feat(ros2)`)
1. Open an example assembly, launch the exporter, build the tree, click
   *Preview and Export*.
2. In the export form, set **ROS Version = ROS 2**, finish the export.
3. In the generated package, confirm:
   - `package.xml` has `format="3"`, `<buildtool_depend>ament_cmake</buildtool_depend>`
     and `<build_type>ament_cmake</build_type>`.
   - `CMakeLists.txt` uses `find_package(ament_cmake REQUIRED)` + `ament_package()`.
   - `launch/` contains `display.launch.py` and `gazebo.launch.py` (Python launch).
4. Re-export with **ROS Version = ROS 1** and confirm the old catkin
   `package.xml`/`CMakeLists.txt` + `display.launch`/`gazebo.launch` are unchanged.
5. Optional end-to-end: `colcon build` the ROS 2 package, then
   `ros2 launch <pkg> display.launch.py`.

### #2 — STL coordinate fix (`fix(stl)`, issues #87 / #116)
1. Use a model whose link coordinate system is **not** at the assembly origin
   (offset/rotated). `ORIGINAL_3_DOF_ARM` is a good candidate.
2. Export with **Mesh Format = STL**.
3. Load the resulting URDF + meshes in RViz (or Isaac). The link meshes should
   sit in the right place. Before the fix they were displaced.
4. Sanity check: export the same model as **3dxml** and confirm the meshes line
   up the same way (STL should now match 3dxml).

### #3 / #4 — full-depth search + name filter (`feat(export)`)
> Currently enabled programmatically via
> `ExportHelper.SetSearchNestedReferenceGeometry(true)` /
> `SetReferenceGeometryFilter("...")` (no UI toggle yet — that's a follow-up).
1. With a model that has a coordinate system **inside a nested subassembly**
   (depth ≥ 2), confirm that with the flag **off** it is *not* listed, and with
   it **on** it *is* listed in the coordinate-system dropdown.
2. With the filter set to a substring, confirm only matching names are listed.
3. **Gate**: pick a depth-2 coordinate system, export, and confirm the link is
   placed correctly. This validates that `Component2.Transform2` is
   root-relative at depth ≥ 2 — the open question from the #0 analysis. If the
   link is misplaced, depth resolution needs the parent-chain composition (a
   larger change).

## 5. Automated tests (optional)

`TestRunner` runs the xunit integration tests (they launch SolidWorks and open
the example models). It loads `SW2URDF\bin\x64\Debug\SW2URDF.dll`, so build the
**x64 Debug** configuration first, then run `TestRunner.exe`
(optionally `TestRunner.exe TestExportHelper` to filter by class). Note these
tests only check structure/link counts, not mesh alignment.
