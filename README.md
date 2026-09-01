# TDXDepthCameraMerger

A TouchDesigner component that merges point clouds from several depth cameras into one coordinate
space. It estimates the rigid transform between cameras with [Open3D](http://www.open3d.org/),
using FPFH features plus RANSAC for the global alignment and ICP for the refinement.

Version 0.2.0. Built and tested on TouchDesigner **2025.33070**.

Supported sources:

- Kinect Azure
- Orbbec
- any pair of TOPs you supply yourself (a point cloud TOP and a colour TOP), so recorded clouds,
  other sensors and synthetic data all work

## If you are coming from 0.0.3, read this first

0.2.0 is a breaking change, and the most important part is the install.

**Do not put Open3D inside TouchDesigner.** The old README told you to copy it into
`TouchDesigner/bin/Lib/site-packages`, or to pip install it into TD's Python. On current builds
that hard-crashes the process the moment the component imports it. Not an exception you can catch,
the whole of TouchDesigner disappears. Open3D ships its own OpenMP and TBB next to the ones
TouchDesigner already loaded, and the two do not coexist.

So 0.2.0 runs the registration **outside** TouchDesigner. You point the component at any Python
that has Open3D installed, it dumps the clouds to disk, runs a worker process, and reads back a
4x4 matrix. TouchDesigner itself only ever imports numpy. A nice side effect is that the component
no longer cares which Python TouchDesigner ships.

Parameters that changed:

| 0.0.3 | 0.2.0 |
|---|---|
| `Gather kinects` | `Gather devices` |
| `Use player for calibration` | `Use mask for calibration` |
| `Specify pair x` / `Specify pair y` (XYZW) | `Specify pair`, now two plain integer fields |
| `Current pair`, `Number of pairs`, `As intermediary` | gone, replaced by `Reference device` and the calibration table |
| (none) | `Device type`, `Python exe`, `Check worker`, `Voxel size`, `Refine voxel`, `Max range`, `Use coloured ICP`, `RANSAC seed`, `Preset matrix DAT`, `Rebuild chain`, `Reset calibration`, and the `Last ...` readouts |

The [walkthrough video](https://vimeo.com/501525725) still shows the 0.0.3 interface. The idea is
the same, the parameter names are not.

## Installation

1. Drop `TDXDepthCameraMerger.tox` into your project.

2. Get a Python 3.11 or newer with Open3D in it. Anything works as long as it is not TouchDesigner's
   own interpreter. Conda is the least painful:

   ```
   conda create -n td python=3.11
   conda activate td
   pip install open3d numpy
   ```

   Or with plain pip, in a virtualenv:

   ```
   python -m venv td-open3d
   td-open3d\Scripts\pip install open3d numpy
   ```

3. On the **Configuration** page, set **Python exe** to that interpreter's `python.exe`, then pulse
   **Check worker**. The **Worker status** field should come back with something like
   `OK: open3d 0.19.0 on python 3.11.13 (numpy 2.3.3)`. If it does not, nothing else will work, so
   fix this before going further.

Developed against Open3D 0.19.0. Nothing in the worker is exotic, so nearby versions should be
fine, but that is the one I test.

## Using it

1. **Configuration** page: pick your **Device type**, then pulse **Gather devices**. The component
   builds one `Device<n>` for each camera it finds and wires up its source TOPs.

2. **Calibration** page: set **Specify pair** to the two cameras. The first field is the target,
   which does not move; the second is the source, which gets transformed onto it. The **IDs**
   field shows which physical devices those numbers refer to.

3. Pulse **Calibrate**. With the default **Mode** the component runs the global registration and
   then seeds ICP with its result, in one pass.

4. Repeat for each remaining pair, chaining outward from the reference camera. Calibrate 1 and 2,
   then 2 and 3, then 3 and 4. Every pairwise result is stored, and the component composes the
   chain so all clouds land in the frame of **Reference device**.

5. Read the merged result from the `out_points` and `out_colors` outputs.

Pulse **Refine** to run ICP again on an existing calibration. It never falls back to a fresh global
registration, so refine means refine. **Reset calibration** puts every device back at identity.
**Rebuild chain** recomposes the transforms without re-registering anything, which is what you want
after changing the reference device.

### Reading the result

After each calibration you get **Last status**, **Last fitness**, **Last RMSE** and **Last
correspondences**.

- `OK` means it converged and the numbers look sane.
- `WARN` means it converged but the overlap or the error is marginal. Look at the merged cloud
  before trusting it.
- `FAIL` means fitness under 0.05 or fewer than 100 corresponding points. The result is noise.

A confident-looking alignment from a bad global stage is the failure mode to watch for, so when a
chained run grades the global stage as `FAIL` the component skips the ICP rather than polishing a
wrong answer into a convincing one.

## Parameters

### Configuration

| Parameter | What it does |
|---|---|
| Device type | `Kinect Azure`, `Orbbec` or `Custom TOPs`. Changing it rebuilds every device's source TOPs |
| Gather devices | Find the connected cameras and build a `Device<n>` for each |
| Number of devices, Devices | Read-only, what was found |
| Python exe | The interpreter that runs the registration. Must have Open3D |
| Check worker | Probe that interpreter and report its versions |
| Worker status | Result of the probe |

### Calibration

| Parameter | Default | What it does |
|---|---|---|
| Specify pair | | Target and source device numbers. The target does not move |
| IDs | | Read-only, the physical devices behind those numbers |
| Mode | Global + ICP refine | `Global + ICP refine` chains both stages in one worker run. `Global registration only` stops after RANSAC so you can inspect it. `From table DAT` takes a 4x4 you produced elsewhere |
| Calibrate | | Run the current mode on the current pair |
| Refine | | ICP on top of the existing calibration for that pair |
| Reference device | 1 | Whose frame everything ends up in |
| Voxel size (m) | 0.05 | Downsampling for the global stage. Bigger is faster and coarser |
| Refine voxel (m) | 0.01 | Downsampling for ICP. 0 uses full resolution |
| Max range (m) | 0 | Drop points beyond this distance. 0 keeps everything |
| Use coloured ICP | off | Coloured ICP. Needs colour on both clouds and wants good overlap |
| Use mask for calibration | off | Register only the masked region. See below |
| RANSAC seed | -1 | Leave it at -1 and RANSAC uses every core, but the same scene can give a slightly different calibration each time. Set it to 0 or more and you get the same answer every run, at the cost of that parallel speedup: Open3D's RANSAC threads race, so reproducibility means running it on one thread |
| Preset matrix DAT | | A 4x4 table DAT, used when Mode is `From table DAT` |
| Rebuild chain | | Recompose all transforms without re-registering |
| Reset calibration | | Forget everything, all devices back to identity |

**Use mask for calibration** restricts registration to a masked part of the cloud, which helps when
the cameras see a lot of static background. It only works where there is a mask to use. The Kinect
Azure select TOP publishes a player index, so it is available there. The Orbbec select TOP does not
publish one, so on Orbbec the toggle is greyed out. With custom sources it becomes available as
soon as at least one row supplies a mask TOP.

## Outputs

The component has two TOP outputs:

- `out_points`, every device's transformed cloud tiled into one texture. RGB carries XYZ in the
  reference frame
- `out_colors`, the matching colours in the same layout

Feed both to instancing, or to whatever you were going to do with the cloud. You do not need to
reach inside the component.

## Custom TOP sources

Set **Device type** to `Custom TOPs` and fill in the `customSources` table inside the component,
one row per device:

| name | pointcloud | color | mask |
|---|---|---|---|
| cam1 | `/project1/clouds/cloud1` | `/project1/clouds/color1` | |
| cam2 | `/project1/clouds/cloud2` | `/project1/clouds/color2` | |

Then pulse **Gather devices** and calibrate as usual. The colour TOP should be the same resolution
as its point cloud TOP, otherwise the two will not line up. Leave `mask` empty if you do not have
one.

This is also how you add a camera the component does not know about yet. If your sensor has a
TouchDesigner TOP that can output a point cloud, adding a row to the `deviceTypes` table is enough,
no code involved.

## Limitations

The technique needs real overlap between cameras, seen from broadly similar angles. Cameras placed
at more than about 90 degrees to each other, and certainly cameras facing each other, do not give
RANSAC enough common structure to work with.

You can get around this by calibrating through intermediate positions, and the chain composition in
0.2.0 does that part for you: calibrate 1 to 2 and 2 to 3, and camera 3 lands in camera 1's frame
even though 1 and 3 never saw the same thing. Move a camera through a path of overlapping views
and you can reach positions a direct pair could not.

The registration is a one-off calibration step, not a per-frame operation. A global registration on
real cameras takes seconds.

## Development

| Path | What |
|---|---|
| `src/` | the four Python files that become DATs in the component |
| `tests/` | the two suites that need no TouchDesigner |
| `tools/` | build and test scripts that run inside TouchDesigner |

`src/worker.py` is the part that runs outside TouchDesigner. It is a standalone script and you can
drive it by hand:

```
python src/worker.py --probe
python src/worker.py job.json
```

The job format is documented at the top of that file. The matrix it returns satisfies
`x_target = M @ x_source` with homogeneous column vectors.

To run the tests:

```
python tests/test_pipeline.py                    # numpy only, no Open3D, no TouchDesigner
python tests/test_worker.py path/to/python.exe   # drives the worker as a subprocess
```

`test_pipeline.py` deliberately asserts that the TouchDesigner side never imports Open3D, since
that is the thing that crashes the process.

## Support

You can follow me on:

[Instagram](https://www.instagram.com/darien.brito/) |
[Twitter](https://twitter.com/DarienBrito)

If you would like to go one step further with your support, you can subscribe here:
[Patreon](https://www.patreon.com/c/darienbrito)

Best,
Darien

Darien Brito, 2026. MIT licensed, see [LICENSE.md](LICENSE.md).
