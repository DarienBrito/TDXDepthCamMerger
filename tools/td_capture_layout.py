"""
Capture the master's network layout to tools/build/layout.json.

Run from TD (MCP execute_code):

    exec(open(r'<this file>', encoding='utf-8').read())

td_build.py rebuilds the component from control.tox, which knows nothing about
where anything was put. The arrangement and the annotate boxes are hand made, so
they live in a data file that the build applies at the end. Run this after any
hand rearrange, or the next build hands the old positions back.

Captured: every top level node, Device1's children, and the annotate boxes in
full. NOT captured: Device2 and up, whose positions extUtilities computes from
the template when a device is gathered.
"""

import json
import os

REPO = globals().get('REPO') or project.folder.replace(os.sep, '/')
BUILD = REPO + '/tools/build'
OUT = BUILD + '/layout.json'

# Same guard as td_build.py: a wrong REPO would write the layout into the wrong
# repository, and nothing downstream would notice.
if not os.path.isfile(REPO + '/src/worker.py'):
	raise RuntimeError(
		'no sources under {}/src. Open the project from the repo root, or set '
		'REPO before the exec.'.format(REPO))

NAME = 'TDXDepthCamMerger'
comp = op('/ProjectName/' + NAME)
if comp is None:
	raise RuntimeError('no /ProjectName/' + NAME + ' to capture')

# Clones are placed by extUtilities.MakeDevices, at template.nodeY - 200 * (n-1).
# Capturing them would bake whatever rig happens to be gathered right now into
# the shipped layout.
CLONES = lambda name: name.startswith('Device') and name[6:].isdigit() and name != 'Device1'

nodes = {}
for child in comp.findChildren(depth=1):
	if child.type == 'annotate' or CLONES(child.name):
		continue
	nodes[child.name] = [int(child.nodeX), int(child.nodeY)]

# Device1 is the template every clone is copied from, so its insides are part of
# the shipped layout. Its POP chain is placed by extUtilities.BuildDeviceSources
# and comes back the same every time, but capturing it costs nothing and covers
# transformMatrix, which nothing else owns.
device = comp.op('Device1')
if device is not None:
	for child in device.findChildren(depth=1):
		nodes['Device1/' + child.name] = [int(child.nodeX), int(child.nodeY)]

annotates = []
for box in comp.findChildren(type=annotateCOMP, maxDepth=1):
	annotates.append({
		'name': box.name,
		'title': box.par.Titletext.eval(),
		'body': box.par.Bodytext.eval(),
		'mode': box.par.Mode.eval(),
		'x': int(box.nodeX), 'y': int(box.nodeY),
		'w': int(box.nodeWidth), 'h': int(box.nodeHeight),
		'fontsize': float(box.par.Bodyfontsize.eval()),
		'back': [round(float(box.par[p].eval()), 4) for p in
			('Backcolorr', 'Backcolorg', 'Backcolorb', 'Backcoloralpha')],
	})
annotates.sort(key=lambda a: a['name'])

with open(OUT, 'w', encoding='utf-8', newline='\n') as handle:
	json.dump({'nodes': nodes, 'annotates': annotates}, handle,
		indent='\t', sort_keys=True, ensure_ascii=False)
	handle.write('\n')

print('captured {} nodes and {} annotate boxes to {}'.format(
	len(nodes), len(annotates), OUT))
for box in annotates:
	print('    {:<16} {:<20} {}x{} at ({}, {}), body {} chars'.format(
		box['name'], repr(box['title']), box['w'], box['h'], box['x'], box['y'],
		len(box['body'])))
